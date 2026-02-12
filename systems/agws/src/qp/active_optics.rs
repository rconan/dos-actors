use std::{convert::Infallible, fmt::Display, fs::File};

use gmt_dos_clients_crseo::calibration::{Calib, MixedMirrorMode, algebra::CalibProps};
use gmt_dos_clients_io::{Estimate, optics::SensorData};
use interface::{Data, TryRead, TryUpdate, TryWrite, optics::{OpticsState, state::{MirrorState, OpticalState}}};
use nalgebra::{DMatrix, DVector, SMatrix};
use osqp::{CscMatrix, Problem};

use super::{J1_J3_RATIO, MIN_RHO3, QpError};

pub trait DoF {
    const N_MODE: usize;
}
impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize> DoF
    for ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    const N_MODE: usize = M1_RBM + M2_RBM + M1_BM * 7;
}

pub struct ActiveOptics<
    const M1_RBM: usize,
    const M2_RBM: usize,
    const M1_BM: usize,
    const N_MODE: usize,
> {
    /// Quadratic programming problem
    pub(super) prob: Problem,
    /// Calibration matrix
    pub(super) d_wfs: DMatrix<f64>,
    /// Previous quadratic programming solution
    pub(super) u_ant: DMatrix<f64>, // N_MODE, 1>,
    /// Current quadratic programming solution
    pub(super) u: Vec<f64>,
    /// Wavefront sensor data
    pub(super) y_valid: Vec<f64>,
    /// Wavefront error weighting matrix
    pub(super) d_t_w1_d: DMatrix<f64>,
    // Matrix<f64, Const<N_MODE>, Const<N_MODE>, ArrayStorage<f64, N_MODE, N_MODE>>,
    /// Controllable mode regularization matrix    
    pub(super) w2: DMatrix<f64>,
    /// Control balance weighting matrix
    pub(super) w3: DMatrix<f64>,
    /// Objective function factor
    pub(super) rho_3: f64,
    /// Controller gain
    pub(super) k: f64,
    /// QP solution lower bound
    pub(super) umin: Vec<f64>,
    /// QP solution upper bound
    pub(super) umax: Vec<f64>,
    /// Control balance weighting matrix
    pub(super) tu: DMatrix<f64>,
    // /// segment bending modes coefficients to segment actuators forces  (column wise) as ([data],[n_rows])
    //coefs2forces: Option<Vec<DynMatrix>>,
    pub(super) calib: Option<Calib<MixedMirrorMode>>,
}

impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize> Display
    for ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Active Optics Quadratic Programming Algorithm:")?;
        writeln!(f, " * M1: RBM={M1_RBM}, M2: BM={M1_BM}")?;
        writeln!(f, " * M2: RBM={M2_RBM}")?;
        writeln!(f, " * objective function factor: {:.3e}", self.rho_3)?;
        writeln!(f, " * controller gain: {:.3e}", self.k)?;
        Ok(())
    }
}

impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize>
    ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    /// Returns AcO controller gain
    pub fn controller_gain(&self) -> f64 {
        self.k
    }

    /// Returns AcO interacion matrix (stacked) version
    pub fn get_d_wfs(&self) -> DMatrix<f64> {
        println!(
            "Cloning [{}x{}] WFS interaction matrix.",
            &self.d_wfs.nrows(),
            &self.d_wfs.ncols()
        );
        self.d_wfs.clone()
    }
}
impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize> TryUpdate
    for ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    type Error = QpError;

    fn try_update(&mut self) -> std::result::Result<&mut Self, Self::Error> {
        log::info!("updating active optics state");
        let y_vec = DVector::from_column_slice(&self.y_valid); //VectorNs::from_vec(y_valid);

        self.u_ant
            .iter_mut()
            .zip(&self.u)
            .take(M1_RBM)
            .for_each(|(u, &v)| *u = v);
        self.u_ant
            .iter_mut()
            .skip(M1_RBM)
            .zip(&self.u[42..])
            .take(M2_RBM)
            .for_each(|(u, &v)| *u = v);
        self.u_ant
            .iter_mut()
            .skip(M1_RBM + M2_RBM)
            .zip(&self.u[84..])
            .for_each(|(u, &v)| *u = v);

        // QP linear term                                                               // QP linear term
        let mut q: Vec<f64> = (-y_vec.clone_owned().tr_mul(&self.d_wfs))
            // - self.u_ant.tr_mul(&self.w3).scale(self.rho_3 * self.k))
            .as_slice()
            .to_vec();
        self.prob.update_lin_cost(&q);
        // Update bounds to inequality constraints
        let tu_u_ant: Vec<f64> = (&self.tu * &self.u_ant).as_slice().to_vec();
        let lb: Vec<f64> = tu_u_ant
            .iter()
            .zip(self.umin.iter())
            .map(|(v, w)| w - v)
            .collect();
        let ub: Vec<f64> = tu_u_ant
            .iter()
            .zip(self.umax.iter())
            .map(|(v, w)| w - v)
            .collect();
        self.prob.update_bounds(&lb, &ub);
        let mut result = self.prob.solve();
        let mut c = match result.x() {
            Some(x) => x,
            None => {
                serde_pickle::to_writer(
                    &mut File::create("OSQP_log.pkl").unwrap(),
                    &(self.y_valid.clone(), self.u_ant.as_slice().to_vec()),
                    Default::default(),
                )?;
                return Err(QpError::QpSolve);
            }
        }; //.expect("Failed to solve QP problem!");
        // Compute costs to set up the 2nd QP iteration
        let c_vec = SMatrix::<f64, N_MODE, 1>::from_vec(c.to_vec());
        let j_1na = {
            let epsilon = &y_vec - (&self.d_wfs * &c_vec);
            // Still need to account for W1
            epsilon.tr_mul(&epsilon)
        };
        // Control effort cost
        let j_3na = {
            let delta = c_vec.scale(self.k) - &self.u_ant;
            delta.tr_mul(&self.w3) * &delta
        };
        // nalgebra object to f64 scalar conversion
        let j_1 = j_1na.get(0).unwrap();
        let j_3 = j_3na.get(0).unwrap();
        //println!(" ===>>> J3: {:}:, J1: {:},RHO_3: {:}", j_3, j_1, self.rho_3);
        if *j_3 != 0f64 {
            self.rho_3 = j_1 / (j_3 * J1_J3_RATIO);
            if self.rho_3 < MIN_RHO3 {
                self.rho_3 = MIN_RHO3
            };

            // Update QP P matrix
            let p_utri = {
                //println!("New rho_3:{}", format!("{:.4e}", self.rho_3));
                let p = &self.d_t_w1_d + &self.w2 + self.w3.scale(self.rho_3 * self.k * self.k);
                CscMatrix::from_column_iter_dense(
                    p.nrows(),
                    p.ncols(),
                    p.as_slice().to_vec().into_iter(),
                )
                .into_upper_tri()
            };
            self.prob.update_P(p_utri);
            // Update QP linear term
            q = (-y_vec.clone_owned().tr_mul(&self.d_wfs)
                - self.u_ant.tr_mul(&self.w3).scale(self.rho_3 * self.k))
            .as_slice()
            .to_vec();
            self.prob.update_lin_cost(&q);

            // Solve problem - 2nd iteration
            result = self.prob.solve();
            c = match result.x() {
                Some(x) => x,
                None => {
                    serde_pickle::to_writer(
                        &mut File::create("OSQP_log.pkl").unwrap(),
                        &(self.y_valid.clone(), self.u_ant.as_slice().to_vec()),
                        Default::default(),
                    )?;
                    return Err(QpError::QpSolve);
                }
            }; //.expect("Failed to solve QP problem!");
        }
        // Control action
        // dbg!(c[0]);
        let k = 0.2; //self.k;
        self.u
            .iter_mut()
            .zip(&c[..M1_RBM])
            .for_each(|(u, c)| *u -= k * c); // u = u - k * c
        self.u[42..]
            .iter_mut()
            .zip(&c[M1_RBM..M1_RBM + M2_RBM])
            .for_each(|(u, c)| *u -= k * c);
        self.u[84..]
            .iter_mut()
            .zip(&c[M1_RBM + M2_RBM..])
            .for_each(|(u, c)| *u -= k * c);
        Ok(self)
    }
}
impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize>
    TryRead<SensorData> for ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    type Error = QpError;

    fn try_read(
        &mut self,
        data: Data<SensorData>,
    ) -> std::result::Result<&mut Self, <Self as TryRead<SensorData>>::Error> {
        self.y_valid = self
            .calib
            .as_ref()
            .ok_or_else(|| QpError::MissingCalibration)?
            .mask(&data);
        Ok(self)
    }
}
impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize>
    TryWrite<Estimate> for ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    type Error = Infallible;

    fn try_write(
        &mut self,
    ) -> std::result::Result<Option<Data<Estimate>>, <Self as TryWrite<Estimate>>::Error> {
        Ok(Some(self.u.clone().into()))
    }
}
impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize>
    TryWrite<OpticsState> for ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    type Error = Infallible;

    fn try_write(
        &mut self,
    ) -> std::result::Result<Option<Data<OpticsState>>, <Self as TryWrite<OpticsState>>::Error>
    {
        let m1 = MirrorState::new(self.u[..42].chunks(6), self.u[84..].chunks(27));
        let m2 = MirrorState::from_rbms(&self.u[42..84]);
        Ok(Some(Data::new(OpticalState::new(m1, m2))))
    }
}
