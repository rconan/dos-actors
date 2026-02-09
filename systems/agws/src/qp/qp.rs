use std::{fs::File, io::BufReader, path::Path};

use gmt_dos_clients_crseo::calibration::{Calib, MixedMirrorMode, algebra::CalibProps};
use nalgebra::{DMatrix, SMatrix};
use osqp::{CscMatrix, Problem, Settings};
use serde::Deserialize;

use super::{active_optics::ActiveOptics, QpError};

/*
/// Data structure for the quadratic programming algorithm
#[derive(Deserialize)]
struct QPData {
    /// Controllable mode regularization matrix
    #[serde(rename = "W2")]
    w2: Vec<f64>,
    /// Control balance weighting matrix
    #[serde(rename = "W3")]
    w3: Vec<f64>,
    /// Controller gain
    #[serde(rename = "K")]
    k: f64,
    /// Objective function factor
    rho_3: f64,
}*/
#[derive(Deserialize)]
struct QPData {
    #[serde(rename = "D")]
    dmat: Vec<f64>,
    #[serde(rename = "W2")]
    w2: Vec<f64>,
    #[serde(rename = "W3")]
    w3: Vec<f64>,
    #[serde(rename = "K")]
    k: f64,
    //#[serde(rename = "wfsMask")]
    //wfs_mask: Vec<Vec<bool>>,
    //umin: Vec<f64>,
    //umax: Vec<f64>,
    //rm_mean_slopes: bool,
    #[serde(rename = "_Tu")]
    tu: Vec<f64>,
    rho_3: f64,
    end2end_ordering: bool,
}
/// Quadratic programming stucture
///
/// It requires 4 generic constants:
///  - `M1_RBM`: the number of controlled M1 segment rigid body motions (at most 41 as S7 Rz is a blind mode)
///  - `M2_RBM`: the number of controlled M2 segment rigid body motions (at most 41 as S7 Rz is a blind mode)
///  - `M1_BM` : the number of controlled M1 segment eigen modes
///  - `N_MODE` = M1_RBM + M2_RBM + 7 * M1_BM
#[derive(Deserialize)]
//pub struct QP<'a, const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize>
pub struct QP<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize> {
    /// Quadratic programming data
    #[serde(rename = "SHAcO_qp")]
    data: QPData,
    /*
    /// calibration matrix (column wise) as (data,n_cols)
    #[serde(skip)]
    dmat: (&'a [f64], usize),
    /// segment bending modes coefficients to segment actuators forces  (column wise) as ([data],[n_rows])
    #[serde(skip)]
    coefs2forces: (&'a [Vec<f64>], Vec<usize>),
    */
    /// OSQP verbosity
    #[serde(skip)]
    verbose: bool,
    /// convert bending modes coefficients to forces if true
    #[serde(skip)]
    #[allow(unused)]
    m1_actuator_forces_outputs: bool,
    /// OSQP convergence tolerances (absolute=1e-8,relative=1e-6)
    #[serde(skip)]
    convergence_tolerances: (f64, f64),
    #[serde(skip)]
    calib: Option<Calib<MixedMirrorMode>>,
}
impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize>
    QP<M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    /// Creates a new quadratic programming object
    pub fn new(
        qp_filename: impl AsRef<Path>,
        //calib_matrix: (&'a [f64], usize),
        //coefs2forces: (&'a [Vec<f64>], Vec<usize>),
    ) -> Result<Self, QpError> {
        assert!(
            M1_RBM + M2_RBM + 7 * M1_BM == N_MODE,
            "The number of modes {} do not match the expected value {} (M1_RBM + M2_RBM + 7 * M1_BM)x",
            N_MODE,
            M1_RBM + M2_RBM + 7 * M1_BM
        );
        let file = File::open(&qp_filename).map_err(|e| QpError::Open {
            filename: qp_filename.as_ref().to_str().unwrap().to_string(),
            source: e,
        })?;
        let rdr = BufReader::with_capacity(10_000, file);
        let this: Self = serde_pickle::from_reader(rdr, Default::default())?;
        Ok(Self {
            //dmat: calib_matrix,
            //coefs2forces,
            verbose: false,
            m1_actuator_forces_outputs: false,
            convergence_tolerances: (1.0e-8, 1.0e-6),
            ..this
        })
    }
    pub fn update_dmat(mut self, path: impl AsRef<Path>) -> Result<Self, QpError> {
        let file = File::open(&path).map_err(|e| QpError::Open {
            filename: path.as_ref().to_str().unwrap().to_string(),
            source: e,
        })?;
        let buffer = BufReader::new(file);
        self.data.dmat = serde_pickle::from_reader(buffer, Default::default())?;
        Ok(self)
    }
    pub fn update_calib(mut self, path: impl AsRef<Path>) -> Result<Self, QpError> {
        let file = File::open(&path).map_err(|e| QpError::Open {
            filename: path.as_ref().to_str().unwrap().to_string(),
            source: e,
        })?;
        let buffer = BufReader::new(file);
        let calib: Calib<MixedMirrorMode> = serde_pickle::from_reader(buffer, Default::default())?;
        self.data.dmat = calib
            .mat_ref()
            .col_iter()
            .flat_map(|c| c.iter().cloned().collect::<Vec<_>>())
            .collect();
        self.calib = Some(calib);
        Ok(self)
    }
    /// Sets OSQP verbosity
    pub fn verbose(self) -> Self {
        Self {
            verbose: true,
            ..self
        }
    }
    /// Outputs the forces of M1 actuators instead of M1 bending modes coefficients
    pub fn as_m1_actuator_forces(self) -> Self {
        Self {
            m1_actuator_forces_outputs: true,
            ..self
        }
    }
    /*
    /// Computes the control balance weighting matrix
    fn balance_weighting(&self) -> DynMatrix {
        let (data, n_actuators) = &self.coefs2forces;
        let n_actuators_sum = n_actuators.iter().sum::<usize>();
        let fz = 10e-5;
        let coefs2forces: Vec<_> = data
            .iter()
            .zip(n_actuators.iter())
            .map(|(c2f, &n)| DMatrix::from_column_slice(n, c2f.len() / n, c2f) * fz)
            .collect();
        //    println!("coefs2forces: {:?}", coefs2force.shape());
        let tu_nrows = M1_RBM * M2_RBM + n_actuators_sum;
        let tu_ncols = N_MODE;
        let mut tu = DMatrix::<f64>::zeros(tu_nrows, tu_ncols);
        for (i, mut row) in tu.row_iter_mut().take(M1_RBM).enumerate() {
            row[(i)] = 1f64;
        }
        for (i, mut row) in tu.row_iter_mut().skip(M1_RBM).take(M2_RBM).enumerate() {
            row[(i + M1_RBM)] = 1f64;
        }
        let mut n_skip_row = M1_RBM + M2_RBM;
        for c2f in coefs2forces {
            for (mut tu_row, c2f_row) in tu.row_iter_mut().skip(n_skip_row).zip(c2f.row_iter()) {
                tu_row
                    .iter_mut()
                    .zip(c2f_row.iter())
                    .for_each(|(tu, &c2f)| {
                        *tu = c2f;
                    });
                n_skip_row += c2f.nrows();
            }
        }
        tu
    }
    */
    /// Sets OSQP convergence tolerances: (absolute,relative)
    pub fn convergence_tolerances(self, convergence_tolerances: (f64, f64)) -> Self {
        Self {
            convergence_tolerances,
            ..self
        }
    }
    /// Builds the quadratic programming problem
    pub fn build(self) -> Result<ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>, QpError> {
        let dmat = self.data.dmat;
        //assert!(n_mode == N_MODE,"The number of columns ({}) of the calibration matrix do not match the number of modes ({})",n_mode,N_MODE);
        // W2 and W3
        let w2 = DMatrix::from_column_slice(N_MODE, N_MODE, &self.data.w2);
        // let w3 = SMatrix::<f64, N_MODE, N_MODE>::from_column_slice(&self.data.w3);
        let w3 = DMatrix::<f64>::from_column_slice(N_MODE, N_MODE, &self.data.w3);
        // W1
        let d_wfs = DMatrix::from_column_slice(dmat.len() / N_MODE, N_MODE, &dmat);
        let d_t_w1_d = {
            let d_t_w1_d_dyn = d_wfs.tr_mul(&d_wfs);
            SMatrix::<f64, N_MODE, N_MODE>::from_vec(d_t_w1_d_dyn.as_slice().to_vec())
        };
        // Extract the upper triangular elements of `P`
        let p_utri = {
            let p = d_t_w1_d + &w2 + w3.scale(self.data.rho_3 * self.data.k * self.data.k);
            CscMatrix::from_column_iter_dense(
                p.nrows(),
                p.ncols(),
                p.as_slice().to_vec().into_iter(),
            )
            .into_upper_tri()
        };

        // Remove S7Rz from T_u matrix
        // Indices to insert (or remove) S7Rz columns of matrix Tu
        let i_m1_s7_rz: u8 = if self.data.end2end_ordering {
            41
        } else {
            (((12 + M1_BM) * 6) + 5).try_into().unwrap()
        };
        let i_m2_s7_rz: u8 = if self.data.end2end_ordering {
            // Add 1 (+1) to delete
            82 + 1
        } else {
            (((12 + M1_BM) * 6) + 10 + 1).try_into().unwrap()
        };
        let tu = DMatrix::<f64>::from_vec(273, 1228, self.data.tu)
            .transpose()
            .remove_columns_at(&[i_m1_s7_rz.into(), i_m2_s7_rz.into()]);

        //let tu = balance_weighting();

        let a_in = {
            let tus = tu.scale(self.data.k);
            CscMatrix::from(
                &tus.row_iter()
                    .map(|x| x.clone_owned().as_slice().to_vec())
                    .collect::<Vec<Vec<f64>>>(),
            )
        };

        // QP linear term
        let q: Vec<f64> = vec![0f64; N_MODE];

        // Inequality constraints
        let umin = vec![f64::NEG_INFINITY; tu.nrows()];
        let umax = vec![f64::INFINITY; tu.nrows()];

        // QP settings
        let settings = Settings::default()
            .eps_abs(self.convergence_tolerances.0)
            .eps_rel(self.convergence_tolerances.1)
            .max_iter((500 * N_MODE).try_into().unwrap())
            .warm_start(true)
            .verbose(self.verbose);

        // Create an OSQP problem
        let prob = Problem::new(p_utri, &q, a_in, &umin, &umax, &settings)?;
        Ok(ActiveOptics {
            prob,
            u: vec![0f64; 84 + 7 * M1_BM],
            y_valid: Vec::with_capacity(d_wfs.nrows()),
            d_wfs,
            u_ant: SMatrix::zeros(),
            d_t_w1_d,
            w2,
            w3,
            rho_3: self.data.rho_3,
            k: self.data.k,
            umin: vec![f64::NEG_INFINITY; tu.nrows()],
            umax: vec![f64::INFINITY; tu.nrows()],
            tu,
            calib: self.calib,
            /*
            coefs2forces: self.m1_actuator_forces_outputs.then(|| {
                let (data, n_actuators) = &self.coefs2forces;
                data.iter()
                    .zip(n_actuators)
                    .map(|(c2f, &n)| {
                        DMatrix::from_column_slice(n, c2f.len() / n, c2f)
                            .columns(0, M1_BM)
                            .into_owned()
                    })
                    .collect()
            }),
            */
        })
    }
}
