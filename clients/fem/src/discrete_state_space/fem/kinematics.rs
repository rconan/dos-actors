use crate::{DiscreteStateSpace, KinematicCompensation, StateSpaceError, solvers::Solver};
use nalgebra as na;

type Result<T> = std::result::Result<T, StateSpaceError>;

impl<'a, T: Solver + Default> DiscreteStateSpace<'a, T> {
    /// Computes the kinematic compensation from the DC gain of the state space model
    /// from the mount drive torques
    #[cfg(mount)]
    pub(crate) fn kinematic_compensator(
        &self,
        forces_2_modes: &na::DMatrix<f64>,
        modes_2_nodes: &na::DMatrix<f64>,
        w: &[f64],
    ) -> Result<KinematicCompensation> {
        use crate::fem_io::{self, SplitFem};

        log::info!("computing the mount kinematic compensation");
        let missing = |name: &str| {
            StateSpaceError::MissingArguments(format!(
                "{name} is required for the kinematic compensation"
            ))
        };
        let in_range = |name: &str, is: &dyn Fn(&dyn std::any::Any) -> bool| {
            self.ins
                .iter()
                .find(|x| is(x.as_any()))
                .map(|x| x.range())
                .ok_or_else(|| missing(name))
        };
        let out_range = |name: &str, is: &dyn Fn(&dyn std::any::Any) -> bool| {
            self.outs
                .iter()
                .find(|x| is(x.as_any()))
                .map(|x| x.range())
                .ok_or_else(|| missing(name))
        };
        let torques = [
            in_range("OSSAzDriveTorque", &|x| {
                x.is::<SplitFem<fem_io::actors_inputs::OSSAzDriveTorque>>()
            })?,
            in_range("OSSElDriveTorque", &|x| {
                x.is::<SplitFem<fem_io::actors_inputs::OSSElDriveTorque>>()
            })?,
            in_range("OSSRotDriveTorque", &|x| {
                x.is::<SplitFem<fem_io::actors_inputs::OSSRotDriveTorque>>()
            })?,
        ];
        let encoders = [
            out_range("OSSAzEncoderAngle", &|x| {
                x.is::<SplitFem<fem_io::actors_outputs::OSSAzEncoderAngle>>()
            })?,
            out_range("OSSElEncoderAngle", &|x| {
                x.is::<SplitFem<fem_io::actors_outputs::OSSElEncoderAngle>>()
            })?,
            out_range("OSSRotEncoderAngle", &|x| {
                x.is::<SplitFem<fem_io::actors_outputs::OSSRotEncoderAngle>>()
            })?,
        ];
        let n_modes = forces_2_modes.nrows();
        // modal forces per drive torque scaled by 1/ω²: [n_modes x n_torques]
        let b = na::DMatrix::from_columns(
            &torques
                .iter()
                .flat_map(|r| r.clone())
                .map(|j| {
                    let mut c = forces_2_modes.column(j).clone_owned();
                    c.iter_mut().zip(w).for_each(|(c, w)| *c /= w * w);
                    c
                })
                .collect::<Vec<_>>(),
        );
        // DC gain from the drive torques to the outputs: [n_outputs x n_torques]
        let h = modes_2_nodes.columns(0, n_modes) * b;
        // DC gain from the drive torques to the averaged encoders: [3 x n_torques]
        let e = na::DMatrix::from_rows(
            &encoders
                .iter()
                .map(|r| h.rows(r.start, r.len()).row_mean())
                .collect::<Vec<_>>(),
        );
        let e_pinv = e.transpose()
            * (&e * e.transpose()).try_inverse().ok_or_else(|| {
                StateSpaceError::Matrix("singular mount encoders DC gain".to_string())
            })?;
        let mut k = h * e_pinv;
        // the mount encoders are left untouched
        for r in &encoders {
            k.rows_mut(r.start, r.len()).fill(0.);
        }
        // and so are the excluded outputs
        for exclusion in &self.kinematic_exclusions {
            if let Some(r) = self
                .outs
                .iter()
                .find(|x| (exclusion.is_output)(x.as_any()))
                .map(|x| x.range())
            {
                log::info!("{} excluded from the kinematic compensation", exclusion.name);
                k.rows_mut(r.start, r.len()).fill(0.);
            }
        }
        // calibrated gains
        for calibration in &self.kinematic_calibrations {
            let r = out_range(&calibration.name, &*calibration.is_output)?;
            if calibration.gain.len() != r.len() {
                return Err(StateSpaceError::Matrix(format!(
                    "the calibrated kinematic gain of {} has {} elements instead of {}",
                    calibration.name,
                    calibration.gain.len(),
                    r.len()
                )));
            }
            log::info!(
                "calibrated kinematic gain for {} (axis #{})",
                calibration.name,
                calibration.axis
            );
            k.view_mut((r.start, calibration.axis), (r.len(), 1))
                .iter_mut()
                .zip(&calibration.gain)
                .for_each(|(k, g)| *k = *g);
        }
        Ok(KinematicCompensation {
            gain: k.transpose().as_slice().to_vec(),
            encoders,
        })
    }
    #[cfg(not(mount))]
    pub(crate) fn kinematic_compensator(
        &self,
        _forces_2_modes: &na::DMatrix<f64>,
        _modes_2_nodes: &na::DMatrix<f64>,
        _w: &[f64],
    ) -> Result<KinematicCompensation> {
        Err(StateSpaceError::MissingArguments(
            "the kinematic compensation requires a FEM with the mount".to_string(),
        ))
    }
}
