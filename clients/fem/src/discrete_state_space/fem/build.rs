use crate::{solvers::Solver, DiscreteModalSolver, DiscreteStateSpace, StateSpaceError};

type Result<T> = std::result::Result<T, StateSpaceError>;

impl<'a, T: Solver + Default> DiscreteStateSpace<'a, T> {
    #[cfg(not(feature = "cuda"))]
    pub fn build(self) -> Result<DiscreteModalSolver<T>> {
        self.builder()
    }
    pub fn builder(mut self) -> Result<DiscreteModalSolver<T>> {
        use std::{f64::consts::PI, sync::Arc};

        let tau = self.sampling.map_or(
            Err(StateSpaceError::MissingArguments("sampling".to_owned())),
            |x| Ok(1f64 / x),
        )?;

        let (w, n_modes, zeta, _n_io) = self.properties()?;

        match (self.in2mode(n_modes), self.mode2out(n_modes)) {
            (Some(forces_2_modes), Some(modes_2_nodes)) => {
                log::info!("forces 2 modes: {:?}", forces_2_modes.shape());
                log::info!("modes 2 nodes: {:?}", modes_2_nodes.shape());

                let (w_ss, state_space): (Vec<_>, Vec<_>) =
                    match self.hankel_singular_values_threshold {
                        Some(hsv_t) => (0..n_modes)
                            .filter_map(|k| {
                                let b = forces_2_modes.row(k).clone_owned();
                                let c = modes_2_nodes.column(k);
                                let hsv = Self::hankel_singular_value(
                                    w[k],
                                    zeta[k],
                                    b.as_slice(),
                                    c.as_slice(),
                                );
                                if w[k]
                                    < self
                                        .hankel_frequency_lower_bound
                                        .map(|x| 2. * PI * x)
                                        .unwrap_or_default()
                                {
                                    Some((
                                        w[k],
                                        T::from_second_order(
                                            tau,
                                            w[k],
                                            zeta[k],
                                            b.as_slice().to_vec(),
                                            c.as_slice().to_vec(),
                                        ),
                                    ))
                                } else {
                                    if hsv > hsv_t {
                                        Some((
                                            w[k],
                                            T::from_second_order(
                                                tau,
                                                w[k],
                                                zeta[k],
                                                b.as_slice().to_vec(),
                                                c.as_slice().to_vec(),
                                            ),
                                        ))
                                    } else {
                                        None
                                    }
                                }
                            })
                            .unzip(),
                        None => (0..n_modes)
                            .map(|k| {
                                let b = forces_2_modes.row(k).clone_owned();
                                let c = modes_2_nodes.column(k);
                                (
                                    w[k],
                                    T::from_second_order(
                                        tau,
                                        w[k],
                                        zeta[k],
                                        b.as_slice().to_vec(),
                                        c.as_slice().to_vec(),
                                    ),
                                )
                            })
                            .unzip(),
                    };
                let psi_dcg = if self.use_static_gain {
                    log::info!(
                        "The elements of psi_dcg corresponding to 
    - OSSAzDriveTorque
    - OSSElDriveTorque
    - OSSRotDriveTorque
and
    - OSSAzEncoderAngle
    - OSSElEncoderAngle
    - OSSRotEncoderAngle
are set to zero."
                    );
                    self.dc_gain_compensator(&state_space, w_ss)
                } else {
                    None
                };
                Ok(DiscreteModalSolver {
                    u: vec![0f64; forces_2_modes.ncols()],
                    y: vec![0f64; modes_2_nodes.nrows()],
                    state_space,
                    ins: self.ins,
                    outs: self.outs,
                    psi_dcg: psi_dcg.map(|psi_dcg| Arc::new(psi_dcg)),
                    facesheet_nodes: self.facesheet_nodes,
                    m1_figure_nodes: self.m1_figure_nodes,
                    ..Default::default()
                })
            }
            (Some(_), None) => Err(StateSpaceError::Matrix(
                "Failed to build modes to nodes transformation matrix".to_string(),
            )),
            (None, Some(_)) => Err(StateSpaceError::Matrix(
                "Failed to build forces to nodes transformation matrix".to_string(),
            )),
            _ => Err(StateSpaceError::Matrix(
                "Failed to build both modal transformation matrices".to_string(),
            )),
        }
    }
}
#[cfg(not(feature = "cuda"))]
impl<'a, S> TryFrom<DiscreteStateSpace<'a, S>> for DiscreteModalSolver<S>
where
    S: Solver + Default,
{
    type Error = StateSpaceError;

    fn try_from(dss: DiscreteStateSpace<'a, S>) -> std::result::Result<Self, Self::Error> {
        dss.build()
    }
}
