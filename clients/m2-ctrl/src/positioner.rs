use gmt_dos_clients_fem::{Model, Switch};
use gmt_dos_clients_io::gmt_m2::{M2PositionerForces, M2PositionerNodes, M2RigidBodyMotions};
use gmt_fem::FEM;
use interface::{
    Data, Read, Update, Write,
    optics::{M2State, state::SegmentState},
};
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, thiserror::Error)]
pub enum PositionersError {
    #[error("cannot create positionners model")]
    Positionners(#[from] gmt_fem::FemError),
}

#[cfg(topend = "ASM")]
type M2Positioner = gmt_m2_ctrl_asm_positionner::AsmPositionner;
#[cfg(not(topend = "ASM"))]
type M2Positioner = gmt_m2_ctrl_fsm_positionner::FsmPositionner;

/// Positionners control system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Positioners {
    // Reference bodies rigid body motions to positioners displacements 42x42 transform
    r2p: na::SMatrix<f64, 42, 42>,
    // Positioner dynamics
    positionners: Vec<M2Positioner>,
    // Rigid body motions
    rbm: na::SVector<f64, 42>,
    // Positioner nodes displacement
    nodes: Vec<f64>,
}

impl Positioners {
    /// Create a new positionners control system from a FEM model
    pub fn new(fem: &mut FEM) -> std::result::Result<Self, PositionersError> {
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        let hex_f2d = {
            let hex_f2d = fem
                .switch_inputs_by_name(vec!["MC_M2_SmHex_F"], Switch::On)
                .and_then(|fem| fem.switch_outputs_by_name(vec!["MC_M2_SmHex_D"], Switch::On))
                .map(|fem| {
                    fem.reduced_static_gain()
                        .unwrap_or_else(|| fem.static_gain())
                })?;
            let left =
                na::DMatrix::from_columns(&hex_f2d.column_iter().step_by(2).collect::<Vec<_>>());
            let right = na::DMatrix::from_columns(
                &hex_f2d.column_iter().skip(1).step_by(2).collect::<Vec<_>>(),
            );
            let hex_f2d = left - right;
            let left = na::DMatrix::from_rows(&hex_f2d.row_iter().step_by(2).collect::<Vec<_>>());
            let right =
                na::DMatrix::from_rows(&hex_f2d.row_iter().skip(1).step_by(2).collect::<Vec<_>>());
            left - right
        };

        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        let hex_f_2_rb_d = {
            let hex_f_2_rb_d = fem
                .switch_inputs_by_name(vec!["MC_M2_SmHex_F"], Switch::On)
                .and_then(|fem| {
                    fem.switch_outputs_by_name(
                        vec![if cfg!(topend = "ASM") {
                            "MC_M2_RB_6D"
                        } else {
                            "MC_M2_lcl_6D"
                        }],
                        Switch::On,
                    )
                })
                .map(|fem| {
                    fem.reduced_static_gain()
                        .unwrap_or_else(|| fem.static_gain())
                })?;
            let left = na::DMatrix::from_columns(
                &hex_f_2_rb_d.column_iter().step_by(2).collect::<Vec<_>>(),
            );
            let right = na::DMatrix::from_columns(
                &hex_f_2_rb_d
                    .column_iter()
                    .skip(1)
                    .step_by(2)
                    .collect::<Vec<_>>(),
            );
            left - right
        };

        let mat = hex_f2d
            * hex_f_2_rb_d
                .try_inverse()
                .expect("failed to inverse the positioners forces to rigid0body-motions matrix");
        let r2p = na::SMatrix::<f64, 42, 42>::from_iterator(mat.into_iter().map(|x| *x));

        fem.switch_inputs(Switch::On, None)
            .switch_outputs(Switch::On, None);
        fem.switch_inputs(Switch::On, None)
            .switch_outputs(Switch::On, None);

        Ok(Self {
            r2p,
            positionners: (0..42).map(|_| M2Positioner::new()).collect(),
            rbm: na::SVector::zeros(),
            nodes: vec![0f64; 84],
        })
    }
}

impl Update for Positioners {
    fn update(&mut self) {
        let pos = &self.r2p * &self.rbm;
        let deltas = pos
            .into_iter()
            .zip(&self.nodes)
            .map(|(pos, node)| pos - node);

        self.positionners
            .iter_mut()
            .zip(deltas)
            .for_each(|(positionner, delta)| {
                positionner.inputs.M2pAct_E = delta;
                positionner.step();
            });
    }
}

impl Read<M2RigidBodyMotions> for Positioners {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.rbm = na::SVector::<f64, 42>::from_column_slice(&data);
    }
}
impl Read<M2State> for Positioners {
    fn read(&mut self, data: Data<M2State>) {
        let rbms: Vec<_> = data
            .iter()
            .flat_map(|segment| {
                if let Some(SegmentState {
                    rbms: Some(rbms), ..
                }) = segment
                {
                    rbms.as_ref().to_vec()
                } else {
                    vec![0f64; 6]
                }
            })
            .collect();
        self.rbm = na::SVector::<f64, 42>::from_column_slice(&rbms);
    }
}

impl Read<M2PositionerNodes> for Positioners {
    fn read(&mut self, data: Data<M2PositionerNodes>) {
        self.nodes = data.chunks(2).map(|x| x[0] - x[1]).collect();
    }
}

impl Write<M2PositionerForces> for Positioners {
    fn write(&mut self) -> Option<Data<M2PositionerForces>> {
        Some(
            self.positionners
                .iter()
                .map(|positionner| positionner.outputs.M2pAct_U)
                .flat_map(|x| vec![x, -x])
                .collect::<Vec<f64>>()
                .into(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
    use gmt_dos_clients_io::gmt_fem::{
        inputs::MCM2SmHexF,
        outputs::{MCM2Lcl6D, MCM2SmHexD},
    };

    // cargo t -r--package gmt_dos-clients_m2-ctrl --lib -- positioner::tests::controller --exact --nocapture
    #[test]
    fn controller() -> std::result::Result<(), Box<dyn std::error::Error>> {
        let mut fem = gmt_fem::FEM::from_env().unwrap();

        let mut positioners = Positioners::new(&mut fem).unwrap();

        #[cfg(topend = "FSM")]
        let mut plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(1e3)
            .proportional_damping(2. / 100.)
            .ins::<MCM2SmHexF>()
            .outs::<MCM2SmHexD>()
            .use_static_gain_compensation()
            .outs::<MCM2Lcl6D>()
            .build()?;
        #[cfg(topend = "ASM")]
        let mut plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(8e3)
            .proportional_damping(2. / 100.)
            .ins::<MCM2SmHexF>()
            .outs::<MCM2SmHexD>()
            .use_static_gain_compensation()
            .outs::<gmt_dos_clients_io::gmt_fem::outputs::MCM2RB6D>()
            .build()?;

        let mut cmd = vec![0f64; 42];
        cmd[0] = 1e-6;
        let mut i = 0;
        let rbm = loop {
            <Positioners as Read<M2RigidBodyMotions>>::read(&mut positioners, cmd.clone().into());

            <Positioners as Update>::update(&mut positioners);

            let data: Data<M2PositionerForces> =
                <Positioners as Write<M2PositionerForces>>::write(&mut positioners).unwrap();
            <DiscreteModalSolver<_> as Read<M2PositionerForces>>::read(&mut plant, data);

            <DiscreteModalSolver<_> as Update>::update(&mut plant);

            let data =
                <DiscreteModalSolver<_> as Write<M2PositionerNodes>>::write(&mut plant).unwrap();

            #[cfg(topend = "FSM")]
            let rbm =
                <DiscreteModalSolver<_> as Write<M2RigidBodyMotions>>::write(&mut plant).unwrap();
            #[cfg(topend = "ASM")]
            let rbm = <DiscreteModalSolver<_> as Write<
                gmt_dos_clients_io::gmt_fem::outputs::MCM2RB6D,
            >>::write(&mut plant)
            .unwrap();

            if i > 24_000 {
                // dbg!(&rbm);
                break rbm;
            }

            <Positioners as Read<M2PositionerNodes>>::read(&mut positioners, data);

            i += 1;
        };
        assert!((rbm[0] - 1e-6).abs() < 1e-3);

        Ok(())
    }
}
