use gmt_dos_clients_fem::{Model, Switch};
use gmt_dos_clients_io::gmt_m2::{M2PositionerForces, M2PositionerNodes, M2RigidBodyMotions};
use gmt_fem::FEM;
use gmt_m2_ctrl_asm_positionner::AsmPositionner;
use interface::{Data, Read, Update, Write};
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, thiserror::Error)]
pub enum AsmsPositionersError {
    #[error("cannot create ASMS positionners model")]
    Positionners(#[from] gmt_fem::FemError),
}

/// ASMS positionners control system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AsmsPositioners {
    // Reference bodies rigid body motions to positioners displacements 42x42 transform
    r2p: na::SMatrix<f64, 42, 42>,
    // Positioner dynamics
    positionners: Vec<AsmPositionner>,
    // Rigid body motions
    rbm: na::SVector<f64, 42>,
    // Positioner nodes displacement
    nodes: Vec<f64>,
}

impl AsmsPositioners {
    /// Create a new ASMS positionners control system from a FEM model
    pub fn new(fem: &mut FEM) -> std::result::Result<Self, AsmsPositionersError> {
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
                .and_then(|fem| fem.switch_outputs_by_name(vec!["MC_M2_RB_6D"], Switch::On))
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
            positionners: (0..42).map(|_| AsmPositionner::new()).collect(),
            rbm: na::SVector::zeros(),
            nodes: vec![0f64; 84],
        })
    }
}

impl Update for AsmsPositioners {
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

impl Read<M2RigidBodyMotions> for AsmsPositioners {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.rbm = na::SVector::<f64, 42>::from_column_slice(&data);
    }
}

impl Read<M2PositionerNodes> for AsmsPositioners {
    fn read(&mut self, data: Data<M2PositionerNodes>) {
        self.nodes = data.chunks(2).map(|x| x[0] - x[1]).collect();
    }
}

impl Write<M2PositionerForces> for AsmsPositioners {
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

/* #[cfg(test)]
mod tests {
    use super::*;
    use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix, Model, Switch};
    use gmt_dos_clients_io::gmt_fem::{
        inputs::MCM2SmHexF,
        outputs::{MCM2Lcl6D, MCM2SmHexD, MCM2RB6D},
    };
    use nalgebra::SMatrix;

    //cargo test --release --package gmt_dos-clients_m2-ctrl --lib --features serde,polars -- positioner::tests::positioner_controller --exact --nocapture
    #[test]
    fn positioner_controller() -> std::result::Result<(), Box<dyn std::error::Error>> {
        let mut fem = gmt_fem::FEM::from_env().unwrap();

/*         fem.switch_inputs(Switch::Off, None)
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
                .and_then(|fem| fem.switch_outputs_by_name(vec!["MC_M2_RB_6D"], Switch::On))
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
                .expect("failed to inverse the positioners forces to displacements matrix");
        let r2p = SMatrix::<f64, 42, 42>::from_iterator(mat.into_iter().map(|x| *x));

        fem.switch_inputs(Switch::On, None)
            .switch_outputs(Switch::On, None); */

        let mut positioners = AsmsPositioners::new(&mut fem).unwrap();

        let mut plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(8e3)
            .proportional_damping(2. / 100.)
            .ins::<MCM2SmHexF>()
            .outs::<MCM2SmHexD>()
            .use_static_gain_compensation()
            .outs::<MCM2RB6D>()
            .build()?;


        let mut cmd = vec![0f64; 42];
        cmd[0] = 1e-6;
        let mut i = 0;
        loop {
            <AsmsPositioners as Read<M2RigidBodyMotions>>::read(
                &mut positioners,
                cmd.clone().into(),
            );

            <AsmsPositioners as Update>::update(&mut positioners);

            let data: Data<M2PositionerForces> =
                <AsmsPositioners as Write<M2PositionerForces>>::write(&mut positioners).unwrap();
            <DiscreteModalSolver<ExponentialMatrix> as Read<M2PositionerForces>>::read(
                &mut plant, data,
            );

            <DiscreteModalSolver<ExponentialMatrix> as Update>::update(&mut plant);

            let data = <DiscreteModalSolver<ExponentialMatrix> as Write<M2PositionerNodes>>::write(
                &mut plant,
            )
            .unwrap();

            let rbm =
                <DiscreteModalSolver<ExponentialMatrix> as Write<MCM2RB6D>>::write(&mut plant)
                    .unwrap();

            if i > 24_000 {
                dbg!(&rbm);
                break;
            }

            <AsmsPositioners as Read<M2PositionerNodes>>::read(&mut positioners, data);

            i += 1;
        }

        Ok(())
    }
}
 */