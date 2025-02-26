use gmt_dos_actors::{
    actor::PlainActor,
    framework::{
        model::SystemFlowChart,
        network::{AddActorOutput, AddOuput, TryIntoInputs},
    },
    prelude::Actor,
    system::{Sys, System},
};
use gmt_dos_clients_fem::DiscreteModalSolver;
use gmt_dos_clients_io::{
    gmt_m1::assembly,
    gmt_m2::{
        asm::{M2ASMFluidDampingForces, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion},
        M2PositionerForces, M2PositionerNodes,
    },
    mount::{MountEncoders, MountTorques},
};
use gmt_dos_clients_m1_ctrl::assembly::M1;
use gmt_dos_clients_m2_ctrl::{AsmsPositioners, ASMS};
use gmt_dos_clients_mount::Mount;

use serde::{Deserialize, Serialize};

use crate::FemSolver;

pub mod io;
pub mod traits;

#[derive(Clone, Serialize, Deserialize)]
pub struct GmtServoMechanisms<const M1_RATE: usize, const M2_RATE: usize = 1> {
    pub fem: Actor<DiscreteModalSolver<FemSolver>>,
    pub mount: Actor<Mount>,
    pub m1: Sys<M1<M1_RATE>>,
    pub m2_positioners: Actor<AsmsPositioners>,
    pub m2: Sys<ASMS<1>>,
}

impl<const M1_RATE: usize, const M2_RATE: usize> System for GmtServoMechanisms<M1_RATE, M2_RATE> {
    fn name(&self) -> String {
        format!("GMT Servo-Mechanisms")
    }
    fn build(&mut self) -> anyhow::Result<&mut Self> {
        self.mount
            .add_output()
            .build::<MountTorques>()
            .into_input(&mut self.fem)?;
        self.fem
            .add_output()
            .bootstrap()
            .build::<MountEncoders>()
            .into_input(&mut self.mount)?;

        self.m1
            .add_output()
            .build::<assembly::M1HardpointsForces>()
            .into_input(&mut self.fem)?;
        self.fem
            .add_output()
            .bootstrap()
            .build::<assembly::M1HardpointsMotion>()
            .into_input(&mut self.m1)?;

        self.m1
            .add_output()
            .build::<assembly::M1ActuatorAppliedForces>()
            .into_input(&mut self.fem)?;

        self.m2_positioners
            .add_output()
            .build::<M2PositionerForces>()
            .into_input(&mut self.fem)?;
        self.fem
            .add_output()
            .bootstrap()
            .build::<M2PositionerNodes>()
            .into_input(&mut self.m2_positioners)?;

        self.m2
            .add_output()
            .build::<M2ASMVoiceCoilsForces>()
            .into_input(&mut self.fem)?;
        self.m2
            .add_output()
            .build::<M2ASMFluidDampingForces>()
            .into_input(&mut self.fem)?;
        self.fem
            .add_output()
            .bootstrap()
            .build::<M2ASMVoiceCoilsMotion>()
            .into_input(&mut self.m2)?;

        Ok(self)
    }
    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut plain = PlainActor::default();
        plain.client = self.name();
        plain.inputs_rate = 1;
        plain.outputs_rate = 1;

        plain.inputs = PlainActor::from(&self.fem)
            /*             .filter_inputs_by_name(&[
                "MountTorques",
                "M1HardpointsForces",
                "M1ActuatorAppliedForces",
                "M2PositionerForces",
                "M2ASMVoiceCoilsForces",
                "M2ASMFluidDampingForces",
            ]) */
            .inputs
            .map(|input| {
                input
                    .into_iter()
                    .filter(|input| {
                        input.filter_by_name(&[
                            "MountTorques",
                            "M1HardpointsForces",
                            "M1ActuatorAppliedForces",
                            "M2PositionerForces",
                            "M2ASMVoiceCoilsForces",
                            "M2ASMFluidDampingForces",
                        ])
                    })
                    .collect::<Vec<_>>()
            })
            .zip(PlainActor::from(&self.mount).inputs.map(|input| {
                input
                    .into_iter()
                    .filter(|input| input.filter(|x| x.name.contains("MountSetPoint")))
                    .collect::<Vec<_>>()
            }))
            .zip(PlainActor::from(&self.m2_positioners).inputs.map(|input| {
                input
                    .into_iter()
                    .filter(|input| input.filter(|x| x.name.contains("M2RigidBodyMotions")))
                    .collect::<Vec<_>>()
            }))
            .zip(PlainActor::from(&self.m1.dispatch_in).inputs.map(|input| {
                input
                    .into_iter()
                    .filter(|input| input.filter(|x| !x.name.contains("M1HardpointsMotion")))
                    .collect::<Vec<_>>()
            }))
            .zip(PlainActor::from(&self.m2.dispatch_in).inputs.map(|input| {
                input
                    .into_iter()
                    .filter(|input| input.filter(|x| !x.name.contains("M2ASMVoiceCoilsMotion")))
                    .collect::<Vec<_>>()
            }))
            .map(|((((mut fem, mount), m2_pos), m1), m2)| {
                fem.extend(mount);
                fem.extend(m2_pos);
                fem.extend(m1);
                fem.extend(m2);
                fem
            });
        plain.outputs = PlainActor::from(&self.fem).outputs.map(|output| {
            output
                .into_iter()
                .filter(|output| {
                    output.filter(|x| {
                        !(x.name.contains("MountEncoders")
                            || x.name.contains("M1HardpointsMotion")
                            || x.name.contains("M2PositionerNodes")
                            || x.name.contains("M2ASMVoiceCoilsMotion"))
                    })
                })
                .collect::<Vec<_>>()
        });
        plain.image = Some("gmt-servos.png".to_string());
        plain.graph = self.graph();
        plain
    }
}
