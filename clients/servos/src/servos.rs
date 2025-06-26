use gmt_dos_actors::{
    actor::{PlainActor, Terminator},
    framework::{
        model::FlowChart,
        network::{AddActorOutput, AddOuput, TryIntoInputs},
    },
    prelude::{Actor, IntoLogs, IntoLogsN},
    system::{Sys, System, SystemError},
};
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_fem::DiscreteModalSolver;
#[cfg(topend = "ASM")]
use gmt_dos_clients_io::gmt_m2::asm::{
    M2ASMFluidDampingForces, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
};
#[cfg(topend = "FSM")]
use gmt_dos_clients_io::gmt_m2::fsm::{M2FSMPiezoForces, M2FSMPiezoNodes};
use gmt_dos_clients_io::{
    gmt_m1::assembly,
    gmt_m2::{M2PositionerForces, M2PositionerNodes},
    mount::{MountEncoders, MountTorques},
};
use gmt_dos_clients_m2_ctrl::Positioners;
use gmt_dos_clients_mount::Mount;
use gmt_dos_systems_m1::assembly::M1;
use gmt_dos_systems_m2::M2;

use interface::Flatten;
use serde::{Deserialize, Serialize};

use crate::FemSolver;

pub mod io;
pub mod traits;

#[derive(Clone, Serialize, Deserialize)]
pub struct GmtServoMechanisms<const M1_RATE: usize, const M2_RATE: usize = 1> {
    pub(crate) fem: Actor<DiscreteModalSolver<FemSolver>>,
    pub(crate) mount: Actor<Mount>,
    pub(crate) m1: Sys<M1<M1_RATE>>,
    pub(crate) m2_positioners: Actor<Positioners>,
    pub(crate) m2: Sys<M2<1>>,
    #[serde(skip)]
    pub(crate) telemetry: Option<Terminator<Arrow>>,
}

impl<const M1_RATE: usize, const M2_RATE: usize> System for GmtServoMechanisms<M1_RATE, M2_RATE> {
    fn name(&self) -> String {
        format!("GMT Servo-Mechanisms")
    }
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        log::info!("building GmtServoMechanisms System");
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

        if let Some(telemetry) = &mut self.telemetry {
            self.mount
                .add_output()
                .unbounded()
                .build::<MountTorques>()
                .log(telemetry)?;
            self.m1
                .add_output()
                .unbounded()
                .build::<Flatten<assembly::M1HardpointsForces>>()
                .log(telemetry)?;
            self.m1
                .add_output()
                .unbounded()
                .build::<Flatten<assembly::M1ActuatorAppliedForces>>()
                .log(telemetry)?;
            self.m2_positioners
                .add_output()
                .unbounded()
                .build::<M2PositionerForces>()
                .logn(telemetry, 42)?;
        }

        #[cfg(topend = "ASM")]
        {
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
            if let Some(telemetry) = &mut self.telemetry {
                self.m2
                    .add_output()
                    .unbounded()
                    .build::<Flatten<M2ASMVoiceCoilsForces>>()
                    .log(telemetry)?;
                self.m2
                    .add_output()
                    .unbounded()
                    .build::<Flatten<M2ASMFluidDampingForces>>()
                    .log(telemetry)?;
            }
        }

        #[cfg(topend = "FSM")]
        {
            self.m2
                .add_output()
                .build::<M2FSMPiezoForces>()
                .into_input(&mut self.fem)?;
            self.fem
                .add_output()
                .bootstrap()
                .build::<M2FSMPiezoNodes>()
                .into_input(&mut self.m2)?;
            if let Some(telemetry) = &mut self.telemetry {
                self.m2
                    .add_output()
                    .unbounded()
                    .build::<M2FSMPiezoForces>()
                    .log(telemetry)?;
            }
        }

        Ok(self)
    }
    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(
                PlainActor::from(&self.fem)
                    .filter_inputs_by_name(&[
                        "MountTorques",
                        "M1HardpointsForces",
                        "M1ActuatorAppliedForces",
                        "M2PositionerForces",
                        "M2ASMVoiceCoilsForces",
                        "M2FSMPiezoForces",
                        "M2ASMFluidDampingForces",
                    ])
                    .zip(PlainActor::from(&self.mount).filter_inputs_by_name(&["MountEncoders"]))
                    .zip(
                        PlainActor::from(&self.m2_positioners)
                            .filter_inputs_by_name(&["M2PositionerNodes"]),
                    )
                    .zip(
                        PlainActor::from(&self.m1.dispatch_in)
                            .filter_inputs_by_name(&["M1HardpointsMotion"]),
                    )
                    .zip(
                        PlainActor::from(&self.m2.dispatch_in)
                            .filter_inputs_by_name(&["M2ASMVoiceCoilsMotion", "M2FSMPiezoNodes"]),
                    )
                    .map(|((((mut fem, mount), m2_pos), m1), m2)| {
                        fem.extend(mount);
                        fem.extend(m2_pos);
                        fem.extend(m1);
                        fem.extend(m2);
                        fem
                    })
                    .unwrap(),
            )
            .outputs(
                PlainActor::from(&self.fem)
                    .filter_outputs_by_name(&[
                        "MountEncoders",
                        "M1HardpointsMotion",
                        "M2PositionerNodes",
                        "M2FSMPiezoNodes",
                        "M2ASMVoiceCoilsMotion",
                    ])
                    .unwrap(),
            )
            .image("gmt-servos.png".to_string())
            .graph(self.graph())
            .build()
    }
}
