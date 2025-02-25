use gmt_dos_actors::{
    actor::Actor,
    framework::{
        model::{Check, Task},
        network::{AddActorOutput, AddOuput, TryIntoInputs},
    },
};
use gmt_dos_clients_io::gmt_m2::asm::segment::{
    AsmCommand, FluidDampingForces, VoiceCoilsForces, VoiceCoilsMotion,
};
use gmt_dos_clients_m2_ctrl::AsmSegmentInnerController;
use serde::{Deserialize, Serialize};

use super::{DispatchIn, DispatchOut};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AsmsInnerControllers<const R: usize> {
    S1(Actor<AsmSegmentInnerController<1>, R, R>),
    S2(Actor<AsmSegmentInnerController<2>, R, R>),
    S3(Actor<AsmSegmentInnerController<3>, R, R>),
    S4(Actor<AsmSegmentInnerController<4>, R, R>),
    S5(Actor<AsmSegmentInnerController<5>, R, R>),
    S6(Actor<AsmSegmentInnerController<6>, R, R>),
    S7(Actor<AsmSegmentInnerController<7>, R, R>),
}
impl<const R: usize> AsmsInnerControllers<R> {
    pub fn new(id: u8, n_mode: usize, ks: Option<Vec<f64>>) -> Self {
        match id {
            1 => Self::S1((AsmSegmentInnerController::<1>::new(n_mode, ks), "ASM #1").into()),
            2 => Self::S2((AsmSegmentInnerController::<2>::new(n_mode, ks), "ASM #2").into()),
            3 => Self::S3((AsmSegmentInnerController::<3>::new(n_mode, ks), "ASM #3").into()),
            4 => Self::S4((AsmSegmentInnerController::<4>::new(n_mode, ks), "ASM #4").into()),
            5 => Self::S5((AsmSegmentInnerController::<5>::new(n_mode, ks), "ASM #5").into()),
            6 => Self::S6((AsmSegmentInnerController::<6>::new(n_mode, ks), "ASM #6").into()),
            7 => Self::S7((AsmSegmentInnerController::<7>::new(n_mode, ks), "ASM #7").into()),
            _ => todo!(),
        }
    }
    pub fn as_check(&self) -> Box<&dyn Check> {
        match self {
            Self::S1(actor) => Box::new(actor as &dyn Check),
            Self::S2(actor) => Box::new(actor as &dyn Check),
            Self::S3(actor) => Box::new(actor as &dyn Check),
            Self::S4(actor) => Box::new(actor as &dyn Check),
            Self::S5(actor) => Box::new(actor as &dyn Check),
            Self::S6(actor) => Box::new(actor as &dyn Check),
            Self::S7(actor) => Box::new(actor as &dyn Check),
        }
    }
    pub fn into_task(self) -> Box<dyn Task> {
        match self {
            Self::S1(actor) => Box::new(actor) as Box<dyn Task>,
            Self::S2(actor) => Box::new(actor) as Box<dyn Task>,
            Self::S3(actor) => Box::new(actor) as Box<dyn Task>,
            Self::S4(actor) => Box::new(actor) as Box<dyn Task>,
            Self::S5(actor) => Box::new(actor) as Box<dyn Task>,
            Self::S6(actor) => Box::new(actor) as Box<dyn Task>,
            Self::S7(actor) => Box::new(actor) as Box<dyn Task>,
        }
    }
    pub fn asm_command(&mut self, dispatch: &mut Actor<DispatchIn, R, R>) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => dispatch
                .add_output()
                .build::<AsmCommand<1>>()
                .into_input(actor)?,
            Self::S2(actor) => dispatch
                .add_output()
                .build::<AsmCommand<2>>()
                .into_input(actor)?,
            Self::S3(actor) => dispatch
                .add_output()
                .build::<AsmCommand<3>>()
                .into_input(actor)?,
            Self::S4(actor) => dispatch
                .add_output()
                .build::<AsmCommand<4>>()
                .into_input(actor)?,
            Self::S5(actor) => dispatch
                .add_output()
                .build::<AsmCommand<5>>()
                .into_input(actor)?,
            Self::S6(actor) => dispatch
                .add_output()
                .build::<AsmCommand<6>>()
                .into_input(actor)?,
            Self::S7(actor) => dispatch
                .add_output()
                .build::<AsmCommand<7>>()
                .into_input(actor)?,
        };
        Ok(())
    }
    pub fn asm_voice_coils_motion(
        &mut self,
        dispatch: &mut Actor<DispatchIn, R, R>,
    ) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => dispatch
                .add_output()
                .build::<VoiceCoilsMotion<1>>()
                .into_input(actor)?,
            Self::S2(actor) => dispatch
                .add_output()
                .build::<VoiceCoilsMotion<2>>()
                .into_input(actor)?,
            Self::S3(actor) => dispatch
                .add_output()
                .build::<VoiceCoilsMotion<3>>()
                .into_input(actor)?,
            Self::S4(actor) => dispatch
                .add_output()
                .build::<VoiceCoilsMotion<4>>()
                .into_input(actor)?,
            Self::S5(actor) => dispatch
                .add_output()
                .build::<VoiceCoilsMotion<5>>()
                .into_input(actor)?,
            Self::S6(actor) => dispatch
                .add_output()
                .build::<VoiceCoilsMotion<6>>()
                .into_input(actor)?,
            Self::S7(actor) => dispatch
                .add_output()
                .build::<VoiceCoilsMotion<7>>()
                .into_input(actor)?,
        };
        Ok(())
    }
    pub fn asm_voice_coils_forces(
        &mut self,
        dispatch: &mut Actor<DispatchOut, R, R>,
    ) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => actor
                .add_output()
                .build::<VoiceCoilsForces<1>>()
                .into_input(dispatch)?,
            Self::S2(actor) => actor
                .add_output()
                .build::<VoiceCoilsForces<2>>()
                .into_input(dispatch)?,
            Self::S3(actor) => actor
                .add_output()
                .build::<VoiceCoilsForces<3>>()
                .into_input(dispatch)?,
            Self::S4(actor) => actor
                .add_output()
                .build::<VoiceCoilsForces<4>>()
                .into_input(dispatch)?,
            Self::S5(actor) => actor
                .add_output()
                .build::<VoiceCoilsForces<5>>()
                .into_input(dispatch)?,
            Self::S6(actor) => actor
                .add_output()
                .build::<VoiceCoilsForces<6>>()
                .into_input(dispatch)?,
            Self::S7(actor) => actor
                .add_output()
                .build::<VoiceCoilsForces<7>>()
                .into_input(dispatch)?,
        };
        Ok(())
    }
    pub fn asm_fluid_damping_forces(
        &mut self,
        dispatch: &mut Actor<DispatchOut, R, R>,
    ) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => actor
                .add_output()
                .build::<FluidDampingForces<1>>()
                .into_input(dispatch)?,
            Self::S2(actor) => actor
                .add_output()
                .build::<FluidDampingForces<2>>()
                .into_input(dispatch)?,
            Self::S3(actor) => actor
                .add_output()
                .build::<FluidDampingForces<3>>()
                .into_input(dispatch)?,
            Self::S4(actor) => actor
                .add_output()
                .build::<FluidDampingForces<4>>()
                .into_input(dispatch)?,
            Self::S5(actor) => actor
                .add_output()
                .build::<FluidDampingForces<5>>()
                .into_input(dispatch)?,
            Self::S6(actor) => actor
                .add_output()
                .build::<FluidDampingForces<6>>()
                .into_input(dispatch)?,
            Self::S7(actor) => actor
                .add_output()
                .build::<FluidDampingForces<7>>()
                .into_input(dispatch)?,
        };
        Ok(())
    }
}
