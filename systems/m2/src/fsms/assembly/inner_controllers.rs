use gmt_dos_actors::{
    actor::Actor,
    framework::{
        model::{Check, Task},
        network::ActorOutputsError,
    },
    prelude::{AddActorOutput, AddOuput, TryIntoInputs},
};
use gmt_dos_clients_io::gmt_m2::fsm::segment::{FsmCommand, PiezoForces, PiezoNodes};
use gmt_dos_clients_m2_ctrl::FsmSegmentInnerController;
use serde::{Deserialize, Serialize};

use super::dispatch::{DispatchIn, DispatchOut};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FsmsInnerControllers<const R: usize> {
    S1(Actor<FsmSegmentInnerController<1>, R, R>),
    S2(Actor<FsmSegmentInnerController<2>, R, R>),
    S3(Actor<FsmSegmentInnerController<3>, R, R>),
    S4(Actor<FsmSegmentInnerController<4>, R, R>),
    S5(Actor<FsmSegmentInnerController<5>, R, R>),
    S6(Actor<FsmSegmentInnerController<6>, R, R>),
    S7(Actor<FsmSegmentInnerController<7>, R, R>),
}
impl<const R: usize> FsmsInnerControllers<R> {
    pub fn new(id: u8) -> Self {
        match id {
            1 => Self::S1((FsmSegmentInnerController::<1>::new(), "FSM #1").into()),
            2 => Self::S2((FsmSegmentInnerController::<2>::new(), "FSM #2").into()),
            3 => Self::S3((FsmSegmentInnerController::<3>::new(), "FSM #3").into()),
            4 => Self::S4((FsmSegmentInnerController::<4>::new(), "FSM #4").into()),
            5 => Self::S5((FsmSegmentInnerController::<5>::new(), "FSM #5").into()),
            6 => Self::S6((FsmSegmentInnerController::<6>::new(), "FSM #6").into()),
            7 => Self::S7((FsmSegmentInnerController::<7>::new(), "FSM #7").into()),
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
    pub fn fsm_command(
        &mut self,
        dispatch: &mut Actor<DispatchIn, R, R>,
    ) -> Result<(), ActorOutputsError> {
        match self {
            Self::S1(actor) => dispatch
                .add_output()
                .build::<FsmCommand<1>>()
                .into_input(actor)?,
            Self::S2(actor) => dispatch
                .add_output()
                .build::<FsmCommand<2>>()
                .into_input(actor)?,
            Self::S3(actor) => dispatch
                .add_output()
                .build::<FsmCommand<3>>()
                .into_input(actor)?,
            Self::S4(actor) => dispatch
                .add_output()
                .build::<FsmCommand<4>>()
                .into_input(actor)?,
            Self::S5(actor) => dispatch
                .add_output()
                .build::<FsmCommand<5>>()
                .into_input(actor)?,
            Self::S6(actor) => dispatch
                .add_output()
                .build::<FsmCommand<6>>()
                .into_input(actor)?,
            Self::S7(actor) => dispatch
                .add_output()
                .build::<FsmCommand<7>>()
                .into_input(actor)?,
        };
        Ok(())
    }
    pub fn fsm_pzt_motion(
        &mut self,
        dispatch: &mut Actor<DispatchIn, R, R>,
    ) -> Result<(), ActorOutputsError> {
        match self {
            Self::S1(actor) => dispatch
                .add_output()
                .build::<PiezoNodes<1>>()
                .into_input(actor)?,
            Self::S2(actor) => dispatch
                .add_output()
                .build::<PiezoNodes<2>>()
                .into_input(actor)?,
            Self::S3(actor) => dispatch
                .add_output()
                .build::<PiezoNodes<3>>()
                .into_input(actor)?,
            Self::S4(actor) => dispatch
                .add_output()
                .build::<PiezoNodes<4>>()
                .into_input(actor)?,
            Self::S5(actor) => dispatch
                .add_output()
                .build::<PiezoNodes<5>>()
                .into_input(actor)?,
            Self::S6(actor) => dispatch
                .add_output()
                .build::<PiezoNodes<6>>()
                .into_input(actor)?,
            Self::S7(actor) => dispatch
                .add_output()
                .build::<PiezoNodes<7>>()
                .into_input(actor)?,
        };
        Ok(())
    }
    pub fn fsm_pzt_forces(
        &mut self,
        dispatch: &mut Actor<DispatchOut, R, R>,
    ) -> Result<(), ActorOutputsError> {
        match self {
            Self::S1(actor) => actor
                .add_output()
                .build::<PiezoForces<1>>()
                .into_input(dispatch)?,
            Self::S2(actor) => actor
                .add_output()
                .build::<PiezoForces<2>>()
                .into_input(dispatch)?,
            Self::S3(actor) => actor
                .add_output()
                .build::<PiezoForces<3>>()
                .into_input(dispatch)?,
            Self::S4(actor) => actor
                .add_output()
                .build::<PiezoForces<4>>()
                .into_input(dispatch)?,
            Self::S5(actor) => actor
                .add_output()
                .build::<PiezoForces<5>>()
                .into_input(dispatch)?,
            Self::S6(actor) => actor
                .add_output()
                .build::<PiezoForces<6>>()
                .into_input(dispatch)?,
            Self::S7(actor) => actor
                .add_output()
                .build::<PiezoForces<7>>()
                .into_input(dispatch)?,
        };
        Ok(())
    }
}
