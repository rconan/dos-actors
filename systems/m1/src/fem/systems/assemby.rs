use std::fmt::Display;
use std::sync::Arc;

use gmt_dos_actors::framework::network::{ActorOutput, AddActorInput};
use gmt_dos_actors::subsystem::{gateway, GetField};
use gmt_dos_actors::system::System;

use gmt_dos_actors::{framework::model::Check, prelude::*};
use gmt_dos_clients_io::gmt_m1::assembly::{
    M1ActuatorAppliedForces, M1ActuatorCommandForces, M1HardpointsMotion, M1RigidBodyMotions,
};
use gmt_dos_clients_io::gmt_m1::segment::{
    ActuatorAppliedForces, ActuatorCommandForces, HardpointsForces, HardpointsMotion, RBM,
};
use gmt_dos_clients_io::Assembly;
use interface::UniqueIdentifier;

use crate::{
    subsystems::{Segment, SegmentControl},
    Actuators, Calibration, Hardpoints,
};

use super::dispatch::{DispatchIn, DispatchOut};

#[derive(Clone)]
pub enum SegmentControls<const R: usize> {
    S1(SegmentControl<1, R>),
    S2(SegmentControl<2, R>),
    S3(SegmentControl<3, R>),
    S4(SegmentControl<4, R>),
    S5(SegmentControl<5, R>),
    S6(SegmentControl<6, R>),
    S7(SegmentControl<7, R>),
}

impl<const R: usize> SegmentControls<R> {
    pub fn new(id: u8, calibration: &Calibration) -> anyhow::Result<Self> {
        Ok(match id {
            1 => Self::S1(SegmentControl::<1, R>::new(calibration)),
            2 => Self::S2(SegmentControl::<2, R>::new(calibration)),
            3 => Self::S3(SegmentControl::<3, R>::new(calibration)),
            4 => Self::S4(SegmentControl::<4, R>::new(calibration)),
            5 => Self::S5(SegmentControl::<5, R>::new(calibration)),
            6 => Self::S6(SegmentControl::<6, R>::new(calibration)),
            7 => Self::S7(SegmentControl::<7, R>::new(calibration)),
            _ => todo!(),
        })
    }
    pub fn get(&self) -> Option<&dyn Check> {
        match self {
            Self::S1(actor) => Some(actor as &dyn Check),
            Self::S2(actor) => Some(actor as &dyn Check),
            Self::S3(actor) => Some(actor as &dyn Check),
            Self::S4(actor) => Some(actor as &dyn Check),
            Self::S5(actor) => Some(actor as &dyn Check),
            Self::S6(actor) => Some(actor as &dyn Check),
            Self::S7(actor) => Some(actor as &dyn Check),
        }
    }
    pub fn into_model(self) -> Model<Unknown> {
        match self {
            Self::S1(actor) => model!(actor),
            Self::S2(actor) => model!(actor),
            Self::S3(actor) => model!(actor),
            Self::S4(actor) => model!(actor),
            Self::S5(actor) => model!(actor),
            Self::S6(actor) => model!(actor),
            Self::S7(actor) => model!(actor),
        }
    }
    pub fn m1_rigid_body_motions(
        &mut self,
        dispatch: &mut Actor<DispatchIn>,
    ) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => dispatch.add_output::<RBM<1>>().build().into_input(actor)?,
            Self::S2(actor) => dispatch.add_output::<RBM<2>>().build().into_input(actor)?,
            Self::S3(actor) => dispatch.add_output::<RBM<3>>().build().into_input(actor)?,
            Self::S4(actor) => dispatch.add_output::<RBM<4>>().build().into_input(actor)?,
            Self::S5(actor) => dispatch.add_output::<RBM<5>>().build().into_input(actor)?,
            Self::S6(actor) => dispatch.add_output::<RBM<6>>().build().into_input(actor)?,
            Self::S7(actor) => dispatch.add_output::<RBM<7>>().build().into_input(actor)?,
        };
        Ok(())
    }
    pub fn m1_actuator_command_forces(
        &mut self,
        dispatch: &mut Actor<DispatchIn>,
    ) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => dispatch
                .add_output::<ActuatorCommandForces<1>>()
                .build()
                .into_input(actor)?,
            Self::S2(actor) => dispatch
                .add_output::<ActuatorCommandForces<2>>()
                .build()
                .into_input(actor)?,
            Self::S3(actor) => dispatch
                .add_output::<ActuatorCommandForces<3>>()
                .build()
                .into_input(actor)?,
            Self::S4(actor) => dispatch
                .add_output::<ActuatorCommandForces<4>>()
                .build()
                .into_input(actor)?,
            Self::S5(actor) => dispatch
                .add_output::<ActuatorCommandForces<5>>()
                .build()
                .into_input(actor)?,
            Self::S6(actor) => dispatch
                .add_output::<ActuatorCommandForces<6>>()
                .build()
                .into_input(actor)?,
            Self::S7(actor) => dispatch
                .add_output::<ActuatorCommandForces<7>>()
                .build()
                .into_input(actor)?,
        };
        Ok(())
    }
    pub fn m1_hardpoints_motion(&mut self, dispatch: &mut Actor<DispatchIn>) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => dispatch
                .add_output::<HardpointsMotion<1>>()
                .build()
                .into_input(actor)?,
            Self::S2(actor) => dispatch
                .add_output::<HardpointsMotion<2>>()
                .build()
                .into_input(actor)?,
            Self::S3(actor) => dispatch
                .add_output::<HardpointsMotion<3>>()
                .build()
                .into_input(actor)?,
            Self::S4(actor) => dispatch
                .add_output::<HardpointsMotion<4>>()
                .build()
                .into_input(actor)?,
            Self::S5(actor) => dispatch
                .add_output::<HardpointsMotion<5>>()
                .build()
                .into_input(actor)?,
            Self::S6(actor) => dispatch
                .add_output::<HardpointsMotion<6>>()
                .build()
                .into_input(actor)?,
            Self::S7(actor) => dispatch
                .add_output::<HardpointsMotion<7>>()
                .build()
                .into_input(actor)?,
        };
        Ok(())
    }
    pub fn m1_actuator_applied_forces(
        &mut self,
        dispatch: &mut Actor<DispatchOut>,
    ) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => {
                <SegmentControl<1, R> as AddActorOutput<'_, Actuators<1>, R, 1>>::add_output::<
                    ActuatorAppliedForces<1>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S2(actor) => {
                <SegmentControl<2, R> as AddActorOutput<'_, Actuators<2>, R, 1>>::add_output::<
                    ActuatorAppliedForces<2>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S3(actor) => {
                <SegmentControl<3, R> as AddActorOutput<'_, Actuators<3>, R, 1>>::add_output::<
                    ActuatorAppliedForces<3>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S4(actor) => {
                <SegmentControl<4, R> as AddActorOutput<'_, Actuators<4>, R, 1>>::add_output::<
                    ActuatorAppliedForces<4>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S5(actor) => {
                <SegmentControl<5, R> as AddActorOutput<'_, Actuators<5>, R, 1>>::add_output::<
                    ActuatorAppliedForces<5>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S6(actor) => {
                <SegmentControl<6, R> as AddActorOutput<'_, Actuators<6>, R, 1>>::add_output::<
                    ActuatorAppliedForces<6>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S7(actor) => {
                <SegmentControl<7, R> as AddActorOutput<'_, Actuators<7>, R, 1>>::add_output::<
                    ActuatorAppliedForces<7>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
        };
        Ok(())
    }
    pub fn m1_hardpoints_forces(
        &mut self,
        dispatch: &mut Actor<DispatchOut>,
    ) -> anyhow::Result<()> {
        match self {
            Self::S1(actor) => {
                <SegmentControl<1, R> as AddActorOutput<'_, Hardpoints, 1, 1>>::add_output::<
                    HardpointsForces<1>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S2(actor) => {
                <SegmentControl<2, R> as AddActorOutput<'_, Hardpoints, 1, 1>>::add_output::<
                    HardpointsForces<2>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S3(actor) => {
                <SegmentControl<3, R> as AddActorOutput<'_, Hardpoints, 1, 1>>::add_output::<
                    HardpointsForces<3>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S4(actor) => {
                <SegmentControl<4, R> as AddActorOutput<'_, Hardpoints, 1, 1>>::add_output::<
                    HardpointsForces<4>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S5(actor) => {
                <SegmentControl<5, R> as AddActorOutput<'_, Hardpoints, 1, 1>>::add_output::<
                    HardpointsForces<5>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S6(actor) => {
                <SegmentControl<6, R> as AddActorOutput<'_, Hardpoints, 1, 1>>::add_output::<
                    HardpointsForces<6>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
            Self::S7(actor) => {
                <SegmentControl<7, R> as AddActorOutput<'_, Hardpoints, 1, 1>>::add_output::<
                    HardpointsForces<7>,
                >(actor)
                .build()
                .into_input(dispatch)?
            }
        };
        Ok(())
    }
}

impl<const R: usize> Assembly for M1<R> {}

#[derive(Clone)]
pub struct M1<const R: usize>
where
    Self: Assembly,
{
    segments: Vec<SegmentControls<R>>,
    dispatch_in: Actor<DispatchIn>,
    dispatch_out: Actor<DispatchOut>,
}

impl<const R: usize> System for M1<R> {
    fn output<C: interface::Update, const NI: usize, const NO: usize>(
        &mut self,
    ) -> &mut Actor<C, NI, NO> {
        todo!()
    }

    fn input<CI: interface::Update, const NI: usize, const NO: usize>(
        &mut self,
    ) -> &mut Actor<CI, NI, NO> {
        todo!()
    }

    fn name(&self) -> Option<String> {
        todo!()
    }

    fn build(&mut self) -> anyhow::Result<()> {
        /*         gateway_in
            .add_output()
            .build::<M1RigidBodyMotions>()
            .into_input(&mut self.dispatch_in)?;
        gateway_in
            .add_output()
            .build::<M1ActuatorCommandForces>()
            .into_input(&mut self.dispatch_in)?;
        gateway_in
            .add_output()
            .build::<M1HardpointsMotion>()
            .into_input(&mut self.dispatch_in)?;*/

        self.segments
            .iter_mut()
            .map(|segment| segment.m1_rigid_body_motions(&mut self.dispatch_in))
            .collect::<anyhow::Result<Vec<()>>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_actuator_command_forces(&mut self.dispatch_in))
            .collect::<anyhow::Result<Vec<()>>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_hardpoints_motion(&mut self.dispatch_in))
            .collect::<anyhow::Result<Vec<()>>>()?;

        self.segments
            .iter_mut()
            .map(|segment| segment.m1_actuator_applied_forces(&mut self.dispatch_out))
            .collect::<anyhow::Result<Vec<()>>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_hardpoints_forces(&mut self.dispatch_out))
            .collect::<anyhow::Result<Vec<()>>>()?;

        /*         self.dispatch_out
            .add_output()
            .build::<M1ActuatorAppliedForces>()
            .into_input(gateway_out)?;
        self.dispatch_out
            .add_output()
            .build::<M1HardpointsForces>()
            .into_input(gateway_out)?; */
        Ok(())
    }
}

impl<const R: usize> GetField for M1<R> {
    fn get_field(&self, idx: usize) -> Option<&dyn Check> {
        match idx {
            n if n < <M1<R> as Assembly>::N => {
                self.segments.get(idx).and_then(|segment| segment.get())
            }
            n if n == <M1<R> as Assembly>::N => Some(&self.dispatch_in as &dyn Check),
            n if n == <M1<R> as Assembly>::N + 1 => Some(&self.dispatch_out as &dyn Check),
            _ => None,
        }
    }
}

impl<const R: usize> From<M1<R>> for Model<Unknown> {
    fn from(m1: M1<R>) -> Self {
        m1.segments
            .into_iter()
            .fold(Model::default(), |model, segment| {
                model + segment.into_model()
            })
            + m1.dispatch_in
            + m1.dispatch_out
    }
}

impl<const R: usize> gateway::Gateways for M1<R> {
    type DataType = Vec<Arc<Vec<f64>>>;

    const N_IN: usize = 3;

    const N_OUT: usize = 2;
}

impl<const R: usize> Display for M1<R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SegmentControls")
    }
}

impl<const R: usize> GetName for M1<R> {
    fn get_name(&self) -> String {
        "integrated_model".into()
    }
}

impl<const R: usize> AddActorInput<M1RigidBodyMotions, DispatchIn, 1, 1> for M1<R> {
    fn add_input(&mut self, rx: flume::Receiver<interface::Data<M1RigidBodyMotions>>, hash: u64) {
        AddActorInput::add_input(&mut self.dispatch_in, rx, hash)
    }
}

impl<const R: usize> AddActorInput<M1ActuatorCommandForces, DispatchIn, 1, 1> for M1<R> {
    fn add_input(
        &mut self,
        rx: flume::Receiver<interface::Data<M1ActuatorCommandForces>>,
        hash: u64,
    ) {
        AddActorInput::add_input(&mut self.dispatch_in, rx, hash)
    }
}

impl<const R: usize> AddActorInput<M1HardpointsMotion, DispatchIn, 1, 1> for M1<R> {
    fn add_input(&mut self, rx: flume::Receiver<interface::Data<M1HardpointsMotion>>, hash: u64) {
        AddActorInput::add_input(&mut self.dispatch_in, rx, hash)
    }
}

impl<'a, const R: usize> AddActorOutput<'a, DispatchOut, 1, 1> for M1<R> {
    fn add_output<M1HardpointsForces: UniqueIdentifier>(
        &'a mut self,
    ) -> ActorOutput<'a, Actor<DispatchOut, 1, 1>> {
        // AddActorOutput::add_output::<M1HardpointsForces>(&mut self.dispatch_out)
        todo!()
    }
}
