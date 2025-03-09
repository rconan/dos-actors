use std::fmt::Display;

use crate::subsystems::SegmentControl;
use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::{
        model::{Check, Task},
        network::{AddActorOutput, AddOuput, TryIntoInputs},
    },
    system::{System, SystemError, SystemInput, SystemOutput},
};
use gmt_dos_clients::sampler::Sampler;
use gmt_dos_clients_io::gmt_m1::segment::{
    ActuatorCommandForces, BarycentricForce, HardpointsForces,
};
use gmt_dos_clients_m1_ctrl::{Actuators, Hardpoints, LoadCells};

impl<const S: u8, const R: usize> System for SegmentControl<S, R> {
    fn name(&self) -> String {
        format!("M1S{S}")
    }

    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.sampler
            .add_output()
            .build::<ActuatorCommandForces<S>>()
            .into_input(&mut self.actuators)?;

        self.hardpoints
            .add_output()
            .build::<HardpointsForces<S>>()
            .into_input(&mut self.loadcells)?;

        self.loadcells
            .add_output()
            .build::<BarycentricForce<S>>()
            .into_input(&mut self.actuators)?;
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut plain = PlainActor::default();
        plain.client = self.name();
        plain.inputs_rate = 1;
        plain.outputs_rate = 1;
        let iter = PlainActor::from(&self.hardpoints)
            .inputs
            .unwrap()
            .into_iter()
            .chain(PlainActor::from(&self.sampler).inputs.unwrap().into_iter())
            .chain(
                PlainActor::from(&self.loadcells)
                    .inputs
                    .unwrap()
                    .into_iter(),
            );
        plain.inputs = Some(iter.collect());
        let iter = PlainActor::from(&self.hardpoints)
            .outputs
            .unwrap()
            .into_iter()
            .chain(
                PlainActor::from(&self.actuators)
                    .outputs
                    .unwrap()
                    .into_iter(),
            );
        plain.outputs = Some(iter.collect());
        plain
    }
}

impl<'a, const S: u8, const R: usize> IntoIterator for &'a SegmentControl<S, R> {
    type Item = Box<&'a dyn Check>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.sampler as &dyn Check),
            Box::new(&self.hardpoints as &dyn Check),
            Box::new(&self.actuators as &dyn Check),
            Box::new(&self.loadcells as &dyn Check),
        ]
        .into_iter()
    }
}

impl<const S: u8, const R: usize> IntoIterator for Box<SegmentControl<S, R>> {
    type Item = Box<dyn Task>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(self.sampler) as Box<dyn Task>,
            Box::new(self.hardpoints) as Box<dyn Task>,
            Box::new(self.actuators) as Box<dyn Task>,
            Box::new(self.loadcells) as Box<dyn Task>,
        ]
        .into_iter()
    }
}

impl<const S: u8, const R: usize> SystemInput<Hardpoints, 1, 1> for SegmentControl<S, R> {
    fn input(&mut self) -> &mut Actor<Hardpoints, 1, 1> {
        &mut self.hardpoints
    }
}

impl<const S: u8, const R: usize> SystemInput<Sampler<Vec<f64>, ActuatorCommandForces<S>>, 1, R>
    for SegmentControl<S, R>
{
    fn input(&mut self) -> &mut Actor<Sampler<Vec<f64>, ActuatorCommandForces<S>>, 1, R> {
        &mut self.sampler
    }
}

impl<const S: u8, const R: usize> SystemInput<LoadCells, 1, R> for SegmentControl<S, R> {
    fn input(&mut self) -> &mut Actor<LoadCells, 1, R> {
        &mut self.loadcells
    }
}

impl<const S: u8, const R: usize> SystemOutput<Hardpoints, 1, 1> for SegmentControl<S, R> {
    fn output(&mut self) -> &mut Actor<Hardpoints, 1, 1> {
        &mut self.hardpoints
    }
}

impl<const S: u8, const R: usize> SystemOutput<Actuators<S>, R, 1> for SegmentControl<S, R> {
    fn output(&mut self) -> &mut Actor<Actuators<S>, R, 1> {
        &mut self.actuators
    }
}

impl<const S: u8, const R: usize> Display for SegmentControl<S, R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}
