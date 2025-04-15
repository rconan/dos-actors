pub mod sh24;
pub mod sh48;

use std::fmt::Display;

use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::model::{Check, FlowChart, Task},
    prelude::{AddActorOutput, AddOuput, TryIntoInputs},
    system::{System, SystemError, SystemInput, SystemOutput},
};
use sh24::Sh24;
use sh48::Sh48;

use crate::{
    kernels::{Kernel, KernelFrame},
    AgwsBuilder,
};

/// GMT AGWS model
#[derive(Clone)]
pub struct Agws<const SH48_I: usize = 1, const SH24_I: usize = 1> {
    pub(crate) sh48: Actor<Sh48<SH48_I>, 1, SH48_I>,
    pub(crate) sh24: Actor<Sh24<SH24_I>, 1, SH24_I>,
    pub(crate) sh24_kernel: Actor<Kernel<Sh24<SH24_I>>, SH24_I, SH24_I>,
    pub(crate) sh48_kernel: Actor<Kernel<Sh48<SH48_I>>, SH48_I, SH48_I>,
}

impl<const SH48_I: usize, const SH24_I: usize> Display for Agws<SH48_I, SH24_I> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.sh48.fmt(f)?;
        self.sh24.fmt(f)?;
        Ok(())
    }
}

impl<const SH48_I: usize, const SH24_I: usize> Agws<SH48_I, SH24_I> {
    pub fn builder() -> AgwsBuilder<SH48_I, SH24_I> {
        Default::default()
    }
    /// Sensor pointing error relative to the source
    ///
    /// The pointing error is given in cartesian coordinates and in radians units
    pub async fn sh24_pointing(
        &mut self,
        xy: (f64, f64),
    ) -> Result<(), Box<dyn std::error::Error>> {
        (self.sh24.client().lock().await).sensor_pointing(&[xy])?;
        Ok(())
    }
}

impl<const SH48_I: usize, const SH24_I: usize> System for Agws<SH48_I, SH24_I> {
    fn name(&self) -> String {
        String::from("AGWS")
    }
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.sh24
            .add_output()
            .bootstrap()
            .build::<KernelFrame<Sh24<SH24_I>>>()
            .into_input(&mut self.sh24_kernel)?;
        self.sh48
            .add_output()
            .bootstrap()
            .build::<KernelFrame<Sh48<SH48_I>>>()
            .into_input(&mut self.sh48_kernel)?;
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(self.sh48.as_plain().inputs().unwrap())
            .outputs(
                self.sh24_kernel
                    .as_plain()
                    .outputs()
                    .unwrap()
                    .into_iter()
                    .chain(self.sh48_kernel.as_plain().outputs().unwrap().into_iter())
                    .collect(),
            )
            .graph(self.graph())
            .build()
    }
}

impl<'a, const SH48_I: usize, const SH24_I: usize> IntoIterator for &'a Agws<SH48_I, SH24_I> {
    type Item = Box<&'a dyn Check>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.sh48 as &dyn Check),
            Box::new(&self.sh24 as &dyn Check),
            Box::new(&self.sh24_kernel as &dyn Check),
            Box::new(&self.sh48_kernel as &dyn Check),
        ]
        .into_iter()
    }
}

impl<const SH48_I: usize, const SH24_I: usize> IntoIterator for Box<Agws<SH48_I, SH24_I>> {
    type Item = Box<dyn Task>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(self.sh48) as Box<dyn Task>,
            Box::new(self.sh24) as Box<dyn Task>,
            Box::new(self.sh24_kernel) as Box<dyn Task>,
            Box::new(self.sh48_kernel) as Box<dyn Task>,
        ]
        .into_iter()
    }
}
impl<const SH48_I: usize, const SH24_I: usize> SystemInput<Sh48<SH48_I>, 1, SH48_I>
    for Agws<SH48_I, SH24_I>
{
    fn input(&mut self) -> &mut Actor<Sh48<SH48_I>, 1, SH48_I> {
        &mut self.sh48
    }
}
impl<const SH48_I: usize, const SH24_I: usize> SystemOutput<Sh48<SH48_I>, 1, SH48_I>
    for Agws<SH48_I, SH24_I>
{
    fn output(&mut self) -> &mut Actor<Sh48<SH48_I>, 1, SH48_I> {
        &mut self.sh48
    }
}
impl<const SH48_I: usize, const SH24_I: usize> SystemOutput<Kernel<Sh48<SH48_I>>, SH48_I, SH48_I>
    for Agws<SH48_I, SH24_I>
{
    fn output(&mut self) -> &mut Actor<Kernel<Sh48<SH48_I>>, SH48_I, SH48_I> {
        &mut self.sh48_kernel
    }
}

impl<const SH48_I: usize, const SH24_I: usize> SystemInput<Sh24<SH24_I>, 1, SH24_I>
    for Agws<SH48_I, SH24_I>
{
    fn input(&mut self) -> &mut Actor<Sh24<SH24_I>, 1, SH24_I> {
        &mut self.sh24
    }
}

impl<const SH48_I: usize, const SH24_I: usize> SystemOutput<Sh24<SH24_I>, 1, SH24_I>
    for Agws<SH48_I, SH24_I>
{
    fn output(&mut self) -> &mut Actor<Sh24<SH24_I>, 1, SH24_I> {
        &mut self.sh24
    }
}
impl<const SH48_I: usize, const SH24_I: usize> SystemOutput<Kernel<Sh24<SH24_I>>, SH24_I, SH24_I>
    for Agws<SH48_I, SH24_I>
{
    fn output(&mut self) -> &mut Actor<Kernel<Sh24<SH24_I>>, SH24_I, SH24_I> {
        &mut self.sh24_kernel
    }
}
