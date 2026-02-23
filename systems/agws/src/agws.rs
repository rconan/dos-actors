pub mod sh24;
pub mod sh48;

use std::{fmt::Display, marker::PhantomData};

use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::model::{Check, FlowChart, Task},
    prelude::{AddActorOutput, AddOuput, TryIntoInputs},
    system::{System, SystemError, SystemInput, SystemOutput},
};
use gmt_dos_clients_crseo::sensors::Camera;
use gmt_dos_clients_io::optics::{Dev, Frame};
use interface::{TryRead, TryUpdate};
use sh24::Sh24;
use sh48::Sh48;

use crate::{
    AgwsBuilder,
    agws::{sh24::kernel::Sh24Kern, sh48::kernel::Sh48Kern},
    kernels::{KernelFrame, KernelSpecs},
};

/// GMT AGWS model
pub struct Agws<
    const SH48_I: usize = 1,
    const SH24_I: usize = 1,
    K48 = Sh48<SH48_I>,
    K24 = Sh24<SH24_I>,
> where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    #[cfg(feature = "sh48")]
    pub(crate) sh48: Actor<Sh48<SH48_I>, 1, SH48_I>,
    #[cfg(feature = "sh24")]
    pub(crate) sh24: Actor<Sh24<SH24_I>, 1, SH24_I>,
    #[cfg(feature = "shk24")]
    pub(crate) sh24_kernel: Actor<Sh24Kern<K24>, SH24_I, SH24_I>,
    #[cfg(feature = "shk48")]
    pub(crate) sh48_kernel: Actor<Sh48Kern<K48>, SH48_I, SH48_I>,
    pub(crate) k48: PhantomData<K48>,
    pub(crate) k24: PhantomData<K24>,
}

impl<const SH48_I: usize, const SH24_I: usize, K48, K24> Clone for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn clone(&self) -> Self {
        Self {
            #[cfg(feature = "sh48")]
            sh48: self.sh48.clone(),
            #[cfg(feature = "sh24")]
            sh24: self.sh24.clone(),
            #[cfg(feature = "shk24")]
            sh24_kernel: self.sh24_kernel.clone(),
            #[cfg(feature = "shk48")]
            sh48_kernel: self.sh48_kernel.clone(),
            k48: PhantomData,
            k24: PhantomData,
        }
    }
}

impl<const SH48_I: usize, const SH24_I: usize, K48, K24> Display for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        #[cfg(feature = "sh48")]
        self.sh48.fmt(f)?;
        #[cfg(feature = "sh24")]
        self.sh24.fmt(f)?;
        Ok(())
    }
}

impl<const SH48_I: usize, const SH24_I: usize, K48, K24> Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    pub fn builder() -> AgwsBuilder<SH48_I, SH24_I, K48, K24> {
        Default::default()
    }
    /// Sensor pointing error relative to the source
    ///
    /// The pointing error is given in cartesian coordinates and in radians units
    #[cfg(feature = "sh24")]
    pub async fn sh24_pointing(
        &mut self,
        xy: (f64, f64),
    ) -> Result<(), Box<dyn std::error::Error>> {
        (self.sh24.client().lock().await).sensor_pointing(&[xy])?;
        Ok(())
    }
}

impl<const SH48_I: usize, const SH24_I: usize, K48, K24> System for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs<Input = Frame<Dev>, Sensor = Camera<SH48_I>> + 'static + Send + Sync,
    K24: KernelSpecs<Input = Frame<Dev>, Sensor = Camera<SH24_I>> + 'static + Send + Sync,
    Sh24Kern<K24>: TryRead<KernelFrame<K24>>,
    Sh48Kern<K48>: TryRead<KernelFrame<K48>>,
{
    fn name(&self) -> String {
        String::from("AGWS")
    }
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        #[cfg(feature = "shk24")]
        self.sh24
            .add_output()
            .bootstrap()
            .build::<KernelFrame<K24>>()
            .into_input(&mut self.sh24_kernel)?;
        #[cfg(feature = "shk48")]
        self.sh48
            .add_output()
            .bootstrap()
            .build::<KernelFrame<K48>>()
            .into_input(&mut self.sh48_kernel)?;
        Ok(self)
    }
    #[cfg(all(feature = "shk48", feature = "shk24"))]
    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(self.sh48.as_plain().inputs().unwrap())
            .outputs(
                self.sh24_kernel
                    .as_plain()
                    .outputs()
                    .unwrap()
                    .into_iter()
                    .chain(
                        self.sh48_kernel
                            .as_plain()
                            .outputs()
                            .unwrap_or_default()
                            .into_iter(),
                    )
                    .collect(),
            )
            .graph(self.graph())
            .build()
    }
    #[cfg(all(feature = "shk48", not(feature = "shk24")))]
    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(self.sh48.as_plain().inputs().unwrap())
            .outputs(self.sh48_kernel.as_plain().outputs().unwrap_or_default())
            .graph(self.graph())
            .build()
    }
    #[cfg(all(feature = "sh48", not(feature = "shk48")))]
    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(self.sh48.as_plain().inputs().unwrap())
            .outputs(self.sh48.as_plain().outputs().unwrap_or_default())
            .graph(self.graph())
            .build()
    }
    #[cfg(all(feature = "shk24", not(feature = "shk48")))]
    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(self.sh24.as_plain().inputs().unwrap())
            .outputs(self.sh24_kernel.as_plain().outputs().unwrap())
            .graph(self.graph())
            .build()
    }
    #[cfg(all(feature = "sh24", not(feature = "shk24")))]
    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(self.sh24.as_plain().inputs().unwrap())
            .outputs(self.sh24.as_plain().outputs().unwrap_or_default())
            .graph(self.graph())
            .build()
    }
}

impl<'a, const SH48_I: usize, const SH24_I: usize, K48, K24> IntoIterator
    for &'a Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs + 'static,
    K24: KernelSpecs + 'static,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    type Item = Box<&'a dyn Check>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            #[cfg(feature = "sh48")]
            Box::new(&self.sh48 as &dyn Check),
            #[cfg(feature = "sh24")]
            Box::new(&self.sh24 as &dyn Check),
            #[cfg(feature = "shk24")]
            Box::new(&self.sh24_kernel as &dyn Check),
            #[cfg(feature = "shk48")]
            Box::new(&self.sh48_kernel as &dyn Check),
        ]
        .into_iter()
    }
}

impl<const SH48_I: usize, const SH24_I: usize, K48, K24> IntoIterator
    for Box<Agws<SH48_I, SH24_I, K48, K24>>
where
    K48: KernelSpecs + 'static,
    K24: KernelSpecs + 'static,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    type Item = Box<dyn Task>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            #[cfg(feature = "sh48")]
            {
                Box::new(self.sh48) as Box<dyn Task>
            },
            #[cfg(feature = "sh24")]
            {
                Box::new(self.sh24) as Box<dyn Task>
            },
            #[cfg(feature = "shk24")]
            {
                Box::new(self.sh24_kernel) as Box<dyn Task>
            },
            #[cfg(feature = "shk48")]
            {
                Box::new(self.sh48_kernel) as Box<dyn Task>
            },
        ]
        .into_iter()
    }
}
#[cfg(feature = "sh48")]
impl<const SH48_I: usize, const SH24_I: usize, K48, K24> SystemInput<Sh48<SH48_I>, 1, SH48_I>
    for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn input(&mut self) -> &mut Actor<Sh48<SH48_I>, 1, SH48_I> {
        &mut self.sh48
    }
}
#[cfg(feature = "sh48")]
impl<const SH48_I: usize, const SH24_I: usize, K48, K24> SystemOutput<Sh48<SH48_I>, 1, SH48_I>
    for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn output(&mut self) -> &mut Actor<Sh48<SH48_I>, 1, SH48_I> {
        &mut self.sh48
    }
}
#[cfg(feature = "shk48")]
impl<const SH48_I: usize, const SH24_I: usize, K48, K24> SystemOutput<Sh48Kern<K48>, SH48_I, SH48_I>
    for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn output(&mut self) -> &mut Actor<Sh48Kern<K48>, SH48_I, SH48_I> {
        &mut self.sh48_kernel
    }
}

#[cfg(feature = "sh24")]
impl<const SH48_I: usize, const SH24_I: usize, K48, K24> SystemInput<Sh24<SH24_I>, 1, SH24_I>
    for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn input(&mut self) -> &mut Actor<Sh24<SH24_I>, 1, SH24_I> {
        &mut self.sh24
    }
}

#[cfg(feature = "sh24")]
impl<const SH48_I: usize, const SH24_I: usize, K48, K24> SystemOutput<Sh24<SH24_I>, 1, SH24_I>
    for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn output(&mut self) -> &mut Actor<Sh24<SH24_I>, 1, SH24_I> {
        &mut self.sh24
    }
}
#[cfg(feature = "shk24")]
impl<const SH48_I: usize, const SH24_I: usize, K48, K24> SystemOutput<Sh24Kern<K24>, SH24_I, SH24_I>
    for Agws<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
    Sh48Kern<K48>: TryUpdate,
    Sh24Kern<K24>: TryUpdate,
{
    fn output(&mut self) -> &mut Actor<Sh24Kern<K24>, SH24_I, SH24_I> {
        &mut self.sh24_kernel
    }
}
