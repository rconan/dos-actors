use std::fmt::Display;

use gmt_dos_actors::{
    actor::{Actor, Initiator, PlainActor},
    framework::{
        model::{Check, FlowChart, Task},
        network::{AddActorOutput, AddOuput, TryIntoInputs},
    },
    system::{Sys, System, SystemError, SystemOutput},
};
use gmt_dos_clients::{
    signals::{OneSignal, Signal, Signals, SignalsError},
    smooth::Weight,
};
use gmt_dos_clients_io::cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads};
use interface::Update;
use serde::{Deserialize, Serialize};

use crate::{builder::Builder, CfdLoads, WindLoadsError, FOH};

mod m1;
mod m2;
mod mount;
pub use m1::M1Smoother as M1;
pub use m2::M2Smoother as M2;
pub use mount::MountSmoother as Mount;

#[derive(Debug, thiserror::Error)]
pub enum SigmoidCfdLoadsError {
    #[error("failed to create sigmoid signal")]
    Sigmoid(#[from] SignalsError),
    #[error("failed to create CFD wind loads")]
    WindLoads(#[from] WindLoadsError),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SigmoidCfdLoads<S = FOH>
where
    CfdLoads<S>: Update,
{
    cfd_loads: Initiator<CfdLoads<S>>,
    m1_smoother: Actor<M1>,
    m2_smoother: Actor<M2>,
    mount_smoother: Actor<Mount>,
    sigmoid: Initiator<OneSignal>,
}

impl TryFrom<Builder<FOH>> for SigmoidCfdLoads {
    type Error = SigmoidCfdLoadsError;

    fn try_from(builder: Builder<FOH>) -> Result<Self, Self::Error> {
        let sampling_frequency = builder.upsampling.rate * 20;
        let m1_smoother = M1::new();
        let m2_smoother = M2::new();
        let mount_smoother = Mount::new();
        let sigmoid = OneSignal::try_from(Signals::new(1, usize::MAX).channel(
            0,
            Signal::Sigmoid {
                amplitude: 1f64,
                sampling_frequency_hz: sampling_frequency as f64,
            },
        ))?;
        Ok(Self {
            cfd_loads: builder.build()?.into(),
            m1_smoother: m1_smoother.into(),
            m2_smoother: m2_smoother.into(),
            mount_smoother: mount_smoother.into(),
            sigmoid: sigmoid.into(),
        })
    }
}

impl From<SigmoidCfdLoadsError> for SystemError {
    fn from(value: SigmoidCfdLoadsError) -> Self {
        SystemError::SubSystem(format!("{:?}", value))
    }
}

impl TryFrom<Builder<FOH>> for Sys<SigmoidCfdLoads> {
    type Error = SystemError;
    fn try_from(builder: Builder<FOH>) -> Result<Self, Self::Error> {
        Sys::new(builder.try_into()?).build()
    }
}

impl<S> Display for SigmoidCfdLoads<S>
where
    CfdLoads<S>: Update,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.cfd_loads.fmt(f)
    }
}

impl System for SigmoidCfdLoads
where
    CfdLoads<FOH>: Update,
{
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.cfd_loads
            .add_output()
            .build::<CFDM1WindLoads>()
            .into_input(&mut self.m1_smoother)?;
        self.cfd_loads
            .add_output()
            .build::<CFDM2WindLoads>()
            .into_input(&mut self.m2_smoother)?;
        self.cfd_loads
            .add_output()
            .build::<CFDMountWindLoads>()
            .into_input(&mut self.mount_smoother)?;

        self.sigmoid
            .add_output()
            .build::<Weight>()
            .into_input(&mut self.m1_smoother)?;
        self.sigmoid
            .add_output()
            .build::<Weight>()
            .into_input(&mut self.m2_smoother)?;
        self.sigmoid
            .add_output()
            .build::<Weight>()
            .into_input(&mut self.mount_smoother)?;

        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut plain = PlainActor::new(self.name());
        if let (Some(mut m1), Some(m2), Some(mount)) = (
            PlainActor::from(&self.m1_smoother).outputs(),
            PlainActor::from(&self.m2_smoother).outputs(),
            PlainActor::from(&self.mount_smoother).outputs(),
        ) {
            m1.extend(m2);
            m1.extend(mount);
            plain = plain.outputs(m1);
        };
        plain.graph(self.graph()).build()
    }

    fn name(&self) -> String {
        "CFD Wind Loads 💨".to_string()
    }
}

impl<'a> IntoIterator for &'a SigmoidCfdLoads {
    type Item = Box<&'a dyn Check>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.cfd_loads as &dyn Check),
            Box::new(&self.m1_smoother as &dyn Check),
            Box::new(&self.m2_smoother as &dyn Check),
            Box::new(&self.mount_smoother as &dyn Check),
            Box::new(&self.sigmoid as &dyn Check),
        ]
        .into_iter()
    }
}

impl IntoIterator for Box<SigmoidCfdLoads> {
    type Item = Box<dyn Task>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(self.cfd_loads) as Box<dyn Task>,
            Box::new(self.m1_smoother) as Box<dyn Task>,
            Box::new(self.m2_smoother) as Box<dyn Task>,
            Box::new(self.mount_smoother) as Box<dyn Task>,
            Box::new(self.sigmoid) as Box<dyn Task>,
        ]
        .into_iter()
    }
}

impl SystemOutput<M1, 1, 1> for SigmoidCfdLoads {
    fn output(&mut self) -> &mut Actor<M1, 1, 1> {
        &mut self.m1_smoother
    }
}

impl SystemOutput<M2, 1, 1> for SigmoidCfdLoads {
    fn output(&mut self) -> &mut Actor<M2, 1, 1> {
        &mut self.m2_smoother
    }
}

impl SystemOutput<Mount, 1, 1> for SigmoidCfdLoads {
    fn output(&mut self) -> &mut Actor<Mount, 1, 1> {
        &mut self.mount_smoother
    }
}
