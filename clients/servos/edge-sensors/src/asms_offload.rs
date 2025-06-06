use std::fmt::Display;

use crate::VoiceCoilToRbm;
use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::{
        model::{Check, FlowChart, Task},
        network::AddActorOutput,
    },
    prelude::{AddOuput, TryIntoInputs},
    system::{System, SystemError, SystemInput, SystemOutput},
};
use gmt_dos_clients::integrator::Integrator;
use io::M2ASMVoiceCoilsMotionAsRbms;

#[derive(Debug, Clone)]
pub struct AsmsToHexOffload {
    voice_coil_to_rbm: Actor<VoiceCoilToRbm>,
    control: Actor<Integrator<M2ASMVoiceCoilsMotionAsRbms>>,
}

impl AsmsToHexOffload {
    pub fn new(lag: f64) -> anyhow::Result<Self> {
        Ok(Self {
            voice_coil_to_rbm: VoiceCoilToRbm::new()?.into(),
            control: Integrator::new(42).gain(lag).into(),
        })
    }
    pub fn leaky(lag: f64, leak: f64) -> anyhow::Result<Self> {
        Ok(Self {
            voice_coil_to_rbm: VoiceCoilToRbm::new()?.into(),
            control: Integrator::new(42).gain(lag).forgetting_factor(leak).into(),
        })
    }
}

impl Display for AsmsToHexOffload {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "ASMS Facesheet to Positioner Off-Load")
    }
}

impl System for AsmsToHexOffload {
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.voice_coil_to_rbm
            .add_output()
            .build::<M2ASMVoiceCoilsMotionAsRbms>()
            .into_input(&mut self.control)?;
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        PlainActor::new(self.name())
            .inputs(PlainActor::from(&self.voice_coil_to_rbm).inputs().unwrap())
            .outputs(PlainActor::from(&self.control).outputs().unwrap())
            .graph(self.graph())
            .build()
    }

    fn name(&self) -> String {
        String::from("ASMS Facesheet to Positioner Off-Load")
    }
}

impl<'a> IntoIterator for &'a AsmsToHexOffload {
    type Item = Box<&'a dyn Check>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.voice_coil_to_rbm as &dyn Check),
            Box::new(&self.control as &dyn Check),
        ]
        .into_iter()
    }
}

impl IntoIterator for Box<AsmsToHexOffload> {
    type Item = Box<dyn Task>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(self.voice_coil_to_rbm) as Box<dyn Task>,
            Box::new(self.control) as Box<dyn Task>,
        ]
        .into_iter()
    }
}

impl SystemInput<VoiceCoilToRbm, 1, 1> for AsmsToHexOffload {
    fn input(&mut self) -> &mut Actor<VoiceCoilToRbm, 1, 1> {
        &mut self.voice_coil_to_rbm
    }
}

impl SystemOutput<Integrator<M2ASMVoiceCoilsMotionAsRbms>, 1, 1> for AsmsToHexOffload {
    fn output(&mut self) -> &mut Actor<Integrator<M2ASMVoiceCoilsMotionAsRbms>, 1, 1> {
        &mut self.control
    }
}
