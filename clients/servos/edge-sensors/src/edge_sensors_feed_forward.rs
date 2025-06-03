use std::fmt::Display;

use crate::{HexToRbm, M2EdgeSensorsToRbm, N_ACTUATOR, RbmToShell};
use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::{
        model::{Check, FlowChart, Task},
        network::AddActorOutput,
    },
    prelude::{AddOuput, TryIntoInputs},
    system::{System, SystemError, SystemInput, SystemOutput},
};
use gmt_dos_clients::{
    low_pass_filter::LowPassFilter,
    operator::{Operator, Right},
};
use gmt_dos_clients_io::gmt_m2::asm::{M2ASMAsmCommand, M2ASMReferenceBodyNodes};
use io::{M2EdgeSensorsAsRbms, RbmAsShell};

#[derive(Debug, Clone)]
pub struct EdgeSensorsFeedForward {
    hex_to_rbm: Actor<HexToRbm>,
    m2_edge_sensors_to_rbm: Actor<M2EdgeSensorsToRbm>,
    rbm_to_shell: Actor<RbmToShell>,
    adder: Actor<Operator<f64>>,
    lowpass_filter: Actor<LowPassFilter<f64>>,
}

impl EdgeSensorsFeedForward {
    pub fn new(lag: f64) -> anyhow::Result<Self> {
        Ok(Self {
            hex_to_rbm: HexToRbm::new()?.into(),
            m2_edge_sensors_to_rbm: M2EdgeSensorsToRbm::new()?.into(),
            rbm_to_shell: RbmToShell::new()?.into(),
            adder: (Operator::new("-"), "-").into(),
            lowpass_filter: LowPassFilter::new(N_ACTUATOR * 7, lag).into(),
        })
    }
}

impl Display for EdgeSensorsFeedForward {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "M1 & M2 Edge Sensors Feed-Forward to ASM Facesheet")
    }
}

impl System for EdgeSensorsFeedForward {
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.hex_to_rbm
            .add_output()
            .build::<M2ASMReferenceBodyNodes>()
            .into_input(&mut self.m2_edge_sensors_to_rbm)?;
        self.m2_edge_sensors_to_rbm
            .add_output()
            .build::<M2EdgeSensorsAsRbms>()
            .into_input(&mut self.rbm_to_shell)?;
        self.rbm_to_shell
            .add_output()
            .build::<Right<RbmAsShell>>()
            .into_input(&mut self.adder)?;
        self.adder
            .add_output()
            .build::<M2ASMAsmCommand>()
            .into_input(&mut self.lowpass_filter)?;
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut plain = PlainActor::new(self.name());
        if let (
            Some(mut m2_edge_sensors_to_rbm),
            Some(hex_to_rbm),
            Some(rbm_to_shell),
            Some(adder),
        ) = (
            PlainActor::from(&self.m2_edge_sensors_to_rbm)
                .filter_inputs_by_name(&["M2EdgeSensors"]),
            PlainActor::from(&self.hex_to_rbm).filter_inputs_by_name(&["MCM2SmHexD"]),
            PlainActor::from(&self.rbm_to_shell).filter_inputs_by_name(&["M1EdgeSensors"]),
            PlainActor::from(&self.adder).filter_inputs_by_name(&["Left"]),
        ) {
            m2_edge_sensors_to_rbm.extend(hex_to_rbm);
            m2_edge_sensors_to_rbm.extend(rbm_to_shell);
            m2_edge_sensors_to_rbm.extend(adder);
            plain = plain.inputs(m2_edge_sensors_to_rbm);
        }
        plain
            .outputs(PlainActor::from(&self.lowpass_filter).outputs().unwrap())
            .graph(self.graph())
            .build()
    }

    fn name(&self) -> String {
        String::from("M1 & M2 Edge Sensors Feed-Forward to ASM Facesheet")
    }
}

impl<'a> IntoIterator for &'a EdgeSensorsFeedForward {
    type Item = Box<&'a dyn Check>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.hex_to_rbm as &dyn Check),
            Box::new(&self.m2_edge_sensors_to_rbm as &dyn Check),
            Box::new(&self.rbm_to_shell as &dyn Check),
            Box::new(&self.adder as &dyn Check),
            Box::new(&self.lowpass_filter as &dyn Check),
        ]
        .into_iter()
    }
}

impl IntoIterator for Box<EdgeSensorsFeedForward> {
    type Item = Box<dyn Task>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(self.hex_to_rbm) as Box<dyn Task>,
            Box::new(self.m2_edge_sensors_to_rbm) as Box<dyn Task>,
            Box::new(self.rbm_to_shell) as Box<dyn Task>,
            Box::new(self.adder) as Box<dyn Task>,
            Box::new(self.lowpass_filter) as Box<dyn Task>,
        ]
        .into_iter()
    }
}

impl SystemInput<HexToRbm, 1, 1> for EdgeSensorsFeedForward {
    fn input(&mut self) -> &mut Actor<HexToRbm, 1, 1> {
        &mut self.hex_to_rbm
    }
}

impl SystemInput<M2EdgeSensorsToRbm, 1, 1> for EdgeSensorsFeedForward {
    fn input(&mut self) -> &mut Actor<M2EdgeSensorsToRbm, 1, 1> {
        &mut self.m2_edge_sensors_to_rbm
    }
}

impl SystemInput<RbmToShell, 1, 1> for EdgeSensorsFeedForward {
    fn input(&mut self) -> &mut Actor<RbmToShell, 1, 1> {
        &mut self.rbm_to_shell
    }
}

impl SystemInput<Operator<f64>, 1, 1> for EdgeSensorsFeedForward {
    fn input(&mut self) -> &mut Actor<Operator<f64>, 1, 1> {
        &mut self.adder
    }
}

impl SystemOutput<LowPassFilter<f64>, 1, 1> for EdgeSensorsFeedForward {
    fn output(&mut self) -> &mut Actor<LowPassFilter<f64>, 1, 1> {
        &mut self.lowpass_filter
    }
}

#[cfg(test)]
mod tests {
    use gmt_dos_actors::system::Sys;

    use super::*;
    #[test]
    fn edge_sensors_feed_forward() {
        let Ok(es) = EdgeSensorsFeedForward::new(0.5) else {
            return;
        };
        let mut system = Sys::new(es).build().unwrap();
    }
}
