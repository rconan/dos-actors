use std::fmt::Display;

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
    integrator::Integrator,
    operator::{Operator, Plus},
};
use gmt_dos_clients_io::gmt_m1::M1EdgeSensors;
use interface::Left;

#[derive(Debug, Clone)]
pub struct M1EdgeSensorsToRbm {
    adder: Actor<Operator<Vec<f64>, Plus>>,
    control: Actor<Integrator<M1EdgeSensors>>,
}

impl M1EdgeSensorsToRbm {
    pub fn new() -> Self {
        Self {
            adder: (Operator::plus(), "+").into(),
            control: Integrator::new(42).gain(1e-3).into(),
        }
    }
}

impl Display for M1EdgeSensorsToRbm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Closed-loop M1 Edge Sensors To RBM")
    }
}

impl System for M1EdgeSensorsToRbm {
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.control
            .add_output()
            .build::<Left<M1EdgeSensors>>()
            .into_input(&mut self.adder)?;
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut plain = PlainActor::new(self.name());
        if let (Some(mut control), Some(adder)) = (
            PlainActor::from(&self.control).inputs(),
            PlainActor::from(&self.adder).filter_inputs_by_name(&["Left"]),
        ) {
            control.extend(adder);
            plain = plain.inputs(control)
        };
        plain
            .outputs(PlainActor::from(&self.adder).outputs().unwrap())
            .graph(self.graph())
            .build()
    }

    fn name(&self) -> String {
        String::from("Closed-loop M1 Edge Sensors To RBM")
    }
}

impl<'a> IntoIterator for &'a M1EdgeSensorsToRbm {
    type Item = Box<&'a dyn Check>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.adder as &dyn Check),
            Box::new(&self.control as &dyn Check),
        ]
        .into_iter()
    }
}

impl IntoIterator for Box<M1EdgeSensorsToRbm> {
    type Item = Box<dyn Task>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(self.adder) as Box<dyn Task>,
            Box::new(self.control) as Box<dyn Task>,
        ]
        .into_iter()
    }
}

impl SystemInput<Integrator<M1EdgeSensors>, 1, 1> for M1EdgeSensorsToRbm {
    fn input(&mut self) -> &mut Actor<Integrator<M1EdgeSensors>, 1, 1> {
        &mut self.control
    }
}

impl SystemInput<Operator<Vec<f64>>, 1, 1> for M1EdgeSensorsToRbm {
    fn input(&mut self) -> &mut Actor<Operator<Vec<f64>>, 1, 1> {
        &mut self.adder
    }
}

impl SystemOutput<Operator<Vec<f64>>, 1, 1> for M1EdgeSensorsToRbm {
    fn output(&mut self) -> &mut Actor<Operator<Vec<f64>>, 1, 1> {
        &mut self.adder
    }
}
