use gmt_dos_actors::{
    framework::model::{Check, Task},
    system::System,
};
use std::fmt::Display;

use super::GmtServoMechanisms;

impl<'a, const M1_RATE: usize, const M2_RATE: usize> IntoIterator
    for &'a GmtServoMechanisms<M1_RATE, M2_RATE>
{
    type Item = Box<&'a dyn Check>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        vec![
            Box::new(&self.fem as &dyn Check),
            Box::new(&self.mount as &dyn Check),
            Box::new(&self.m1 as &dyn Check),
            Box::new(&self.m2_positioners as &dyn Check),
            Box::new(&self.m2 as &dyn Check),
        ]
        .into_iter()
        .chain(
            self.telemetry
                .as_ref()
                .map(|telemetry| Box::new(telemetry as &dyn Check)),
        )
        .collect::<Vec<_>>()
        .into_iter()
    }
}

impl<const M1_RATE: usize, const M2_RATE: usize> IntoIterator
    for Box<GmtServoMechanisms<M1_RATE, M2_RATE>>
{
    type Item = Box<dyn Task>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(mut self) -> Self::IntoIter {
        vec![
            Box::new(self.fem) as Box<dyn Task>,
            Box::new(self.mount) as Box<dyn Task>,
            Box::new(self.m1) as Box<dyn Task>,
            Box::new(self.m2_positioners) as Box<dyn Task>,
            Box::new(self.m2) as Box<dyn Task>,
        ]
        .into_iter()
        .chain(
            self.telemetry
                .take()
                .map(|telemetry| Box::new(telemetry) as Box<dyn Task>),
        )
        .collect::<Vec<_>>()
        .into_iter()
    }
}

impl<'a, const M1_RATE: usize, const M2_RATE: usize> Display
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}
