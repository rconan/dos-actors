use std::fmt::Display;

pub use dispatch::{DispatchIn, DispatchOut};
use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::{
        model::{Check, SystemFlowChart, Task},
        network::ActorOutputsError,
    },
    system::{System, SystemError, SystemInput, SystemOutput},
};
use gmt_dos_clients_io::Assembly;
use serde::{Deserialize, Serialize};

mod dispatch;
mod inner_controllers;
pub use inner_controllers::FsmsInnerControllers;

impl<const R: usize> Assembly for FSMS<R> {}

/// FSMS control system
///
/// The system is made of the inner FSM controller of the 7 segments
/// and 2 dispatchers: one for the [inputs](DispatchIn) and the other for the [outputs](DispatchOut).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FSMS<const R: usize = 1>
where
    Self: Assembly,
{
    pub(crate) segments: Vec<FsmsInnerControllers<R>>,
    pub dispatch_in: Actor<DispatchIn, R, R>,
    pub dispatch_out: Actor<DispatchOut, R, R>,
}
impl<'a, const R: usize> IntoIterator for &'a FSMS<R> {
    type Item = Box<&'a dyn Check>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.segments
            .iter()
            .map(|x| x.as_check())
            .chain(
                vec![
                    Box::new(&self.dispatch_in as &dyn Check),
                    Box::new(&self.dispatch_out as &dyn Check),
                ]
                .into_iter(),
            )
            .collect::<Vec<_>>()
            .into_iter()
    }
}
impl<const R: usize> IntoIterator for Box<FSMS<R>> {
    type Item = Box<dyn Task>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.segments
            .into_iter()
            .map(|x| x.into_task())
            .chain(
                vec![
                    Box::new(self.dispatch_in) as Box<dyn Task>,
                    Box::new(self.dispatch_out) as Box<dyn Task>,
                ]
                .into_iter(),
            )
            .collect::<Vec<_>>()
            .into_iter()
    }
}

impl<const R: usize> Display for FSMS<R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

impl<const R: usize> System for FSMS<R> {
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.segments
            .iter_mut()
            .map(|segment| segment.fsm_command(&mut self.dispatch_in))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.fsm_pzt_motion(&mut self.dispatch_in))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.fsm_pzt_forces(&mut self.dispatch_out))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut plain = PlainActor::default();
        plain.client = self.name();
        plain.inputs_rate = 1;
        plain.outputs_rate = 1;
        plain.inputs = PlainActor::from(&self.dispatch_in).inputs;
        plain.outputs = PlainActor::from(&self.dispatch_out).outputs;
        plain.graph = self.graph();
        plain
    }

    fn name(&self) -> String {
        if R > 1 {
            format!("FSMS@{R}")
        } else {
            "FSMS".to_string()
        }
    }
}

impl<const R: usize> SystemInput<DispatchIn, R, R> for FSMS<R> {
    fn input(&mut self) -> &mut Actor<DispatchIn, R, R> {
        &mut self.dispatch_in
    }
}

impl<const R: usize> SystemOutput<DispatchOut, R, R> for FSMS<R> {
    fn output(&mut self) -> &mut Actor<DispatchOut, R, R> {
        &mut self.dispatch_out
    }
}
