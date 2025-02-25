use std::fmt::Display;

use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::model::{Check, Task},
    graph::Graph,
    system::{System, SystemInput, SystemOutput},
};

use gmt_dos_clients_io::Assembly;

mod dispatch;
mod segment_subsystems;
pub use dispatch::{DispatchIn, DispatchOut};

use segment_subsystems::SegmentControls;
use serde::{Deserialize, Serialize};

use crate::Calibration;

impl<const R: usize> Assembly for M1<R> {}

#[derive(Clone, Serialize, Deserialize)]
pub struct M1<const R: usize>
where
    Self: Assembly,
{
    segments: Vec<SegmentControls<R>>,
    pub dispatch_in: Actor<DispatchIn>,
    pub dispatch_out: Actor<DispatchOut>,
}

impl<'a, const R: usize> IntoIterator for &'a M1<R> {
    type Item = Box<&'a dyn Check>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.segments
            .iter()
            .map(|segment| segment.as_check())
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
impl<const R: usize> IntoIterator for Box<M1<R>> {
    type Item = Box<dyn Task>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.segments
            .into_iter()
            .map(|segment| segment.into_task())
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

impl<const R: usize> Display for M1<R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

impl<const R: usize> System for M1<R> {
    fn name(&self) -> String {
        format!("M1@{R}")
    }

    fn build(&mut self) -> anyhow::Result<&mut Self> {
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
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut g: Vec<_> = self
            .segments
            .iter()
            .map(|segment| {
                let mut s = segment.as_plain();
                s.graph = segment.graph();
                s
            })
            .collect();
        g.push(self.dispatch_in.as_plain());
        g.push(self.dispatch_out.as_plain());
        let mut plain = PlainActor::default();
        plain.client = self.name();
        plain.inputs_rate = 1;
        plain.outputs_rate = 1;
        plain.inputs = PlainActor::from(&self.dispatch_in).inputs;
        plain.outputs = PlainActor::from(&self.dispatch_out).outputs;
        plain.graph = Some(Graph::new(self.name(), g));
        plain
    }
}

impl<const R: usize> M1<R> {
    pub fn new(calibration: &Calibration) -> anyhow::Result<Self> {
        Ok(Self {
            segments: <M1<R> as Assembly>::SIDS
                .into_iter()
                .map(|sid| SegmentControls::new(sid, calibration))
                .collect::<anyhow::Result<Vec<_>>>()?,
            dispatch_in: DispatchIn::new().into(),
            dispatch_out: DispatchOut::new().into(),
        })
    }
}

impl<const R: usize> SystemInput<DispatchIn, 1, 1> for M1<R> {
    fn input(&mut self) -> &mut Actor<DispatchIn, 1, 1> {
        &mut self.dispatch_in
    }
}

impl<const R: usize> SystemOutput<DispatchOut, 1, 1> for M1<R> {
    fn output(&mut self) -> &mut Actor<DispatchOut, 1, 1> {
        &mut self.dispatch_out
    }
}
