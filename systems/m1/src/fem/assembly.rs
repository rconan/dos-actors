use std::fmt::Display;

use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::{
        model::{Check, Task},
        network::ActorOutputsError,
    },
    graph::Graph,
    system::{Sys, System, SystemError, SystemInput, SystemOutput},
};

use gmt_dos_clients_io::Assembly;

mod dispatch;
mod segment_controls;
pub use dispatch::{DispatchIn, DispatchOut};

use segment_controls::SegmentControls;
use serde::{Deserialize, Serialize};

use crate::Calibration;

use super::M1Builder;

impl<const R: usize> Assembly for M1<R> {}

#[derive(Clone, Serialize, Deserialize)]
pub struct M1<const R: usize>
where
    Self: Assembly,
{
    pub(crate) segments: Vec<SegmentControls<R>>,
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

    fn build(&mut self) -> Result<&mut Self, SystemError> {
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_rigid_body_motions(&mut self.dispatch_in))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_actuator_command_forces(&mut self.dispatch_in))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;

        #[cfg(not(m1_hp_force_extension))]
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_hardpoints_motion(&mut self.dispatch_in))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        #[cfg(m1_hp_force_extension)]
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_hardpoints_forces_in(&mut self.dispatch_in))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;

        self.segments
            .iter_mut()
            .map(|segment| segment.m1_actuator_applied_forces(&mut self.dispatch_out))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        #[cfg(not(m1_hp_force_extension))]
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_hardpoints_forces(&mut self.dispatch_out))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        #[cfg(m1_hp_force_extension)]
        self.segments
            .iter_mut()
            .map(|segment| segment.m1_hardpoints_motion(&mut self.dispatch_out))
            .collect::<Result<Vec<()>, ActorOutputsError>>()?;
        Ok(self)
    }

    fn plain(&self) -> gmt_dos_actors::actor::PlainActor {
        let mut g: Vec<_> = self
            .segments
            .iter()
            .map(|segment| {
                segment.as_plain()
                // s.graph = segment.graph();
            })
            .collect();
        g.push(self.dispatch_in.as_plain());
        g.push(self.dispatch_out.as_plain());
        PlainActor::new(self.name())
            .inputs(PlainActor::from(&self.dispatch_in).inputs().unwrap())
            .outputs(PlainActor::from(&self.dispatch_out).outputs().unwrap())
            .graph(Some(Graph::new(self.name(), g)))
            .build()
    }
}

impl<const R: usize> M1<R> {
    pub fn new(calibration: &Calibration) -> Result<Self, SystemError> {
        Ok(Self {
            segments: <M1<R> as Assembly>::SIDS
                .into_iter()
                .map(|sid| SegmentControls::new(sid, calibration))
                .collect::<Result<Vec<_>, SystemError>>()?,
            dispatch_in: DispatchIn::new().into(),
            dispatch_out: DispatchOut::new().into(),
        })
    }
}
impl<const R: usize> M1Builder<R> {
    pub fn build(self) -> std::result::Result<Sys<M1<R>>, SystemError> {
        let m1 = M1 {
            segments: <M1<R> as Assembly>::SIDS
                .into_iter()
                .map(|sid| SegmentControls::new(sid, &self.calibration))
                .collect::<Result<Vec<_>, SystemError>>()?,
            dispatch_in: if let Some(mats) = self.mode_2_force_transforms {
                DispatchIn::new().modes_to_forces(mats)
            } else {
                DispatchIn::new()
            }
            .into(),
            dispatch_out: DispatchOut::new().into(),
        };
        Sys::new(m1).build()
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
