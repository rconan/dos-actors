use std::fmt::Display;

use gmt_dos_actors::{
    actor::{Actor, PlainActor},
    framework::model::{Check, SystemFlowChart, Task},
    system::{System, SystemInput, SystemOutput},
};
use gmt_dos_clients_io::Assembly;

mod dispatch;
mod inner_controllers;
pub use dispatch::{DispatchIn, DispatchOut};
pub use inner_controllers::AsmsInnerControllers;
use serde::{Deserialize, Serialize};

use crate::{AsmsBuilder, M2CtrlError};

impl<const R: usize> Assembly for ASMS<R> {}

/// ASMS control system
///
/// The system is made of the inner ASM controller of the 7 segments
/// and 2 dispatchers: one for the [inputs](DispatchIn) and the other for the [outputs](DispatchOut).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ASMS<const R: usize = 1>
where
    Self: Assembly,
{
    segments: Vec<AsmsInnerControllers<R>>,
    pub dispatch_in: Actor<DispatchIn, R, R>,
    pub dispatch_out: Actor<DispatchOut, R, R>,
}

impl<'a, const R: usize> IntoIterator for &'a ASMS<R> {
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
impl<const R: usize> IntoIterator for Box<ASMS<R>> {
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

pub type Result<T> = std::result::Result<T, M2CtrlError>;

impl<'a, const R: usize> TryFrom<AsmsBuilder<'a, R>> for ASMS<R> {
    type Error = anyhow::Error;
    fn try_from(builder: AsmsBuilder<'a, R>) -> anyhow::Result<ASMS<R>> {
        let iter = builder
            .gain
            .into_iter()
            .map(|x| Ok::<_, M2CtrlError>(x.try_inverse().ok_or(M2CtrlError::InverseStiffness)?));
        let ks = if let Some(modes) = builder.modes {
            iter.zip(modes.into_iter())
                .map(|(x, modes)| {
                    let modes_t = modes.transpose();
                    x.map(|x| modes_t * x * modes)
                })
                .map(|x| x.map(|x| x.as_slice().to_vec()))
                .map(|x| x.map(|x| Some(x)))
                .collect::<Result<Vec<_>>>()?
        } else {
            iter.map(|x| x.map(|x| x.as_slice().to_vec()))
                .map(|x| x.map(|x| Some(x)))
                .collect::<Result<Vec<_>>>()?
        };

        let n_mode: Vec<_> = ks
            .iter()
            .filter_map(|x| x.as_ref().map(|x| (x.len() as f64).sqrt() as usize))
            .collect();

        Ok(ASMS {
            segments: n_mode
                .clone()
                .into_iter()
                .zip(ks.into_iter())
                .zip(<ASMS<R> as Assembly>::SIDS.into_iter())
                .map(|((n_mode, ks), sid)| AsmsInnerControllers::new(sid, n_mode, ks))
                .collect::<Vec<_>>(),
            dispatch_in: DispatchIn::new(n_mode.clone()).into(),
            dispatch_out: DispatchOut::new(n_mode).into(),
        })
    }
}

impl<const R: usize> Display for ASMS<R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

impl<const R: usize> System for ASMS<R> {
    fn build(&mut self) -> anyhow::Result<&mut Self> {
        self.segments
            .iter_mut()
            .map(|segment| segment.asm_command(&mut self.dispatch_in))
            .collect::<anyhow::Result<Vec<()>>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.asm_voice_coils_motion(&mut self.dispatch_in))
            .collect::<anyhow::Result<Vec<()>>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.asm_voice_coils_forces(&mut self.dispatch_out))
            .collect::<anyhow::Result<Vec<()>>>()?;
        self.segments
            .iter_mut()
            .map(|segment| segment.asm_fluid_damping_forces(&mut self.dispatch_out))
            .collect::<anyhow::Result<Vec<()>>>()?;
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
            format!("ASMS@{R}")
        } else {
            "ASMS".to_string()
        }
    }
}

impl<const R: usize> SystemInput<DispatchIn, R, R> for ASMS<R> {
    fn input(&mut self) -> &mut Actor<DispatchIn, R, R> {
        &mut self.dispatch_in
    }
}

impl<const R: usize> SystemOutput<DispatchOut, R, R> for ASMS<R> {
    fn output(&mut self) -> &mut Actor<DispatchOut, R, R> {
        &mut self.dispatch_out
    }
}
