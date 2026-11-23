//! # Subsystem
//!
//! A [Subsystem] groups already wired actors (or systems) into a single [System],
//! without having to write a dedicated structure and its trait implementations.
//!
//! The members are given as a tuple and the actors are wired as usual before being grouped:
//! ```ignore
//! ff.add_output().build::<B>().into_input(&mut plant)?;
//! let sds = Sys::new(Subsystem::new("SDS", (ff, plant))).build()?;
//! model!(setpoint, sds).check()?.run().await?;
//! ```
//!
//! The inputs of the subsystem are the inputs of the members that are not fed by another member
//! and the outputs of the subsystem are the outputs of the members that are not, or not only,
//! read by another member.
//!
//! Cloning an [Actor](crate::actor::Actor) drops its inputs and outputs,
//! so a [Subsystem] of wired actors must not be cloned.

use std::fmt::Display;

use crate::{
    actor::{PlainActor, IO},
    framework::model::{Check, Task},
    prelude::FlowChart,
};

use super::System;

/// Generic system made of a tuple of members
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Subsystem<T> {
    name: String,
    members: T,
}

impl<T> Subsystem<T> {
    /// Creates a new subsystem from a tuple of members
    pub fn new(name: impl ToString, members: T) -> Self {
        Self {
            name: name.to_string(),
            members,
        }
    }
    /// Returns a reference to the members
    pub fn members(&self) -> &T {
        &self.members
    }
    /// Returns a mutable reference to the members
    pub fn members_mut(&mut self) -> &mut T {
        &mut self.members
    }
    /// Returns the members
    pub fn into_inner(self) -> T {
        self.members
    }
}

impl<T> Display for Subsystem<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name)
    }
}

/// Returns the inputs and outputs of a group of actors that are connected to clients outside of the group
///
/// An input is external if no member output has the same hash and an output is external
/// if it is multiplexed to more inputs than the member inputs with the same hash
pub fn boundary<'a>(
    members: impl IntoIterator<Item = Box<&'a dyn Check>>,
) -> (Option<Vec<IO>>, Option<Vec<IO>>) {
    let mut plains: Vec<PlainActor> = members.into_iter().map(|m| m._as_plain()).collect();
    let inputs: Vec<IO> = plains
        .iter_mut()
        .flat_map(|p| p.inputs().unwrap_or_default())
        .collect();
    let outputs: Vec<IO> = plains
        .iter_mut()
        .flat_map(|p| p.outputs().unwrap_or_default())
        .collect();
    let external_inputs: Vec<IO> = inputs
        .iter()
        .filter(|i| !outputs.iter().any(|o| o.hash() == i.hash()))
        .cloned()
        .collect();
    let external_outputs: Vec<IO> = outputs
        .iter()
        .filter(|o| inputs.iter().filter(|i| i.hash() == o.hash()).count() < o.len())
        .cloned()
        .collect();
    (
        (!external_inputs.is_empty()).then_some(external_inputs),
        (!external_outputs.is_empty()).then_some(external_outputs),
    )
}

impl<T> System for Subsystem<T>
where
    T: Clone + Send + Sync,
    for<'a> &'a Subsystem<T>: IntoIterator<Item = Box<&'a dyn Check>>,
{
    fn name(&self) -> String {
        self.name.clone()
    }
    fn plain(&self) -> PlainActor {
        let (inputs, outputs) = boundary(self);
        PlainActor::new(self.name.clone())
            .maybe_inputs(inputs)
            .maybe_outputs(outputs)
            .graph(self.graph())
            .build()
    }
}

macro_rules! impl_subsystem {
    ($($m:ident $i:tt),+) => {
        impl<'a, $($m: Check),+> IntoIterator for &'a Subsystem<($($m,)+)> {
            type Item = Box<&'a dyn Check>;
            type IntoIter = std::vec::IntoIter<Self::Item>;
            fn into_iter(self) -> Self::IntoIter {
                vec![$(Box::new(&self.members.$i as &dyn Check)),+].into_iter()
            }
        }
        impl<$($m: Task + 'static),+> IntoIterator for Box<Subsystem<($($m,)+)>> {
            type Item = Box<dyn Task>;
            type IntoIter = std::vec::IntoIter<Self::Item>;
            fn into_iter(self) -> Self::IntoIter {
                let members = self.members;
                vec![$(Box::new(members.$i) as Box<dyn Task>),+].into_iter()
            }
        }
    };
}

impl_subsystem!(M0 0);
impl_subsystem!(M0 0, M1 1);
impl_subsystem!(M0 0, M1 1, M2 2);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4, M5 5);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4, M5 5, M6 6);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4, M5 5, M6 6, M7 7);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4, M5 5, M6 6, M7 7, M8 8);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4, M5 5, M6 6, M7 7, M8 8, M9 9);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4, M5 5, M6 6, M7 7, M8 8, M9 9, M10 10);
impl_subsystem!(M0 0, M1 1, M2 2, M3 3, M4 4, M5 5, M6 6, M7 7, M8 8, M9 9, M10 10, M11 11);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{prelude::*, system::Sys};
    use gmt_dos_clients::{
        gain::Gain,
        logging::Logging,
        signals::{Signal, Signals},
    };
    use interface::UID;

    #[derive(UID)]
    pub enum A {}
    #[derive(UID)]
    pub enum B {}

    #[tokio::test]
    async fn subsystem() -> anyhow::Result<()> {
        let n = 100;
        let setpoint: Signals = Signals::new(1, n).channels(Signal::Constant(1.0));
        let mut setpoint: Actor<_, 0, 1> = setpoint.into();
        let mut ff: Actor<_, 1, 1> = Gain::new(vec![2.0]).into();
        let plant = Logging::<f64>::new(2).into_arcx();
        let mut plant_actor: Actor<_, 1, 0> = Actor::new(plant.clone());

        setpoint
            .add_output()
            .multiplex(2)
            .build::<A>()
            .into_input(&mut plant_actor)
            .into_input(&mut ff)?;
        ff.add_output().build::<B>().into_input(&mut plant_actor)?;

        let sds = Sys::new(Subsystem::new("SDS", (ff, plant_actor))).build()?;
        let mut plain = sds.plain();
        assert_eq!(
            plain
                .inputs()
                .unwrap()
                .iter()
                .map(|io| io.name().to_string())
                .collect::<Vec<_>>()
                .len(),
            2
        );
        assert!(plain.outputs().is_none());

        model!(setpoint, sds).check()?.run().await?;

        let log = plant.lock().await;
        assert_eq!(log.len(), n);
        Ok(())
    }

    #[derive(UID)]
    pub enum C {}

    #[tokio::test]
    async fn nested_subsystem() -> anyhow::Result<()> {
        let n = 100;
        let setpoint: Signals = Signals::new(1, n).channels(Signal::Constant(1.0));
        let mut setpoint: Actor<_, 0, 1> = setpoint.into();
        let mut pre: Actor<_, 1, 1> = Gain::new(vec![3.0]).into();
        let mut ff: Actor<_, 1, 1> = Gain::new(vec![2.0]).into();
        let plant = Logging::<f64>::new(2).into_arcx();
        let mut plant_actor: Actor<_, 1, 0> = Actor::new(plant.clone());

        setpoint.add_output().build::<A>().into_input(&mut pre)?;
        pre.add_output()
            .multiplex(2)
            .build::<C>()
            .into_input(&mut plant_actor)
            .into_input(&mut ff)?;
        ff.add_output().build::<B>().into_input(&mut plant_actor)?;

        let inner = Sys::new(Subsystem::new("INNER", (ff, plant_actor))).build()?;
        let outer = Sys::new(Subsystem::new("OUTER", (pre, inner))).build()?;

        let mut plain = outer.plain();
        let inputs: Vec<_> = plain
            .inputs()
            .unwrap()
            .iter()
            .map(|io| io.hash())
            .collect();
        assert_eq!(inputs.len(), 1);
        assert!(plain.outputs().is_none());

        let graph = outer.graph().unwrap();
        let dot = graph.to_string();
        assert!(dot.contains("INNER"));

        model!(setpoint, outer).check()?.run().await?;

        let log = plant.lock().await;
        assert_eq!(log.len(), n);
        assert_eq!(*log.as_slice(), [3f64, 6f64].repeat(n));
        Ok(())
    }
}
