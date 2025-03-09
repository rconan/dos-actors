//! # System
//!
//! A system is a collection of actors that are hidden behind the [Sys] client.
//! The actors of a system are given as fields of a user-defined structure `S` that is passed to [Sys].
//!
//! The user-defined structure `S` must implement the following traits:
//!   * [System]` for S`
//!   * [IntoIterator]`<`[Box]`<&'a dyn `[Check]>>` for &'a S`
//!   * [IntoIterator]`<`[Box]`<dyn `[Task]`>> for `[Box]`<S>`
//!   * [SystemInput]`<Gateway> for S`
//!   * [SystemOutput]`<Gateway> for S`
//!
//! `Gateway` is a system's actor that receives inputs from other clients to this system ([SystemInput]`<Gateway>`)
//! or send outputs from the system to other clients (([SystemOutput]`<Gateway>`))

use std::marker::PhantomData;
use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use crate::actor::Actor;
use crate::framework::model::{Check, SystemFlowChart, Task};
use crate::framework::network::{ActorOutputsError, OutputRx};

mod implementations;
mod interfaces;

pub use interfaces::{System, SystemInput, SystemOutput};

pub enum New {}
pub enum Built {}

#[derive(Debug, thiserror::Error)]
pub enum SystemError {
    #[error("failed to build system")]
    Ouputs(#[from] ActorOutputsError),
    #[error("{0}")]
    SubSystem(String),
}
impl<U, CO, const NO: usize, const NI: usize> From<OutputRx<U, CO, NI, NO>> for SystemError
where
    U: 'static + interface::UniqueIdentifier,
    CO: interface::Write<U>,
{
    fn from(value: OutputRx<U, CO, NI, NO>) -> Self {
        SystemError::Ouputs(ActorOutputsError {
            actor: value.actor,
            output: value.output,
        })
    }
}

/// System client  
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sys<T: System, S = Built> {
    pub sys: T,
    state: PhantomData<S>,
}

impl<T: System, S> Clone for Sys<T, S> {
    fn clone(&self) -> Self {
        let mut sys = self.sys.clone();
        sys.build().unwrap();
        Self {
            sys,
            state: PhantomData,
        }
    }
}

impl<T: System> Deref for Sys<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.sys
    }
}

impl<T: System> DerefMut for Sys<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.sys
    }
}
impl<T: System> Sys<T, New> {
    pub fn new(sys: T) -> Self {
        Self {
            sys,
            state: PhantomData,
        }
    }

    pub fn build(self) -> Result<Sys<T>, SystemError> {
        let mut this: Sys<T> = Sys {
            sys: self.sys,
            state: PhantomData,
        };
        this.sys.build()?;
        Ok(this)
    }
}
impl<T: System + SystemFlowChart> Sys<T> {
    /*     pub fn flowchart(self) -> Self {
        self.sys.flowchart();
        self
    }
    pub fn sys_flowchart(&self) {
        self.sys.flowchart();
    } */
    pub fn sys_graph(&self) -> Option<crate::graph::Graph> {
        self.sys.graph()
    }
}

impl<T: System> Display for Sys<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.sys)
    }
}

impl<'a, T: System> IntoIterator for &'a Sys<T>
where
    &'a T: IntoIterator<
        Item = Box<&'a dyn Check>,
        IntoIter = std::vec::IntoIter<<&'a T as IntoIterator>::Item>,
    >,
{
    type Item = Box<&'a dyn Check>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.sys.into_iter()
    }
}

impl<T: System> IntoIterator for Box<Sys<T>>
where
    Box<T>: IntoIterator<
        Item = Box<dyn Task>,
        IntoIter = std::vec::IntoIter<<Box<T> as IntoIterator>::Item>,
    >,
{
    type Item = Box<dyn Task>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        let q = *self;
        let w = q.sys;
        let b = Box::new(w);
        b.into_iter()
    }
}

impl<
        T: System + SystemInput<C, NI, NO>,
        C: interface::Update,
        const NI: usize,
        const NO: usize,
    > SystemInput<C, NI, NO> for Sys<T>
{
    fn input(&mut self) -> &mut Actor<C, NI, NO> {
        self.sys.input()
    }
}

impl<
        T: System + SystemOutput<C, NI, NO>,
        C: interface::Update,
        const NI: usize,
        const NO: usize,
    > SystemOutput<C, NI, NO> for Sys<T>
{
    fn output(&mut self) -> &mut Actor<C, NI, NO> {
        self.sys.output()
    }
}

#[cfg(feature = "filing")]
impl<T> interface::filing::Codec for Sys<T> where
    T: Sized + System + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>
{
}
