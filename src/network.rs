use std::sync::Arc;

use crate::{
    io::{self, Assoc, UniqueIdentifier, Update},
    Actor, Result,
};
use async_trait::async_trait;

mod inputs;
mod outputs;

/// Assign inputs to actors
pub trait IntoInputs<'a, T, U, CO, const NO: usize, const NI: usize>
where
    T: 'static + Send + Sync,
    U: 'static + Send + Sync + UniqueIdentifier<DataType = T>,
    CO: 'static + Update + Send + io::Write<U>,
{
    /// Creates a new input for 'actor' from the last 'Receiver'
    fn into_input<CI, const N: usize>(self, actor: &mut Actor<CI, NO, N>) -> Self
    where
        CI: 'static + Update + Send + io::Read<U>,
        Self: Sized;
    /// Returns an error if there are any unassigned receivers
    ///
    /// Otherwise return the actor with the new output
    fn confirm(self) -> Result<&'a mut Actor<CO, NI, NO>>
    where
        Self: Sized;
}

/// Interface for data logging types
pub trait Entry<U: UniqueIdentifier> {
    /// Adds an entry to the logger
    fn entry(&mut self, size: usize);
}
/// Assign a new entry to a logging actor
#[async_trait]
pub trait IntoLogsN<CI, const N: usize, const NO: usize>
where
    CI: Update + Send,
{
    async fn logn(self, actor: &mut Actor<CI, NO, N>, size: usize) -> Self
    where
        Self: Sized;
}

/// Assign a new entry to a logging actor
#[async_trait]
pub trait IntoLogs<CI, const N: usize, const NO: usize>
where
    CI: Update + Send,
{
    async fn log(self, actor: &mut Actor<CI, NO, N>) -> Self
    where
        Self: Sized;
}

/// Actor outputs builder
pub struct ActorOutputBuilder {
    capacity: Vec<usize>,
    bootstrap: bool,
}

type Rx<U> = flume::Receiver<Arc<io::Data<U>>>;

/// Actor add output interface
pub trait AddOuput<'a, C, const NI: usize, const NO: usize>
where
    C: 'static + Update + Send,
{
    /// Sets the channel to unbounded
    fn unbounded(self) -> Self;
    /// Flags the output to be bootstrapped
    fn bootstrap(self) -> Self;
    /// Multiplexes the output `n` times
    fn multiplex(self, n: usize) -> Self;
    /// Builds the new output
    fn build<U>(self) -> (&'a mut Actor<C, NI, NO>, Vec<Rx<U>>)
    where
        C: io::Write<U>,
        U: 'static + UniqueIdentifier + Send + Sync,
        Assoc<U>: Send + Sync;
}