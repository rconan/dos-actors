use std::any::type_name;

use interface::{Entry, Size, TryRead, TryWrite, UniqueIdentifier, Update};
use tokio::task;

use crate::{actor::Actor, ActorError};

use super::{AddActorInput, OutputRx};

/// Assign a new entry to a logging actor
pub trait IntoLogsN<CI, const N: usize, const NO: usize>
where
    CI: Update,
{
    fn logn(self, actor: &mut Actor<CI, NO, N>, size: usize) -> Self
    where
        Self: Sized;
}

/// Assign a new entry to a logging actor
pub trait IntoLogs<CI, const N: usize, const NO: usize>
where
    CI: Update,
{
    fn log(self, actor: &mut Actor<CI, NO, N>) -> Self
    where
        Self: Sized;
}

impl<T, U, CI, CO, const N: usize, const NO: usize, const NI: usize> IntoLogsN<CI, N, NO>
    for std::result::Result<(), OutputRx<U, CO, NI, NO>>
where
    T: 'static + Send + Sync,
    U: 'static + UniqueIdentifier<DataType = T>,
    CI: 'static + TryRead<U> + Entry<U>,
    CO: 'static + TryWrite<U>,
    ActorError: From<<CI as TryRead<U>>::Error>,
    ActorError: From<<CO as TryWrite<U>>::Error>,
{
    /// Creates a new logging entry for the output
    fn logn(mut self, actor: &mut Actor<CI, NO, N>, size: usize) -> Self {
        match self {
            Ok(()) => panic!(
                r#"Input receivers have been exhausted, may be {} should be multiplexed"#,
                type_name::<U>()
            ),
            Err(OutputRx {
                hash, ref mut rxs, ..
            }) => {
                let Some(recv) = rxs.pop() else {
                    panic!(r#"Input receivers is empty"#)
                };
                // (*actor.client.lock().await).entry(size);
                task::block_in_place(|| actor.client().blocking_lock().entry(size));
                actor.add_input(recv, hash);
                if rxs.is_empty() {
                    Ok(())
                } else {
                    self
                }
            }
        }
    }
}

impl<T, U, CI, CO, const N: usize, const NO: usize, const NI: usize> IntoLogs<CI, N, NO>
    for std::result::Result<(), OutputRx<U, CO, NI, NO>>
where
    T: 'static + Send + Sync,
    U: 'static + UniqueIdentifier<DataType = T>,
    CI: 'static + TryRead<U> + Entry<U>,
    CO: 'static + TryWrite<U> + Size<U>,
    ActorError: From<<CI as TryRead<U>>::Error>,
    ActorError: From<<CO as TryWrite<U>>::Error>,
{
    /// Creates a new logging entry for the output
    fn log(mut self, actor: &mut Actor<CI, NO, N>) -> Self {
        match self {
            Ok(()) => panic!(r#"Input receivers have been exhausted"#),
            Err(OutputRx {
                hash,
                ref mut rxs,
                ref client,
                ..
            }) => {
                let Some(recv) = rxs.pop() else {
                    panic!(r#"Input receivers is empty"#)
                };
                // (*actor.client.lock().await).entry(<CO as Size<U>>::len(&*client.lock().await));
                task::block_in_place(|| {
                    actor
                        .client()
                        .blocking_lock()
                        .entry(<CO as Size<U>>::len(&*client.blocking_lock()))
                });
                actor.add_input(recv, hash);
                if rxs.is_empty() {
                    Ok(())
                } else {
                    self
                }
            }
        }
    }
}
