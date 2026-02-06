use super::S;
use crate::{ActorError, Result};
use async_trait::async_trait;
use flume::Receiver;
use interface::{TryRead, UniqueIdentifier, Who};
use std::any::type_name;
use std::fmt::Debug;
use std::{fmt::Display, sync::Arc};
use tokio::sync::Mutex;

/// [Actor](crate::Actor)s input
#[derive(Clone)]
pub(crate) struct Input<C, U, const N: usize>
where
    U: UniqueIdentifier,
    C: TryRead<U>,
{
    rx: Receiver<S<U>>,
    client: Arc<Mutex<C>>,
    hash: u64,
}
impl<C, U, const N: usize> Input<C, U, N>
where
    U: UniqueIdentifier,
    C: TryRead<U>,
{
    /// Creates a new intput from a [Receiver], an [Actor] client and an identifier [hash]
    pub fn new(rx: Receiver<S<U>>, client: Arc<Mutex<C>>, hash: u64) -> Self {
        Self { rx, client, hash }
    }
}
impl<C, U, const N: usize> Who<U> for Input<C, U, N>
where
    C: TryRead<U>,
    U: UniqueIdentifier,
{
}
impl<C, U, const N: usize> Display for Input<C, U, N>
where
    C: TryRead<U>,
    U: UniqueIdentifier,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "#{:>19}: {}", self.hash, Who::who(self))
    }
}
impl<C, U, const N: usize> Debug for Input<C, U, N>
where
    C: TryRead<U> + Debug,
    U: UniqueIdentifier,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Input")
            .field("rx", &self.rx)
            .field("client", &self.client)
            .field("hash", &self.hash)
            .finish()
    }
}

#[async_trait]
pub(crate) trait InputObject: Display + Send + Sync {
    /// Receives output data
    async fn recv(&mut self) -> Result<()>;
    /// Returns the input UID
    fn who(&self) -> String;
    /// Gets the input hash
    fn get_hash(&self) -> u64;
    fn capacity(&self) -> Option<usize>;
}

impl Debug for Box<dyn InputObject> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Debug::fmt(&std::ops::Deref::deref(&self), f)
    }
}

#[async_trait]
impl<C, U, const N: usize> InputObject for Input<C, U, N>
where
    C: TryRead<U>,
    ActorError: From<<C as TryRead<U>>::Error>,
    U: UniqueIdentifier,
{
    async fn recv(&mut self) -> Result<()> {
        // log::debug!("{} receiving", Who::highlight(self));
        // log::debug!("{} receiving (locking client)", Who::who(self));
        let mut client = self.client.lock().await;
        // log::debug!("{} receiving (client locked)", Who::who(self));
        (*client).try_read(
            self.rx
                .recv_async()
                .await
                .map_err(|e| ActorError::DropRecv {
                    msg: format!("input {} to {}", type_name::<U>(), type_name::<C>()), //Who::lite(self),
                    source: e,
                })?,
        )?;
        log::debug!(
            "{} RECV@{N}: {} - {}",
            self.hash,
            type_name::<U>(),
            type_name::<C>()
        ); // log::debug!("{} received ({})", Who::highlight(self), type_name::<C>());
        Ok(())
    }
    fn who(&self) -> String {
        Who::who(self)
    }
    fn get_hash(&self) -> u64 {
        self.hash
    }
    fn capacity(&self) -> Option<usize> {
        self.rx.capacity()
    }
}
