use std::{
    future::IntoFuture,
    ops::{Deref, DerefMut},
};

use gmt_dos_actors::client::Client;
use interface::UniqueIdentifier;
use tokio::task::{JoinError, JoinHandle};

use crate::{On, Transceiver, TransceiverError, Transmitter};

/// [Transceiver](crate::Transceiver) monitor
///
/// Collect [Transceiver](crate::Transceiver) transmitter or receiver thread handles
///
#[derive(Default, Debug)]
pub struct Monitor(Vec<JoinHandle<crate::Result<()>>>);
impl Monitor {
    /// Creates a new empty [Transceiver](crate::Transceiver) monitor
    pub fn new() -> Self {
        Default::default()
    }
    /// Joins all [Transceiver](crate::Transceiver) threads
    ///
    /// Instead you can `await` on [Monitor]s
    pub async fn join(self) -> crate::Result<()> {
        for h in self.0 {
            let _ = h.await??;
        }
        Ok(())
    }
    /// Drops the [Transmitter] [client](https://docs.rs/gmt_dos-actors/latest/gmt_dos_actors/client/struct.Client.html) explicitely
    ///
    /// This is required when a [Transmitter] is used within [actorscript](https://docs.rs/gmt_dos-actors/latest/gmt_dos_actors/macro.actorscript.html) to prevent an infinite loop
    pub fn drop<'a, U: UniqueIdentifier>(
        self,
        client: Client<'a, Transceiver<U, Transmitter, On>>,
    ) -> Self {
        client
            .into_inner()
            .map(|client: Transceiver<U, Transmitter, On>| client.drop());
        self
    }
}
impl Deref for Monitor {
    type Target = Vec<JoinHandle<crate::Result<()>>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for Monitor {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl IntoFuture for Monitor {
    type Output = Result<Vec<Result<(), TransceiverError>>, JoinError>;

    type IntoFuture = futures::future::TryJoinAll<JoinHandle<Result<(), TransceiverError>>>;

    fn into_future(self) -> Self::IntoFuture {
        futures::future::try_join_all(self.0)
    }
}
