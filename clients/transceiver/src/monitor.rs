use std::{
    future::IntoFuture,
    sync::{atomic::AtomicBool, Arc},
};

use tokio::task::{JoinError, JoinHandle};

use crate::TransceiverError;

/// [Transceiver](crate::Transceiver) monitor
///
/// Collect [Transceiver](crate::Transceiver) transmitter or receiver thread handles
///
#[derive(Debug)]
pub struct Monitor {
    handles: Vec<JoinHandle<crate::Result<()>>>,
    pub interupt: Arc<AtomicBool>,
}
impl Monitor {
    /// Creates a new empty [Transceiver](crate::Transceiver) monitor
    pub fn new() -> Self {
        Self {
            handles: Vec::new(),
            interupt: Arc::new(AtomicBool::new(false)),
        }
    }
    /// Joins all [Transceiver](crate::Transceiver) threads
    ///
    /// Instead you can `await` on [Monitor]s
    pub async fn join(self) -> crate::Result<()> {
        self.interupt
            .store(true, std::sync::atomic::Ordering::Relaxed);
        for h in self.handles {
            let _ = h.await??;
        }
        Ok(())
    }
    pub fn push(&mut self, handle: JoinHandle<crate::Result<()>>) {
        self.handles.push(handle);
    }
}

impl IntoFuture for Monitor {
    type Output = Result<Vec<Result<(), TransceiverError>>, JoinError>;

    type IntoFuture = futures::future::TryJoinAll<JoinHandle<Result<(), TransceiverError>>>;

    fn into_future(self) -> Self::IntoFuture {
        self.interupt
            .store(true, std::sync::atomic::Ordering::Relaxed);
        futures::future::try_join_all(self.handles)
    }
}
