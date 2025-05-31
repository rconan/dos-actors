/*!
# Scope server (`server` feature)

The scopes defined in the server module send data to the scope clients.

## Example

```ignore
use gmt_dos_clients_scope::server;

#[derive(interface::UID)]
#[uid(port = 5001)]
pub enum Signal {}

let sampling_period = 1e-3; // 1ms

let mut monitor = server::Monitor::new();
let server = server::Scope::<Signal>::builder(&mut monitor)
    .sampling_period(sampling_period)
    .build().unwrap();
```
*/

mod scope;
mod shot;
use std::{any::type_name, env, marker::PhantomData, thread, time::Duration};

pub use gmt_dos_clients_transceiver::Monitor;

use gmt_dos_clients_transceiver::{On, Transceiver, TransceiverError, Transmitter};
use interface::{UniqueIdentifier, trim_type_name};
pub use shot::{GmtShot, Shot};

use crate::{PlotScope, payload::ScopeData};

#[derive(Debug, thiserror::Error)]
pub enum ServerError {
    #[error("failed to create a transmiter for a scope server")]
    Transmitter(#[from] TransceiverError),
}

/// Builder for scope server
#[derive(Debug)]
pub struct Builder<'a, FU, K>
where
    FU: UniqueIdentifier,
    K: crate::ScopeKind,
{
    address: String,
    monitor: Option<&'a mut Monitor>,
    tau: Option<f64>,
    idx: Option<usize>,
    scale: Option<f64>,
    size: Option<[usize; 2]>,
    frame_by_frame: bool,
    minmax: Option<(f64, f64)>,
    payload: PhantomData<FU>,
    kind: PhantomData<K>,
}

impl<'a, FU, K> Default for Builder<'a, FU, K>
where
    FU: UniqueIdentifier,
    K: crate::ScopeKind,
{
    fn default() -> Self {
        Self {
            address: env::var("SCOPE_SERVER_IP").unwrap_or(crate::SERVER_IP.into()),
            monitor: Default::default(),
            tau: Default::default(),
            idx: Default::default(),
            scale: Default::default(),
            size: Default::default(),
            frame_by_frame: false,
            minmax: Default::default(),
            payload: PhantomData,
            kind: PhantomData,
        }
    }
}

impl<'a, FU, K> Builder<'a, FU, K>
where
    FU: UniqueIdentifier + 'static,
    K: crate::ScopeKind,
{
    /// Sets the server IP address
    pub fn server_ip<S: Into<String>>(mut self, server_ip: S) -> Self {
        self.address = server_ip.into();
        self
    }
    /// Selects the signal channel #
    pub fn channel(mut self, idx: usize) -> Self {
        self.idx = Some(idx);
        self
    }
    /// Sets the signal sampling period `[s]`
    pub fn sampling_period(mut self, tau: f64) -> Self {
        self.tau = Some(tau);
        self
    }
    /// Sets the signal sampling frequency `[Hz]`
    pub fn sampling_frequency(mut self, freq: f64) -> Self {
        self.tau = Some(freq.recip());
        self
    }
    /// Sets the factor to scale up the data
    pub fn scale(mut self, scale: f64) -> Self {
        self.scale = Some(scale);
        self
    }
}

/// Server for signal for plotting scope
pub type Scope<FU> = XScope<FU, crate::PlotScope>;

/// Scope server
///
/// Wraps a signal into the scope payload before sending it to the scope [client](crate::client)
#[derive(Debug)]
pub struct XScope<FU, K = PlotScope>
where
    FU: UniqueIdentifier,
{
    tx: Transceiver<ScopeData<FU>, Transmitter, On>,
    tau: f64,
    idx: Option<usize>,
    size: [usize; 2],
    minmax: Option<(f64, f64)>,
    scale: Option<f64>,
    kind: PhantomData<K>,
}

impl<FU, K> XScope<FU, K>
where
    FU: UniqueIdentifier,
    K: crate::ScopeKind + Send + Sync,
{
    /// Terminates the data transmission
    ///
    /// This process waits for all the data to have been sent
    pub fn end_transmission(&mut self) -> &mut Self {
        if let Some(tx) = self.tx.take_channel_transmitter() {
            let mut d = 1;
            while !tx.is_empty() {
                log::info!(
                    "There is still {} messages in the channel, waiting {d}s for {} to go through ...",
                    tx.len(),
                    trim_type_name::<FU>()
                );
                thread::sleep(Duration::from_secs(d));
                if d < 10 {
                    d += 1;
                }
            }
            drop(tx);
        }
        // drop(self.tx.cxtake_channel_transmitter().unwrap());
        self
    }
}
impl<FU, K> interface::Update for XScope<FU, K>
where
    FU: UniqueIdentifier,
    K: crate::ScopeKind + Send + Sync,
{
}

impl<FU: UniqueIdentifier, K> std::fmt::Display for XScope<FU, K> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Scope of {} with {}",
            type_name::<FU>(),
            type_name::<K>()
        )?;
        self.tx.fmt(f)?;
        Ok(())
    }
}
