/*!
# GMT DOS Actors Transceiver

`gmt_dos-clients_transceiver` provides implementation for two GMT DOS actors clients: a [Transmitter]
and a [Receiver] allowing to transfer [Data] between GMT DOS actors models through the network.

The communication betweem the transmitter and the receiver is secured by procuring a signed certificate
shared by both the transmitter and the receiver and a private key for the transmitter only (see also [Crypto]).

The certificate and the private key are generated with
`
cargo run --bin crypto
`

[Data]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/interface/struct.Data.html
*/

mod crypto;
mod monitor;
mod receiver;
mod transmitter;

use std::{any::type_name, marker::PhantomData};

use interface::{Data, Read, UniqueIdentifier, Update, Write};
use quinn::Endpoint;

pub use crypto::Crypto;
pub use monitor::Monitor;
pub use receiver::CompactRecvr;
pub use transmitter::TransmitterBuilder;

#[derive(Debug, thiserror::Error)]
pub enum TransceiverError {
    #[error("failed to parse IP socket address")]
    Socket(#[from] std::net::AddrParseError),
    // #[error("failed to bind endpoint to socket address")]
    // IO(#[from] std::io::Result<Endpoint>),
    #[error("connection failed")]
    ConnectionError(#[from] quinn::ConnectionError),
    #[error("failed to connect")]
    ConnectError(#[from] quinn::ConnectError),
    #[error(transparent)]
    IO(#[from] std::io::Error),
    #[error("encryption failed")]
    Crypto(#[from] rustls::Error),
    #[error("failed to send data to receiver")]
    SendToRx(#[from] quinn::WriteError),
    #[error("data serialization failed ({0})")]
    Encode(String),
    #[error("data deserialization failed ({0})")]
    Decode(String),
    #[error("failed to read data from transmitter")]
    RecvFromTx(#[from] quinn::ReadToEndError),
    #[error("failed to join task")]
    Join(#[from] tokio::task::JoinError),
    #[error("expected {0}, received {1}")]
    DataMismatch(String, String),
    #[error("{0} stream ended: {1} in {2} ({3}/s)")]
    StreamEnd(String, String, String, String),
    #[error("failed to encode data")]
    BincodeEncode(#[from] bincode::error::EncodeError),
    #[error("failed to decode data")]
    BincodeDecode(#[from] bincode::error::DecodeError),
    #[error("")]
    Duration(#[from] quinn_proto::VarIntBoundsExceeded),
}
pub type Result<T> = std::result::Result<T, TransceiverError>;

/// Receiver functionality of a [Transceiver]
pub enum Receiver {}
/// Transmitter functionality of a [Transceiver]
pub enum Transmitter {}
/// [Transceiver] without purpose
pub enum Unset {}
trait RxOrTx {}
impl RxOrTx for Transmitter {}
impl RxOrTx for Receiver {}

pub enum On {}
pub enum Off {}

#[derive(Default, Debug)]
pub enum InnerChannel {
    Bounded(usize),
    #[default]
    Unbounded,
}

/// Transmitter and receiver of [gmt_dos-actors](https://docs.rs/gmt_dos-actors/) [Data](https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/interface/struct.Data.html)
pub struct Transceiver<U: UniqueIdentifier, F = Unset, S = Off> {
    crypto: Crypto,
    endpoint: Option<quinn::Endpoint>,
    server_address: String,
    tx: Option<flume::Sender<Data<U>>>,
    pub rx: Option<flume::Receiver<Data<U>>>,
    function: PhantomData<F>,
    state: PhantomData<S>,
}
impl<U: UniqueIdentifier, F> Transceiver<U, F> {
    pub fn new<S: Into<String>>(
        crypto: Crypto,
        server_address: S,
        endpoint: Endpoint,
        inner_channel: InnerChannel,
    ) -> Self {
        let (tx, rx) = match inner_channel {
            InnerChannel::Bounded(cap) => flume::bounded(cap),
            InnerChannel::Unbounded => flume::unbounded(),
        };
        Self {
            crypto,
            server_address: server_address.into(),
            endpoint: Some(endpoint),
            tx: Some(tx),
            rx: Some(rx),
            function: PhantomData,
            state: PhantomData,
        }
    }
}
impl<U: UniqueIdentifier, F, S> Transceiver<U, F, S> {
    pub fn take_channel_receiver(&mut self) -> Option<flume::Receiver<Data<U>>> {
        self.rx.take()
    }
    pub fn take_channel_transmitter(&mut self) -> Option<flume::Sender<Data<U>>> {
        self.tx.take()
    }
}

impl<U: UniqueIdentifier, F, S> std::fmt::Debug for Transceiver<U, F, S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Transceiver")
            .field("crypto", &self.crypto)
            .field("endpoint", &self.endpoint)
            .field("server_address", &self.server_address)
            .field("tx", &self.tx)
            .field("rx", &self.rx)
            .field("function", &self.function)
            .field("state", &self.state)
            .finish()
    }
}

impl<U: UniqueIdentifier, F, S> std::fmt::Display for Transceiver<U, F, S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Transceiver[{}] @ {}",
            type_name::<F>(),
            self.server_address
        )?;
        if let Some(rx) = &self.rx {
            if rx.is_disconnected() {
                writeln!(f, " . RX is disconnected")?;
            } else {
                if rx.is_empty() {
                    writeln!(f, " . RX is empty")?;
                } else {
                    if rx.is_full() {
                        writeln!(f, " . RX is full")?;
                    } else {
                        writeln!(f, " . RX is holding {} messages", rx.len())?;
                    }
                }
            }
        }
        if let Some(tx) = &self.tx {
            if tx.is_disconnected() {
                writeln!(f, " . TX is disconnected")?;
            } else {
                if tx.is_empty() {
                    writeln!(f, " . TX is empty")?;
                } else {
                    if tx.is_full() {
                        writeln!(f, " . TX is full")?;
                    } else {
                        writeln!(f, " . TX is holding {} messages", tx.len())?;
                    }
                }
            }
        }
        Ok(())
    }
}

/* impl<U: UniqueIdentifier, V: UniqueIdentifier, F> From<&Transceiver<U, F>> for Transceiver<V, F> {
    fn from(other: &Transceiver<U, F>) -> Self {
        let (tx, rx) = flume::unbounded();
        Self {
            crypto: other.crypto.clone(),
            server_address: other.server_address.clone(),
            endpoint: other.endpoint.clone(),
            tx: Some(tx),
            rx: Some(rx),
            function: PhantomData,
            state: PhantomData,
        }
    }
} */

impl<U: UniqueIdentifier, F: RxOrTx + Send + Sync> Update for Transceiver<U, F, On> {}

impl<U: UniqueIdentifier> Read<U> for Transceiver<U, Transmitter, On> {
    fn read(&mut self, data: Data<U>) {
        if let Some(tx) = self.tx.as_ref() {
            let _ = tx.send(data);
        }
    }
}

impl<U: UniqueIdentifier> Write<U> for Transceiver<U, Receiver, On> {
    fn write(&mut self) -> Option<Data<U>> {
        // if let Some(rx) = self.rx.as_ref() {
        //     if let Ok(data) = rx.recv() {
        //         info!("data forwarded");
        //         Some(data)
        //     } else {
        //         info!("rx failed");
        //         None
        //     }
        // } else {
        //     info!("no rx");
        //     None
        // }
        self.rx.as_ref().and_then(|rx| rx.recv().ok())
    }
}

pub fn trim(name: &str) -> String {
    if let Some((prefix, suffix)) = name.split_once('<') {
        let generics: Vec<_> = suffix.split(',').map(|s| trim(s)).collect();
        format!("{}<{}", trim(prefix), generics.join(","))
    } else {
        if let Some((_, suffix)) = name.rsplit_once("::") {
            suffix.into()
        } else {
            name.into()
        }
    }
}
