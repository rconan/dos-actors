use std::{any::type_name, fmt::Debug, marker::PhantomData, net::SocketAddr};

use bincode::config;
use interface::{Data, UniqueIdentifier};
use quinn::Endpoint;
use tokio::task::JoinHandle;
use tracing::{debug, error, info};

use crate::{Crypto, InnerChannel, Monitor, On, Transceiver, TransceiverError, Transmitter};

impl<U: UniqueIdentifier> Transceiver<U> {
    /// [Transceiver] transmitter functionality
    ///
    /// A transmitter is build from its internet socket address
    ///
    /// # Examples
    ///
    /// ```
    /// let address = "127.0.0.1:5001";
    /// let tx = Transceiver::<IO>::transmitter(address).unwrap();
    /// ```
    pub fn transmitter<S: Into<String>>(address: S) -> crate::Result<Transceiver<U, Transmitter>> {
        TransmitterBuilder {
            server_address: address.into(),
            uid: PhantomData,
            ..Default::default()
        }
        .build()
    }
    pub fn transmitter_builder<S: Into<String>>(address: S) -> TransmitterBuilder<U> {
        TransmitterBuilder {
            server_address: address.into(),
            uid: PhantomData,
            ..Default::default()
        }
    }
}

#[cfg(feature = "flate2")]
fn encode<U>(payload: (String, Option<Vec<Data<U>>>)) -> crate::Result<Vec<u8>>
where
    U: UniqueIdentifier,
    <U as UniqueIdentifier>::DataType: Send + Sync + serde::ser::Serialize,
{
    use flate2::write::DeflateEncoder;
    use flate2::Compression;
    let zbytes: Vec<u8> = Vec::new();
    let mut e = DeflateEncoder::new(zbytes, Compression::fast());
    bincode::serde::encode_into_std_write(payload, &mut e, config::standard())?;
    let zbytes = e.finish()?;
    Ok(zbytes)
}
#[cfg(not(feature = "flate2"))]
fn encode<U>(payload: (String, Option<Vec<Data<U>>>)) -> crate::Result<Vec<u8>>
where
    U: UniqueIdentifier,
    <U as UniqueIdentifier>::DataType: Send + Sync + serde::ser::Serialize,
{
    Ok(bincode::serde::encode_to_vec(payload, config::standard())?)
}

impl<U: UniqueIdentifier + 'static> Transceiver<U, Transmitter> {
    /// Send data to the receiver
    ///
    /// Communication with the receiver happens in a separate thread.
    /// The transmitter will hold until the receiver calls in.
    pub fn run(self, monitor: &mut Monitor) -> Transceiver<U, Transmitter, On>
    where
        <U as UniqueIdentifier>::DataType: Send + Sync + serde::ser::Serialize,
    {
        let Self {
            crypto,
            mut endpoint,
            server_address,
            tx,
            mut rx,
            function,
            ..
        } = self;
        let endpoint = endpoint.take().unwrap();
        let rx = rx.take().unwrap();
        let name = crate::trim(type_name::<U>());
        let interupt = monitor.interupt.clone();
        let handle: JoinHandle<Result<(), TransceiverError>> = tokio::spawn(async move {
            // info!("<{name}>: waiting for receiver to connect");
            let stream = endpoint
                .accept()
                .await
                .expect("failed to accept a new connection");
            let connection = stream.await.map_err(|e| {
                println!("transmitter connection: {e}");
                e
            })?;
            info!(
                "<{}>: outgoing connection: {}",
                name,
                connection.remote_address()
            );
            loop {
                match connection.open_uni().await {
                    Ok(mut send) => {
                        // check if client sent data
                        let data: Vec<_> = rx.try_iter().collect();
                        if rx.is_disconnected() && data.is_empty() {
                            debug!("<{name}>: rx disconnected");
                            let bytes: Vec<u8> =
                                encode((name.to_string(), Option::<Vec<Data<U>>>::None))
                                    .map_err(|e| TransceiverError::Encode(e.to_string()))?;
                            send.write_all(&bytes).await?;
                            send.finish().await?;
                            break Ok(());
                        } else {
                            match encode((name.to_string(), Some(data))) {
                                Ok(bytes) => {
                                    send.write_all(&bytes).await?;
                                    send.finish().await?;
                                    if interupt.load(std::sync::atomic::Ordering::Relaxed) {
                                        info!("<{name}>: interupting connection");
                                        connection.close(101u32.into(), b"interupted by monitor");
                                        break Ok(());
                                    }
                                }
                                Err(e) => {
                                    error!("<{name}>: serializing failed");
                                    break Err(TransceiverError::Encode(e.to_string()));
                                }
                            };
                        }
                    }
                    Err(e) => {
                        error!("<{name}>: connection with receiver lost");
                        break Err(TransceiverError::ConnectionError(e));
                    }
                }
            }
        });
        monitor.push(handle);
        Transceiver::<U, Transmitter, On> {
            crypto,
            endpoint: None,
            server_address,
            tx,
            rx: None,
            function,
            state: PhantomData,
        }
    }
}

#[derive(Debug)]
pub struct TransmitterBuilder<U: UniqueIdentifier> {
    server_address: String,
    inner_channel: InnerChannel,
    crypto: Option<Crypto>,
    uid: PhantomData<U>,
}
impl<U: UniqueIdentifier> Default for TransmitterBuilder<U> {
    fn default() -> Self {
        Self {
            server_address: Default::default(),
            inner_channel: Default::default(),
            crypto: Default::default(),
            uid: PhantomData,
        }
    }
}
impl<U: UniqueIdentifier> TransmitterBuilder<U> {
    pub fn new<S: Into<String>>(address: S) -> Self {
        Self {
            server_address: address.into(),
            ..Default::default()
        }
    }
    pub fn crypto(mut self, crypto: Crypto) -> Self {
        self.crypto = Some(crypto);
        self
    }
    pub fn capacity(mut self, capacity: usize) -> Self {
        self.inner_channel = InnerChannel::Bounded(capacity);
        self
    }
    pub fn build(self) -> crate::Result<Transceiver<U, Transmitter>> {
        let crypto = self.crypto.unwrap_or_default();
        let server_config = crypto.server()?;
        // let address = self.server_address.parse::<SocketAddr>()?;
        let address = SocketAddr::new(self.server_address.parse()?, U::PORT as u16);
        let endpoint = Endpoint::server(server_config, address).expect(&format!(
            "Transmitter {address} error for {}",
            type_name::<U>()
        ));
        Ok(Transceiver::new(
            crypto,
            self.server_address,
            endpoint,
            self.inner_channel,
        ))
    }
}
