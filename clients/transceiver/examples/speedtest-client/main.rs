use std::ops::Deref;

use gmt_dos_actors::prelude::*;
use gmt_dos_clients_transceiver::{Monitor, Transceiver};
use interface::{Data, Read, UID, Update};

#[derive(UID)]
#[uid(data = Vec<u8>)]
pub enum Packet {}

pub struct Payload(usize);

impl Update for Payload {}
impl Read<Packet> for Payload {
    fn read(&mut self, data: Data<Packet>) {
        let _ = data.deref();
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing::subscriber::set_global_default(
        tracing_subscriber::FmtSubscriber::builder()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .finish(),
    )?;

    let mut payload: Terminator<_> = Payload(0).into();
    let mut monitor = Monitor::new();
    let mut rx: Initiator<_> = Transceiver::<Packet>::receiver("44.235.124.92:5001", "0.0.0.0:0")?
        .run(&mut monitor)
        .into();

    rx.add_output().build().into_input(&mut payload)?;

    model!(payload, rx).check()?.run().await?;

    let _ = monitor.await?;

    Ok(())
}
