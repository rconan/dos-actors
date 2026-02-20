use std::env;

use gmt_dos_actors::prelude::*;
use gmt_dos_clients_transceiver::{Monitor, Transceiver};
use interface::{Data, UID, Update, Write};

#[derive(UID)]
#[uid(data = Vec<u8>)]
pub enum Packet {}

pub struct Payload {
    len: usize,
    repeat: usize,
    counter: usize,
}
impl Payload {
    pub fn new(len: usize, repeat: usize) -> Self {
        Self {
            len,
            repeat,
            counter: 0,
        }
    }
}

impl Update for Payload {}
impl Write<Packet> for Payload {
    fn write(&mut self) -> Option<Data<Packet>> {
        let Self {
            len,
            repeat,
            counter,
        } = self;
        *counter += 1;
        if *counter > *repeat {
            None
        } else {
            Some(vec![1u8; *len].into())
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing::subscriber::set_global_default(
        tracing_subscriber::FmtSubscriber::builder()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .finish(),
    )?;
    let mut args = env::args();
    args.next();
    let len = args
        .next()
        .map_or_else(|| Ok(0), |v| v.parse::<u32>())
        .map(|x| 2usize.pow(x))?;
    let repeat = args
        .next()
        .map_or_else(|| Ok(0), |v| v.parse::<u32>())
        .map(|x| 10usize.pow(x))?;
    println!("Sending a {} bytes data packet {} times", len, repeat);

    let mut payload: Initiator<_> = Payload::new(len, repeat).into();
    let mut monitor = Monitor::new();
    let mut tx: Terminator<_> = Transceiver::<Packet>::transmitter("172.31.11.38")?
        .run(&mut monitor)
        .into();

    payload.add_output().build().into_input(&mut tx)?;

    model!(payload, tx).check()?.run().await?;

    let _ = monitor.await?;

    Ok(())
}
