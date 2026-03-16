use std::{thread, time::Duration};

use gmt_dos_actors::prelude::*;
use gmt_dos_clients::signals::{Signal, Signals};
use gmt_dos_clients_transceiver::{Monitor, Transceiver};

mod txrx;
use txrx::{ISin, Sin};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing::subscriber::set_global_default(
        tracing_subscriber::FmtSubscriber::builder()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .finish(),
    )
    .unwrap();

    let sin: Signals = Signals::new(1, 7).channels(Signal::Sinusoid {
        amplitude: 1f64,
        sampling_frequency_hz: 4f64,
        frequency_hz: 1f64,
        phase_s: 0f64,
    });
    let mut sin: Initiator<_> = sin.into();

    let mut monitor = Monitor::new();
    let sin_tx = Transceiver::<Sin>::transmitter("127.0.0.1")?;
    let isin_tx = Transceiver::<ISin>::transmitter("127.0.0.1")?;

    let mut sin_atx: Terminator<_> = sin_tx.run(&mut monitor).into();

    let isin: Signals = Signals::new(1, 7).channels(Signal::Sinusoid {
        amplitude: -10f64,
        sampling_frequency_hz: 4f64,
        frequency_hz: 1f64,
        phase_s: 0f64,
    });
    let mut isin: Initiator<_> = isin.into();
    let mut isin_atx: Terminator<_> = isin_tx.run(&mut monitor).into();

    sin.add_output()
        .unbounded()
        .build::<Sin>()
        .into_input(&mut sin_atx)?;

    isin.add_output()
        .unbounded()
        .build::<ISin>()
        .into_input(&mut isin_atx)?;

    model!(sin, isin, sin_atx, isin_atx)
        .name("tx")
        .flowchart()
        .check()?
        .run()
        .await?;

    println!("sleeping for 10s ....");
    thread::sleep(Duration::from_secs(10));
    println!("waking up!!");

    let res = monitor.await?;
    dbg!(res);

    Ok(())
}
