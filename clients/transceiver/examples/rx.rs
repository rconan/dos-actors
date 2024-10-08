use gmt_dos_actors::prelude::*;
use gmt_dos_clients_transceiver::{Monitor, Transceiver};

mod txrx;
use txrx::{Blah, ISin, Print, Sin};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing::subscriber::set_global_default(
        tracing_subscriber::FmtSubscriber::builder()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .finish(),
    )
    .unwrap();

    let mut monitor = Monitor::new();
    let sin_rx = Transceiver::<Sin>::receiver("127.0.0.1", "127.0.0.1:0")?;
    let isin_rx = sin_rx.spawn("127.0.0.1")?;

    let mut sin_arx: Initiator<_> = sin_rx.run(&mut monitor).into();
    let mut sin_rx_print: Terminator<_> = Print.into();

    let mut isin_arx: Initiator<_> = isin_rx.run(&mut monitor).into();
    let mut isin_rx_print: Terminator<_> = Print.into();

    sin_arx
        .add_output()
        .unbounded()
        .build::<Sin>()
        .into_input(&mut sin_rx_print)?;

    isin_arx
        .add_output()
        .unbounded()
        .build::<ISin>()
        .into_input(&mut isin_rx_print)?;

    model!(sin_arx, sin_rx_print, isin_arx, isin_rx_print)
        .name("rx")
        .flowchart()
        .check()?
        .run()
        .await?;

    let res = monitor.await?;
    dbg!(res);

    Ok(())
}
