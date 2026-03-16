use gmt_dos_actors::actorscript;
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
    // let mut sin: Initiator<_> = sin.into();

    let mut monitor = Monitor::new();
    let sin_tx = Transceiver::<Sin>::transmitter("127.0.0.1")?.run(&mut monitor);
    let isin_tx = Transceiver::<ISin>::transmitter("127.0.0.1")?.run(&mut monitor);

    // let mut sin_atx: Terminator<_> = sin_tx.run(&mut monitor).into();

    let isin: Signals = Signals::new(1, 7).channels(Signal::Sinusoid {
        amplitude: -10f64,
        sampling_frequency_hz: 4f64,
        frequency_hz: 1f64,
        phase_s: 0f64,
    });
    // let mut isin: Initiator<_> = isin.into();
    // let mut isin_atx: Terminator<_> = isin_tx.run(&mut monitor).into();

    // sin.add_output()
    //     .unbounded()
    //     .build::<Sin>()
    //     .into_input(&mut sin_atx)?;

    // isin.add_output()
    //     .unbounded()
    //     .build::<ISin>()
    //     .into_input(&mut isin_atx)?;

    // model!(sin, isin, sin_atx, isin_atx)
    //     .name("tx")
    //     .flowchart()
    //     .check()?
    //     .run()
    //     .await?;

    actorscript! {
        1: sin[Sin].. -> sin_tx
        1: isin[ISin].. -> isin_tx
    }

    let res = monitor.drop(sin_tx).drop(isin_tx).await?;
    dbg!(res);

    Ok(())
}
