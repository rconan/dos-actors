use gmt_dos_actors::actorscript;
use gmt_dos_clients::Signals;

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

    let sin: Signals = Signals::new(1, 7).channels(gmt_dos_clients::Signal::Sinusoid {
        amplitude: 1f64,
        sampling_frequency_hz: 4f64,
        frequency_hz: 1f64,
        phase_s: 0f64,
    });

    let isin: Signals = Signals::new(1, 7).channels(gmt_dos_clients::Signal::Sinusoid {
        amplitude: -10f64,
        sampling_frequency_hz: 4f64,
        frequency_hz: 1f64,
        phase_s: 0f64,
    });

    actorscript! {
        1: sin[Sin]>>
        1: isin[ISin]>>
    }

    Ok(())
}
