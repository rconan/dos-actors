use gmt_dos_actors::actorscript;
use gmt_dos_clients::Signals;

mod txrx;
use txrx::{ISin, Print, Sin};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let isin: Signals = Signals::new(1, 7).channels(gmt_dos_clients::Signal::Sinusoid {
        amplitude: -10f64,
        sampling_frequency_hz: 4f64,
        frequency_hz: 1f64,
        phase_s: 0f64,
    });

    let sin_rx_print = Print;

    actorscript! {
        1: isin[ISin]>>
        1: >>[Sin] -> sin_rx_print
    }

    Ok(())
}
