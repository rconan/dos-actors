use gmt_dos_actors::actorscript;

mod txrx;
use txrx::{ISin, Print, Sin};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing::subscriber::set_global_default(
        tracing_subscriber::FmtSubscriber::builder()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .finish(),
    )
    .unwrap();

    let sin_rx_print = Print;
    let isin_rx_print = Print;

    actorscript! {
        1: >>[Sin] -> sin_rx_print
        1: >>[ISin] -> isin_rx_print
    }

    Ok(())
}
