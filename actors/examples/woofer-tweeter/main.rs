use gmt_dos_actors::prelude::*;
use gmt_dos_clients::{Signal, Signals};
use gmt_dos_clients_scope::server::{Monitor, Scope};
mod common;
use crate::common::*;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sampling_frequency_hz = 1_000.;
    let mut lofi: Initiator<_> = Signals::new(1, 4_000)
        .channels(
            Signal::Sinusoid {
                amplitude: 1.,
                sampling_frequency_hz,
                frequency_hz: 1.,
                phase_s: 0.,
            } + Signal::Sinusoid {
                amplitude: 0.25,
                sampling_frequency_hz,
                frequency_hz: 10.,
                phase_s: 0.,
            },
        )
        .into();
    let mut monitor = Monitor::new();
    let mut logging: Terminator<_> = Scope::<ResHiFi>::builder("127.0.0.1:5001", &mut monitor)
        .sampling_period(sampling_frequency_hz.recip())
        .build()?
        .into();

    let mut woofer = SubSystem::new(Woofer::new())
        .build()?
        .name("woofer")
        .flowchart();
    let mut tweeter = SubSystem::new(Tweeter::new())
        .build()?
        .name("tweeter")
        .flowchart();

    lofi.add_output()
        .build::<AddLoFi>()
        .into_input(&mut woofer)?;
    woofer
        .add_output()
        .build::<ResLoFi>()
        .into_input(&mut tweeter)?;
    tweeter
        .add_output()
        .build::<ResHiFi>()
        .into_input(&mut logging)?;

    let model = model!(lofi, logging, woofer, tweeter)
        .flowchart()
        .check()?
        .run();

    gmt_dos_clients_scope_client::Scope::new("127.0.0.1", "127.0.0.1:0")
        .signal::<ResHiFi>(5001)
        .unwrap()
        .show();

    model.await?;
    monitor.await?;

    Ok(())
}