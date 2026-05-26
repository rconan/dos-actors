use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    iir::IIRFilter,
    signals::{Signal, Signals},
};
use interface::UID;

// cargo r --example iir_butterworth_1 --features scope
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let n_u = 3; // number of channels (e.g. X, Y, Z)
    
    // Pre-warped (recommended) — -3 dB exactly at 25 Hz
    let filter = IIRFilter::butterworth_prewarped(
        25.0 * 2.0 * std::f64::consts::PI,  // wn [rad/s]
        1.0 / 2.0_f64,                    // zeta (Butterworth)
        5e-3,                               // Ts [s]
        n_u,                        // number of channels
    );

    let n_step = 50; //1_000;
    let signal: Signals = Signals::new(n_u, n_step)
        .channel(0, Signal::Constant(1.))
        .channel(1, Signal::Constant(-1.))
        .channel(2, Signal::Constant(2.));

    actorscript!(
        1: signal[X]~ -> filter[IirOutput]~
    );

    Ok(())
}

#[derive(UID)]
#[uid(port = 5001)]
pub enum X {}

#[derive(UID)]
#[uid(port = 5002)]
pub enum IirOutput {}
