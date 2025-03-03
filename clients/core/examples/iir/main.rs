use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    iir::IIRFilter,
    signals::{Signal, Signals},
};
use interface::UID;

// cargo run -r --example iir
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Create a simple low-pass filter (example coefficients)
    let b_coeffs = vec![0.0, 0.12, 0.08];  // Feed-forward coefficients
    let a_coeffs = vec![-1.5, 0.7];      // Feedback coefficients (excluding a[0]=1.0)
    let n_u = 3;
    
    let filter = IIRFilter::new(b_coeffs, a_coeffs, n_u);

    let n_step = 50;//1_000;
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