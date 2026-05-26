//use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    iir::IIRFilter,
};

// cargo run --example iir_butterworth_0
/// Example: construct a 2nd-order Butterworth IIR filter
/// using the bilinear transform, with and without pre-warping.

fn main() {
    let wn         = 25.0 * 2.0 * std::f64::consts::PI;  // 25 Hz in rad/s
    let zeta       = 1.0 / 2.0_f64.sqrt();               // Butterworth damping
    let ts         = 5e-3;                                // 1 kHz sampling
    let channels   = 3;                                   // e.g. X, Y, Z

    // --- Without pre-warping ---
    let filter_nw = IIRFilter::butterworth(wn, zeta, ts, channels);
    println!("Without pre-warping: {:?}", filter_nw);

    // --- With pre-warping (exact -3 dB at wn) ---
    let filter_pw = IIRFilter::butterworth_prewarped(wn, zeta, ts, channels);
    println!("With pre-warping:    {:?}", filter_pw);
}
