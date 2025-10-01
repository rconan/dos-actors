use crseo::{FromBuilder, Source, gmt::GmtM2, imaging::LensletArray};
use gmt_dos_clients_crseo::{
    OpticalModel,
    calibration::{Calibration, CalibrationMode, ClosedLoopCalibration},
    centroiding::{CentroidsProcessing, ZeroMean},
    sensors::{Camera, WaveSensor},
};
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let zenith: Vec<_> = vec![7f32, 5f32, 8f32]
        .into_iter()
        .map(|x| x.from_arcmin())
        .collect();

    let azimuth: Vec<_> = vec![-10f32, 95f32, 255f32]
        .into_iter()
        .map(|x| x.to_radians())
        .collect();
    let agws_gs = Source::builder()
        .size(3)
        // .on_ring(6f32.from_arcmin())
        .zenith_azimuth(zenith, azimuth);
    let omb = OpticalModel::<Camera>::builder()
        .sensor(
            Camera::builder()
                .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(16)),
        )
        .source(agws_gs);
    let rbm = CalibrationMode::RBM([Some(1e-6), Some(1e-6), Some(1e-6), None, None, None]);
    let mut recon =
        <CentroidsProcessing as Calibration<GmtM2>>::calibrate(&(omb.clone().into()), rbm)?;
    recon.pseudoinverse();
    println!("{recon}");

    let closed_loop_omb = OpticalModel::<WaveSensor>::builder();

    let mut recon =
        <CentroidsProcessing as ClosedLoopCalibration<GmtM2, WaveSensor>>::calibrate_serial(
            &(omb.into()),
            rbm,
            &closed_loop_omb,
            CalibrationMode::r_xy(1e-6),
        )?;
    recon.pseudoinverse();
    println!("{recon}");
    Ok(())
}
