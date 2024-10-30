/*! # Command estimator

 The module defines traits to estimate a command based on a reconstructor and an optical model

## Examples

Calibration and estimation of M1 rigid body motion `Tz`

```
 use crseo::gmt::GmtM1;
 use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
 use gmt_dos_clients_crseo::{OpticalModel,
    calibration::{Calibration, CalibrationMode, estimation::Estimation},
    sensors::WaveSensor};

let optical_model = OpticalModel::<WaveSensor>::builder();
let mut recon = <WaveSensor as Calibration<GmtM1>>::calibrate_serial(
    &optical_model,
    CalibrationMode::t_z(1e-6),
)?;
recon.pseudoinverse();
println!("{recon}");

let mut data = vec![0.; 42];
data[2] = 1e-6;
let estimate = <WaveSensor as Estimation<M1RigidBodyMotions>>::estimate(
    &optical_model,
    &mut recon,
    &data,
)?;
estimate
    .chunks(6)
    .map(|c| c.iter().map(|x| x * 1e6).collect::<Vec<_>>())
    .enumerate()
    .for_each(|(i, x)| println!("S{}: {:.0?}", i + 1, x));
# Ok::<(),Box<dyn std::error::Error>>(())
```
*/

use std::fmt::Display;

use crseo::FromBuilder;
use gmt_dos_clients_io::optics::{
    dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
    Dev, Frame, SegmentTipTilt, SensorData, Wavefront,
};
use interface::{Read, UniqueIdentifier, Update, Write};

use crate::{
    calibration::{Calib, CalibrationError, Modality},
    centroiding::{CentroidKind, CentroidsProcessing},
    sensors::{Camera, DispersedFringeSensor, SegmentGradientSensor, WaveSensor},
    DeviceInitialize, DispersedFringeSensorProcessing, OpticalModel, OpticalModelBuilder,
};

use super::Reconstructor;

pub mod closed_loop;

/// Command estimator
///
/// Estimates the command `U` from a [Reconstructor] given an [OpticalModel]
pub trait Estimation<U>
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    type Sensor: FromBuilder;

    /// Applies the command to the [OpticalModel] and estimates it using the [Reconstructor]
    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        recon: &mut Reconstructor<M, Calib<M>>,
        cmd: &[f64],
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Sync + Send + Default + Display;
}

impl<U> Estimation<U> for WaveSensor
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<WaveSensor>: Read<U>,
{
    type Sensor = WaveSensor;

    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        recon: &mut Reconstructor<M, Calib<M>>,
        cmd: &[f64],
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Sync + Send + Default + Display,
    {
        let mut om = optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, cmd.into());
        om.update();
        <OpticalModel<_> as Write<Wavefront>>::write(&mut om)
            .map(|cmd| <Reconstructor<M, Calib<M>> as Read<Wavefront>>::read(recon, cmd));
        recon.update();
        Ok(recon.estimate.clone())
    }
}
impl<U> Estimation<U> for SegmentGradientSensor
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<SegmentGradientSensor>: Read<U>,
{
    type Sensor = SegmentGradientSensor;

    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        recon: &mut Reconstructor<M, Calib<M>>,
        cmd: &[f64],
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Sync + Send + Default + Display,
    {
        let mut om = optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, cmd.into());
        om.update();
        <OpticalModel<_> as Write<SegmentTipTilt>>::write(&mut om)
            .map(|cmd| <Reconstructor<M, Calib<M>> as Read<SegmentTipTilt>>::read(recon, cmd));
        recon.update();
        Ok(recon.estimate.clone())
    }
}

impl<U, K> Estimation<U> for CentroidsProcessing<K>
where
    K: CentroidKind,
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<Camera>: Read<U>,
    CentroidsProcessing<K>: Write<SensorData>,
{
    type Sensor = Camera;

    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        recon: &mut Reconstructor<M, Calib<M>>,
        cmd: &[f64],
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Sync + Send + Default + Display,
    {
        let mut processor = CentroidsProcessing::<K>::try_from(optical_model)?;
        optical_model.initialize(&mut processor);
        let mut om = optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, cmd.into());
        om.update();
        <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut om)
            .map(|cmd| <CentroidsProcessing<K> as Read<Frame<Dev>>>::read(&mut processor, cmd));
        processor.update();
        <CentroidsProcessing<K> as Write<SensorData>>::write(&mut processor)
            .map(|data| <Reconstructor<_, _> as Read<SensorData>>::read(recon, data));
        recon.update();
        Ok(recon.estimate.clone())
    }
}

impl<U> Estimation<U> for DispersedFringeSensorProcessing
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<DispersedFringeSensor>: Read<U>,
{
    type Sensor = DispersedFringeSensor;

    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        recon: &mut Reconstructor<M, Calib<M>>,
        cmd: &[f64],
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Sync + Send + Default + Display,
    {
        let mut processor = DispersedFringeSensorProcessing::new();
        optical_model.initialize(&mut processor);
        let mut om = optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, cmd.into());
        om.update();
        <OpticalModel<_> as Write<DfsFftFrame<Dev>>>::write(&mut om).map(|cmd| {
            <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(&mut processor, cmd)
        });
        processor.update();
        <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut processor)
            .map(|data| <Reconstructor<_, _> as Read<Intercepts>>::read(recon, data));
        recon.update();
        Ok(recon.estimate.clone())
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{gmt::GmtM1, imaging::LensletArray};
    use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
    use skyangle::Conversion;

    use crate::calibration::{Calibration, CalibrationMode};

    use super::*;

    #[test]
    fn wave_sensor() -> Result<(), Box<dyn Error>> {
        let optical_model = OpticalModel::<WaveSensor>::builder();

        let mut recon = <WaveSensor as Calibration<GmtM1>>::calibrate(
            &optical_model,
            CalibrationMode::t_z(1e-6),
        )?;
        recon.pseudoinverse();
        println!("{recon}");

        let mut data = vec![0.; 42];
        data[2] = 1e-6;
        let estimate = <WaveSensor as Estimation<M1RigidBodyMotions>>::estimate(
            &optical_model,
            &mut recon,
            &data,
        )?;
        estimate
            .chunks(6)
            .map(|c| c.iter().map(|x| x * 1e6).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("S{}: {:.0?}", i + 1, x));
        Ok(())
    }

    #[test]
    fn centroiding() -> Result<(), Box<dyn Error>> {
        let sh48 = Camera::builder()
            .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32))
            .lenslet_flux(0.75);
        let optical_model = OpticalModel::<Camera<1>>::builder().sensor(sh48);

        let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
            &(&optical_model).into(),
            CalibrationMode::r_xy(100f64.from_mas()),
        )?;
        recon.pseudoinverse();
        println!("{recon}");

        let mut data = vec![0.; 42];
        data[3] = 100f64.from_mas();
        let estimate = <CentroidsProcessing as Estimation<M1RigidBodyMotions>>::estimate(
            &optical_model,
            &mut recon,
            &data,
        )?;
        estimate
            .chunks(6)
            .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("S{}: {:.0?}", i + 1, x));
        Ok(())
    }
}
