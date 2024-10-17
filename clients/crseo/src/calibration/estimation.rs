/*! # Command estimator

 The module defines traits to estimate a command based on a reconstructor and an optical model

## Examples

Calibration and estimation of M1 rigid body motion `Tz`

```
 use crseo::gmt::GmtM1;
 use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
 use gmt_dos_clients_crseo::{OpticalModel,
    calibration::{Calibrate, CalibrationMode, estimation::Estimation},
    sensors::WaveSensor};

let optical_model = OpticalModel::<WaveSensor>::builder();
let mut recon = <WaveSensor as Calibrate<GmtM1>>::calibrate_serial(
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
    data,
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
use gmt_dos_clients_io::optics::Wavefront;
use interface::{Read, UniqueIdentifier, Update, Write};

use crate::{
    calibration::{Calib, CalibrationError, Modality},
    sensors::WaveSensor,
    OpticalModel, OpticalModelBuilder,
};

use super::Reconstructor;

pub mod closed_loop;

/// Command estimator
///
/// Estimates the command `U` from a [Reconstructor] given an [OpticalModel]
pub trait Estimation<U> {
    type Sensor: FromBuilder;

    /// Applies the command to the [OpticalModel] and estimates it using the [Reconstructor]
    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        recon: &mut Reconstructor<M, Calib<M>>,
        cmd: Vec<f64>,
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
        cmd: Vec<f64>,
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

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::gmt::GmtM1;
    use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;

    use crate::calibration::{Calibrate, CalibrationMode};

    use super::*;

    #[test]
    fn wave_sensor() -> Result<(), Box<dyn Error>> {
        let optical_model = OpticalModel::<WaveSensor>::builder();

        let mut recon = <WaveSensor as Calibrate<GmtM1>>::calibrate(
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
            data,
        )?;
        estimate
            .chunks(6)
            .map(|c| c.iter().map(|x| x * 1e6).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("S{}: {:.0?}", i + 1, x));
        Ok(())
    }
}
