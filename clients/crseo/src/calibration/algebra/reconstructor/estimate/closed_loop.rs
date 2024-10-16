use std::fmt::Display;

use crseo::FromBuilder;
use gmt_dos_clients_io::{gmt_m2::asm::M2ASMAsmCommand, optics::Wavefront};
use interface::{Read, UniqueIdentifier, Update, Write};

use crate::{
    calibration::{CalibrationError, ClosedLoopCalib, Modality, Reconstructor},
    sensors::WaveSensor,
    OpticalModel, OpticalModelBuilder,
};

/// Command closed-loop estimator
///
/// Estimates the command `U` from a closed-loop [Reconstructor] given an [OpticalModel]
pub trait ClosedLoopEstimate<ClosedLoopSensor: FromBuilder, U> {
    type Sensor: FromBuilder;

    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        closed_loop_optical_model: &OpticalModelBuilder<
            <ClosedLoopSensor as FromBuilder>::ComponentBuilder,
        >,
        recon: &mut Reconstructor<M, ClosedLoopCalib<M>>,
        cmd: Vec<f64>,
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Sync + Send + Default + Display;
}
impl<U> ClosedLoopEstimate<WaveSensor, U> for WaveSensor
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<WaveSensor>: Read<U>,
{
    type Sensor = WaveSensor;

    /// Applies the command to the [OpticalModel] and estimates it using the [Reconstructor]
    /// after applying a correction with the closed-loop [OpticalModel]
    fn estimate<M>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        closed_loop_optical_model: &OpticalModelBuilder<
            <WaveSensor as FromBuilder>::ComponentBuilder,
        >,
        recon: &mut Reconstructor<M, ClosedLoopCalib<M>>,
        cmd: Vec<f64>,
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Sync + Send + Default + Display,
    {
        let mut m2_to_closed_loop_sensor = Reconstructor::new(
            recon
                .calib_slice()
                .iter()
                .flat_map(|c| {
                    c.m2_to_closed_loop_sensor
                        .calib_slice()
                        .into_iter()
                        .cloned()
                        .collect::<Vec<_>>()
                })
                .collect(),
        );
        m2_to_closed_loop_sensor.pseudoinverse();
        // println!("{m2_to_closed_loop_sensor}");

        let mut com = closed_loop_optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut com, cmd.clone().into());
        com.update();
        <OpticalModel<_> as Write<Wavefront>>::write(&mut com).map(|cmd| {
            <Reconstructor as Read<Wavefront>>::read(&mut m2_to_closed_loop_sensor, cmd)
        });
        m2_to_closed_loop_sensor.update();
        let m2_command: Vec<_> =
            <Reconstructor as Write<M2ASMAsmCommand>>::write(&mut m2_to_closed_loop_sensor)
                .unwrap()
                .into_arc()
                .iter()
                .map(|x| -*x)
                .collect();

        let mut om = optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, cmd.into());
        <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(&mut om, m2_command.into());
        om.update();
        <OpticalModel<_> as Write<Wavefront>>::write(&mut om)
            .map(|cmd| <Reconstructor<M, ClosedLoopCalib<M>> as Read<Wavefront>>::read(recon, cmd));
        recon.update();
        Ok(recon.estimate.clone())
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::Gmt;
    use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
    use skyangle::Conversion;

    use crate::{
        calibration::{CalibrationMode, ClosedLoopCalibrate},
        sensors::WaveSensor,
        OpticalModel,
    };

    use super::*;

    #[test]
    fn wave_sensor() -> Result<(), Box<dyn Error>> {
        let m2_n_mode = 21;
        let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
        let optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());
        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt);

        let mut recon = <WaveSensor as ClosedLoopCalibrate<WaveSensor>>::calibrate(
            &optical_model,
            CalibrationMode::r_xy(1f64.from_arcsec()),
            &closed_loop_optical_model,
            CalibrationMode::modes(m2_n_mode, 1e-6),
        )?;
        recon.pseudoinverse();
        println!("{recon}");

        let mut data = vec![0.; 42];
        data[3] = 1f64.from_arcsec();
        let estimate =
            <WaveSensor as ClosedLoopEstimate<WaveSensor, M1RigidBodyMotions>>::estimate(
                &optical_model,
                &closed_loop_optical_model,
                &mut recon,
                data,
            )?;
        estimate
            .chunks(6)
            .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));
        Ok(())
    }
}
