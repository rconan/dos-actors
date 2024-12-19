use crseo::FromBuilder;
use gmt_dos_clients_io::{gmt_m2::asm::M2ASMAsmCommand, optics::Wavefront};
use interface::{Read, UniqueIdentifier, Update, Write};

use crate::{
    calibration::{algebra::CalibProps, CalibrationError, Modality, Reconstructor},
    sensors::WaveSensor,
    OpticalModel, OpticalModelBuilder,
};

use super::ClosedLoopEstimation;

impl<U> ClosedLoopEstimation<WaveSensor, U> for WaveSensor
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<WaveSensor>: Read<U>,
{
    type Sensor = WaveSensor;

    /// Applies the command to the [OpticalModel] and estimates it using the [Reconstructor]
    /// after applying a correction with the closed-loop [OpticalModel]
    fn estimate_with_closed_loop_reconstructor<M, C>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        closed_loop_optical_model: &OpticalModelBuilder<
            <WaveSensor as FromBuilder>::ComponentBuilder,
        >,
        recon: &mut Reconstructor<M, C>,
        cmd: &[f64],
        m2_to_closed_loop_sensor: &mut Reconstructor,
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        C: CalibProps<M> + Default + Send + Sync + Clone,
        M: Modality + Default + Send + Sync,
    {
        let mut com = closed_loop_optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut com, cmd.into());
        com.update();
        <OpticalModel<_> as Write<Wavefront>>::write(&mut com)
            .map(|cmd| <Reconstructor as Read<Wavefront>>::read(m2_to_closed_loop_sensor, cmd));
        m2_to_closed_loop_sensor.update();
        let m2_command: Vec<_> =
            <Reconstructor as Write<M2ASMAsmCommand>>::write(m2_to_closed_loop_sensor)
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
            .map(|cmd| <Reconstructor<M, C> as Read<Wavefront>>::read(recon, cmd));
        recon.update();
        Ok(recon.estimate.clone())
    }
}
