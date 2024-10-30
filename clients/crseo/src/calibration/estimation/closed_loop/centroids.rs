use gmt_dos_clients_io::{
    gmt_m2::asm::M2ASMAsmCommand,
    optics::{Dev, Frame, SensorData},
};
use interface::{Read, UniqueIdentifier, Update, Write};

use crate::{
    calibration::{algebra::CalibProps, CalibrationError, Modality, Reconstructor},
    centroiding::{CentroidKind, CentroidsProcessing, ZeroMean},
    sensors::{builders::CameraBuilder, Camera},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};

use super::ClosedLoopEstimation;

impl<U, K> ClosedLoopEstimation<Camera, U, CentroidsProcessing<ZeroMean>> for CentroidsProcessing<K>
where
    K: CentroidKind,
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<Camera>: Read<U>,
    CentroidsProcessing<K>: Write<SensorData>,
{
    type Sensor = Camera;

    /// Applies the command to the [OpticalModel] and estimates it using the [Reconstructor]
    /// after applying a correction with the closed-loop [OpticalModel]
    fn estimate_with_closed_loop_reconstructor<M, C>(
        optical_model: &OpticalModelBuilder<CameraBuilder>,
        closed_loop_optical_model: &OpticalModelBuilder<CameraBuilder>,
        recon: &mut Reconstructor<M, C>,
        cmd: &[f64],
        m2_to_closed_loop_sensor: &mut Reconstructor,
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Default + Send + Sync,
        C: CalibProps<M> + Default + Send + Sync + Clone,
    {
        let mut processor = CentroidsProcessing::<ZeroMean>::try_from(closed_loop_optical_model)?;
        closed_loop_optical_model.initialize(&mut processor);
        let mut com = closed_loop_optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut com, cmd.into());
        com.update();
        <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut com).map(|data| {
            <CentroidsProcessing<ZeroMean> as Read<Frame<Dev>>>::read(&mut processor, data)
        });
        processor.update();
        <CentroidsProcessing<ZeroMean> as Write<SensorData>>::write(&mut processor).map(|data| {
            <Reconstructor<_, _> as Read<SensorData>>::read(m2_to_closed_loop_sensor, data)
        });
        m2_to_closed_loop_sensor.update();
        let m2_command: Vec<_> =
            <Reconstructor as Write<M2ASMAsmCommand>>::write(m2_to_closed_loop_sensor)
                .unwrap()
                .into_arc()
                .iter()
                .map(|x| -*x)
                .collect();

        let mut processor = CentroidsProcessing::<K>::try_from(optical_model)?;
        optical_model.initialize(&mut processor);
        let mut om = optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, cmd.into());
        <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(&mut om, m2_command.into());
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
