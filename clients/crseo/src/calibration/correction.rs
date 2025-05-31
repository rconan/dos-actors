use std::sync::Arc;

use crseo::{Builder, FromBuilder};
use gmt_dos_clients_io::{gmt_m2::asm::M2ASMAsmCommand, optics::Wavefront};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

use crate::{
    DispersedFringeSensorProcessing, OpticalModel, OpticalModelBuilder,
    sensors::{
        WaveSensor,
        builders::{SensorBuilderProperty, WaveSensorBuilder},
    },
};

use super::{CalibrationError, Reconstructor};

pub trait Correction<U>
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<WaveSensor>: Read<U>,
{
    fn correct<S: SensorBuilderProperty + Builder>(
        optical_model: &OpticalModelBuilder<S>,
        command: &[f64],
        estimate: Arc<Vec<f64>>,
    ) -> Result<Arc<Vec<f64>>, CalibrationError> {
        let residual: Vec<_> = command
            .iter()
            .zip(estimate.iter())
            .map(|(c, e)| *c - *e)
            .collect();
        let mut om = OpticalModelBuilder::<WaveSensorBuilder>::from(optical_model).build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, Data::new(residual));
        om.update();
        Ok(<OpticalModel<_> as Write<Wavefront>>::write(&mut om)
            .unwrap()
            .into_arc())
    }
}

impl<U> Correction<U> for DispersedFringeSensorProcessing
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<WaveSensor>: Read<U>,
{
}

pub trait ClosedLoopCorrection<U>
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<WaveSensor>: Read<U>,
{
    fn correct<S: SensorBuilderProperty + Builder>(
        optical_model: &OpticalModelBuilder<S>,
        command: &[f64],
        estimate: Arc<Vec<f64>>,
        closed_loop_optical_model: &OpticalModelBuilder<
            <WaveSensor as FromBuilder>::ComponentBuilder,
        >,
        mut m2_to_closed_loop_sensor: Reconstructor,
    ) -> Result<Arc<Vec<f64>>, CalibrationError> {
        let residual: Vec<_> = command
            .iter()
            .zip(estimate.iter())
            .map(|(c, e)| *c - *e)
            .collect();

        let mut com = closed_loop_optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut com, residual.clone().into());
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

        let mut om = OpticalModelBuilder::<WaveSensorBuilder>::from(optical_model).build()?;
        <OpticalModel<_> as Read<U>>::read(&mut om, Data::new(residual));
        <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(&mut om, m2_command.into());
        om.update();
        Ok(<OpticalModel<_> as Write<Wavefront>>::write(&mut om)
            .unwrap()
            .into_arc())
    }
}

impl<U> ClosedLoopCorrection<U> for DispersedFringeSensorProcessing
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel<WaveSensor>: Read<U>,
{
}
