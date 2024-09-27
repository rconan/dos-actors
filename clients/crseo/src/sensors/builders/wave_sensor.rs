use crseo::{gmt::GmtBuilder, source::SourceBuilder, Builder, CrseoError};

use crate::{
    sensors::{NoSensor, WaveSensor},
    OpticalModel, OpticalModelBuilder,
};

use super::SensorBuilderProperty;

/// [WaveSensor] builder
///
/// # Examples:
///
/// Build a [WaveSensor] with the default values for [OpticalModelBuilder] without sensor
///
/// ```
/// use gmt_dos_clients_crseo::sensors::WaveSensor;
/// use crseo::{Builder, FromBuilder};
///
/// let wave = WaveSensor::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Default, Clone)]
pub struct WaveSensorBuilder(pub(crate) OpticalModelBuilder<NoSensor>);

impl WaveSensorBuilder {
    pub fn gmt(mut self, gmt: GmtBuilder) -> Self {
        self.0 = self.0.gmt(gmt);
        self
    }
    pub fn source(mut self, source: SourceBuilder) -> Self {
        self.0 = self.0.source(source);
        self
    }
}

impl Builder for WaveSensorBuilder {
    type Component = WaveSensor;
    fn build(self) -> std::result::Result<Self::Component, CrseoError> {
        let Self(omb) = self;
        let om: OpticalModel<NoSensor> = omb.build().unwrap();
        Ok(om.into())
    }
}

impl SensorBuilderProperty for WaveSensorBuilder {
    fn pupil_sampling(&self) -> Option<usize> {
        Some(self.0.src.pupil_sampling.side())
    }
}
