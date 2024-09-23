/*!
# Optical model sensor builders

A sensor builder is defined as an entity that implements the [SensorBuilderProperty] trait.
*/
use crate::{OpticalModel, OpticalModelBuilder};
use crseo::{imaging::ImagingBuilder, Builder, CrseoError};

use super::{NoSensor, WaveSensor};

mod camera;
pub use camera::CameraBuilder;
mod disperse_fringe_sensor;
pub use disperse_fringe_sensor::DispersedFringeSensorBuidler;

/// Common properties for all sensor builders
pub trait SensorBuilderProperty {
    /// Returns the pupil samplign corresponding to the sensor
    fn pupil_sampling(&self) -> Option<usize> {
        None
    }
}

impl SensorBuilderProperty for ImagingBuilder {
    fn pupil_sampling(&self) -> Option<usize> {
        Some(
            self.lenslet_array.n_side_lenslet
                * self.lenslet_array.n_px_lenslet
                * self.n_sensor as usize
                + 1,
        )
    }
}

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
