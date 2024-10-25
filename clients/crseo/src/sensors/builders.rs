/*!
# Optical model sensor builders

A sensor builder is defined as an entity that implements the [SensorBuilderProperty] trait.
*/
use crseo::imaging::ImagingBuilder;

mod camera;
pub use camera::CameraBuilder;
mod disperse_fringe_sensor;
pub use disperse_fringe_sensor::DispersedFringeSensorBuilder;
mod wave_sensor;
pub use wave_sensor::WaveSensorBuilder;
mod segment_piston;
pub use segment_piston::SegmentPistonSensorBuilder;
mod segment_gradient;
pub use segment_gradient::SegmentGradientSensorBuilder;

/// Common properties for all sensor builders
pub trait SensorBuilderProperty {
    /// Returns the pupil samplign corresponding to the sensor
    fn pupil_sampling(&self) -> Option<usize> {
        None
    }
}

impl SensorBuilderProperty for ImagingBuilder {
    fn pupil_sampling(&self) -> Option<usize> {
        Some(self.lenslet_array.n_side_lenslet * self.lenslet_array.n_px_lenslet + 1)
    }
}
