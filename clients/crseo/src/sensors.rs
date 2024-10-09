/*!
# Optical model sensors

A sensor is defined as an entity that implements the [SensorPropagation] trait.

Every entity that implements the [crseo::Propagation] trait is also as sensor.
*/

use crseo::{Propagation, Source};

pub mod builders;
mod camera;
mod dispersed_fringe_sensor;
mod no_sensor;
mod segment_piston;
mod wave_sensor;

pub use camera::Camera;
pub use dispersed_fringe_sensor::{DispersedFringeSensor, DispersedFringeSensorProcessing};
pub use no_sensor::NoSensor;
pub use segment_piston::SegmentPistonSensor;
pub use wave_sensor::WaveSensor;

/// Propagation definition for sensors
pub trait SensorPropagation {
    /// Propagates a [Source] through a sensor
    fn propagate(&mut self, src: &mut Source);
}

impl<T: Propagation> SensorPropagation for T {
    fn propagate(&mut self, src: &mut Source) {
        self.propagate(src);
    }
}
