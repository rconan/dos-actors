mod calibration;
mod centroiding;
mod optical_model;

pub use calibration::{Calibrate, CalibrateSegment, CalibrationMode, Reconstructor};
pub use centroiding::Centroids;
use crseo::{imaging::ImagingBuilder, Propagation, Source};
use interface::TimerMarker;
pub use optical_model::{
    builder::OpticalModelBuilder,
    dispersed_fringe_sensor::{
        DispersedFringeSensor, DispersedFringeSensorBuidler, DispersedFringeSensorProcessing,
    },
    no_sensor::NoSensor,
    wave_sensor::{WaveSensor, WaveSensorBuilder},
    OpticalModel,
};

impl<T> TimerMarker for OpticalModel<T> {}

pub trait SensorBuilderProperty {
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

pub trait SensorPropagation {
    fn propagate(&mut self, src: &mut Source);
}

impl<T: Propagation> SensorPropagation for T {
    fn propagate(&mut self, src: &mut Source) {
        self.propagate(src);
    }
}

// impl SensorBuilderProperty for SegmentPistonSensorBuilder {}

// impl SensorProperty for Imaging {
//     fn reset(&mut self) {
//         Imaging::reset(self);
//     }
// }
