mod calibration;
mod centroiding;
mod optical_model;

pub use calibration::{Calibrate, CalibrateSegment, CalibrationMode, Reconstructor};
pub use centroiding::Centroids;
use crseo::imaging::ImagingBuilder;
use crseo::{Imaging, Propagation};
use interface::TimerMarker;
pub use optical_model::builder::OpticalModelBuilder;
pub use optical_model::{
    no_sensor::NoSensor,
    wavefront::{Wave, WavefrontBuilder, WavefrontSensor},
    OpticalModel,
};

impl<T> TimerMarker for OpticalModel<T> {}

pub trait SensorBuilderProperty {
    fn pupil_sampling(&self) -> usize;
}

pub trait SensorProperty: Propagation {
    fn reset(&mut self);
}
impl SensorBuilderProperty for ImagingBuilder {
    fn pupil_sampling(&self) -> usize {
        self.lenslet_array.n_side_lenslet * self.lenslet_array.n_px_lenslet * self.n_sensor as usize
            + 1
    }
}

impl SensorProperty for Imaging {
    fn reset(&mut self) {
        Imaging::reset(self);
    }
}
