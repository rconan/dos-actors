mod calib;
mod calib_pinv;
mod centroids;
mod reconstructor;
mod wave_sensor;
mod dispersed_fringe_sensor;

use crate::ltao::centroiding::CentroidsError;
use crate::ltao::optical_model::OpticalModelError;
use crate::{OpticalModel, OpticalModelBuilder};
pub use calib::Calib;
pub use calib_pinv::CalibPinv;
use crseo::gmt::GmtMx;
pub use reconstructor::Reconstructor;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

#[derive(Debug, Clone, PartialEq, Copy, Serialize, Deserialize)]
pub enum CalibrationMode {
    RBM([Option<f64>; 6]),
    Modes {
        n_mode: usize,
        stroke: f64,
        start_idx: usize,
    },
}
impl CalibrationMode {
    pub fn modes(n_mode: usize, stroke: f64) -> Self {
        Self::Modes {
            n_mode,
            stroke,
            start_idx: 0,
        }
    }
    pub fn start_from(self, id: usize) -> Self {
        if let Self::Modes { n_mode, stroke, .. } = self {
            Self::Modes {
                n_mode,
                stroke,
                start_idx: id - 1,
            }
        } else {
            self
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum CalibrationError {
    #[error("failed to build optical model")]
    OpticalModel(#[from] OpticalModelError),
    #[error("failed to build centroids")]
    Centroids(#[from] CentroidsError),
}

pub type Result<T> = std::result::Result<T, CalibrationError>;

pub trait PushPull<const SID: u8> {
    type Sensor;
    fn push_pull<F>(
        &mut self,
        optical_model: &mut OpticalModel<Self::Sensor>,
        i: usize,
        s: f64,
        cmd: &mut [f64],
        cmd_fn: F,
    ) -> Vec<f64>
    where
        F: Fn(&mut crseo::Gmt, u8, &[f64]);
}

pub trait CalibrateSegment<M: GmtMx, const SID: u8> {
    type SegmentSensorBuilder;
    fn calibrate(
        optical_model: OpticalModelBuilder<Self::SegmentSensorBuilder>,
        calib_mode: CalibrationMode,
    ) -> Result<Calib>;
}

pub trait Calibrate<M: GmtMx>
where
    Self: CalibrateSegment<M, 1, SegmentSensorBuilder = Self::SensorBuilder>,
    Self: CalibrateSegment<M, 2, SegmentSensorBuilder = Self::SensorBuilder>,
    Self: CalibrateSegment<M, 3, SegmentSensorBuilder = Self::SensorBuilder>,
    Self: CalibrateSegment<M, 4, SegmentSensorBuilder = Self::SensorBuilder>,
    Self: CalibrateSegment<M, 5, SegmentSensorBuilder = Self::SensorBuilder>,
    Self: CalibrateSegment<M, 6, SegmentSensorBuilder = Self::SensorBuilder>,
    Self: CalibrateSegment<M, 7, SegmentSensorBuilder = Self::SensorBuilder>,
{
    type SensorBuilder;
    fn calibrate(
        optical_model: OpticalModelBuilder<Self::SensorBuilder>,
        calib_mode: CalibrationMode,
    ) -> Result<Reconstructor>;
}
