use crate::{
    centroiding::CentroidsError, optical_model::OpticalModelError, CeoError, OpticalModel,
    OpticalModelBuilder,
};
use crseo::gmt::GmtMx;
use serde::{Deserialize, Serialize};
use std::{
    fmt::Debug,
    ops::{Range, RangeInclusive},
};

mod calib;
mod calib_pinv;
mod centroids;
mod dispersed_fringe_sensor;
mod reconstructor;
mod wave_sensor;

pub use calib::Calib;
pub use calib_pinv::CalibPinv;
pub use reconstructor::Reconstructor;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CalibrationMode {
    RBM([Option<f64>; 6]),
    Modes {
        n_mode: usize,
        stroke: f64,
        start_idx: usize,
        end_id: Option<usize>,
    },
}
impl Default for CalibrationMode {
    fn default() -> Self {
        Self::RBM([None; 6])
    }
}
impl CalibrationMode {
    pub fn modes(n_mode: usize, stroke: f64) -> Self {
        Self::Modes {
            n_mode,
            stroke,
            start_idx: 0,
            end_id: None,
        }
    }
    pub fn start_from(self, id: usize) -> Self {
        if let Self::Modes {
            n_mode,
            stroke,
            end_id,
            ..
        } = self
        {
            Self::Modes {
                n_mode,
                stroke,
                start_idx: id - 1,
                end_id,
            }
        } else {
            self
        }
    }
    pub fn ends_at(self, id: usize) -> Self {
        if let Self::Modes {
            n_mode,
            stroke,
            start_idx,
            ..
        } = self
        {
            Self::Modes {
                n_mode,
                stroke,
                start_idx,
                end_id: Some(id),
            }
        } else {
            self
        }
    }
    pub fn range(&self) -> Range<usize> {
        match self {
            Self::RBM(_) => 0..6,
            Self::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => {
                let end = end_id.unwrap_or(*n_mode);
                *start_idx..end
            }
        }
    }
    pub fn mode_range(&self) -> RangeInclusive<usize> {
        match self {
            Self::RBM(_) => 1..=6,
            Self::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => {
                let start = *start_idx + 1;
                let end = end_id.unwrap_or(*n_mode);
                start..=end
            }
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum CalibrationError {
    #[error("failed to build optical model")]
    OpticalModel(#[from] OpticalModelError),
    #[error("failed to build centroids")]
    Centroids(#[from] CentroidsError),
    #[error("failed to build optical model")]
    CEO(#[from] CeoError),
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
        optical_model: &OpticalModelBuilder<Self::SensorBuilder>,
        calib_mode: CalibrationMode,
    ) -> Result<Reconstructor>;
}
