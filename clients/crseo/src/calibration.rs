/*!
# Sensor calibration

The calibration module implements the calibration procedures for several sensors.

The calibration implementation is given by the [Calibrate] trait for the data processing
corresponding to a particular sensor.

The calibration is performed segment wise leading to 7 calibration matrices, each one saved in [Calib]
and the 7 [Calib]s are saved in [Reconstructor].

# Examples

Calibration of the 6 rigid body motions of all M1 segments with the [WaveSensor](crate::sensors::WaveSensor)

```
use gmt_dos_clients_crseo::{OpticalModel,
    sensors::WaveSensor, calibration::{Calibrate, CalibrationMode}};
use crseo::{gmt::GmtM1, Source, FromBuilder};

let omb = OpticalModel::<WaveSensor>::builder()
    .source(Source::builder().pupil_sampling(256));
let calib = <WaveSensor as Calibrate<GmtM1>>::calibrate(&omb,
    CalibrationMode::RBM([Some(1e-6);6]));

# Ok::<(),Box<dyn std::error::Error>>(())
```

*/

use crate::{
    centroiding::CentroidsError, optical_model::OpticalModelError, OpticalModel,
    OpticalModelBuilder,
};
use crseo::{gmt::GmtMx, CrseoError, FromBuilder};
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

pub use calib::{Calib, CalibBuilder};
pub use calib_pinv::CalibPinv;
pub use reconstructor::Reconstructor;

/// Selection of calibration modes per segment
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CalibrationMode {
    /// Rigid body motions as amplitudes of motion
    RBM([Option<f64>; 6]),
    /// Mirror shapes
    Modes {
        /// total number of modes
        n_mode: usize,
        /// mode amplitude
        stroke: f64,
        /// index of the 1st mode to calibrate
        start_idx: usize,
        /// last mode to calibrate
        end_id: Option<usize>,
    },
}
impl Default for CalibrationMode {
    fn default() -> Self {
        Self::RBM([None; 6])
    }
}
impl CalibrationMode {
    /// Sets the number of modes and the mode amplitude
    pub fn modes(n_mode: usize, stroke: f64) -> Self {
        Self::Modes {
            n_mode,
            stroke,
            start_idx: 0,
            end_id: None,
        }
    }
    /// Starts the calibration from the given mode
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
    /// Ends the calibration at the given mode
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
    /// Returns the indices as the range of modes to calibrate
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
    /// Returns the mode number as the range of modes to calibrate
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
    Crseo(#[from] CrseoError),
}

type Result<T> = std::result::Result<T, CalibrationError>;

/// Actuator push and pull
pub trait PushPull<const SID: u8> {
    type PushPullSensor;
    fn push_pull<F>(
        &mut self,
        optical_model: &mut OpticalModel<Self::PushPullSensor>,
        i: usize,
        s: f64,
        cmd: &mut [f64],
        cmd_fn: F,
    ) -> Vec<f64>
    where
        F: Fn(&mut crseo::Gmt, u8, &[f64]);
}

type SensorBuilder<M, T> = <<T as Calibrate<M>>::Sensor as FromBuilder>::ComponentBuilder;
type SegmentSensorBuilder<M, T, const SID: u8> =
    <<T as CalibrateSegment<M, SID>>::SegmentSensor as FromBuilder>::ComponentBuilder;

/// Segment calibration
pub trait CalibrateSegment<M: GmtMx, const SID: u8>
where
    Self: PushPull<SID, PushPullSensor = Self::SegmentSensor>,
{
    type SegmentSensor: FromBuilder;
    fn calibrate(
        optical_model: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: CalibrationMode,
    ) -> Result<Calib>;
}

/// Mirror calibration
pub trait Calibrate<M: GmtMx>
where
    <<Self as Calibrate<M>>::Sensor as FromBuilder>::ComponentBuilder: Clone + Send + 'static,
    Self: CalibrateSegment<M, 1, SegmentSensor = Self::Sensor>,
    Self: CalibrateSegment<M, 2, SegmentSensor = Self::Sensor>,
    Self: CalibrateSegment<M, 3, SegmentSensor = Self::Sensor>,
    Self: CalibrateSegment<M, 4, SegmentSensor = Self::Sensor>,
    Self: CalibrateSegment<M, 5, SegmentSensor = Self::Sensor>,
    Self: CalibrateSegment<M, 6, SegmentSensor = Self::Sensor>,
    Self: CalibrateSegment<M, 7, SegmentSensor = Self::Sensor>,
{
    type Sensor: FromBuilder;
    fn calibrate(
        optical_model: &OpticalModelBuilder<SensorBuilder<M, Self>>,
        calib_mode: CalibrationMode,
    ) -> Result<Reconstructor> {
        let om = optical_model.clone();
        let cm = calib_mode.clone();
        let c1 = std::thread::spawn(move || <Self as CalibrateSegment<M, 1>>::calibrate(om, cm));
        let om = optical_model.clone();
        let cm = calib_mode.clone();
        let c2 = std::thread::spawn(move || <Self as CalibrateSegment<M, 2>>::calibrate(om, cm));
        let om = optical_model.clone();
        let cm = calib_mode.clone();
        let c3 = std::thread::spawn(move || <Self as CalibrateSegment<M, 3>>::calibrate(om, cm));
        let om = optical_model.clone();
        let cm = calib_mode.clone();
        let c4 = std::thread::spawn(move || <Self as CalibrateSegment<M, 4>>::calibrate(om, cm));
        let om = optical_model.clone();
        let cm = calib_mode.clone();
        let c5 = std::thread::spawn(move || <Self as CalibrateSegment<M, 5>>::calibrate(om, cm));
        let om = optical_model.clone();
        let cm = calib_mode.clone();
        let c6 = std::thread::spawn(move || <Self as CalibrateSegment<M, 6>>::calibrate(om, cm));
        let om = optical_model.clone();
        let cm = calib_mode.clone();
        let c7 = std::thread::spawn(move || <Self as CalibrateSegment<M, 7>>::calibrate(om, cm));
        let mut ci = vec![];
        for c in [c1, c2, c3, c4, c5, c6, c7] {
            ci.push(c.join().unwrap().unwrap());
        }
        // let c1 = <Self as CalibrateSegment<M, 1>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c2 = <Self as CalibrateSegment<M, 2>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c3 = <Self as CalibrateSegment<M, 3>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c4 = <Self as CalibrateSegment<M, 4>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c5 = <Self as CalibrateSegment<M, 5>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c6 = <Self as CalibrateSegment<M, 6>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c7 = <Self as CalibrateSegment<M, 7>>::calibrate(optical_model.clone(), calib_mode)?;
        // let ci = vec![c1, c2, c3, c4, c5, c6, c7];
        Ok(Reconstructor::new(ci))
    }
}

#[cfg(test)]
mod tests {
    use crate::sensors::{Camera, WaveSensor};

    use super::*;
    use crseo::gmt::GmtM1;

    #[test]
    fn wave_sensor() {
        let omb = OpticalModel::<WaveSensor>::builder();
        println!("{:#?}", omb);
        // let calib = <WaveSensor as Calibrate<GmtM1>>::calibrate(&omb, CalibrationMode::RBM([Some(1e-6);6]));
    }
}
