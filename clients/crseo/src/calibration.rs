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
use std::{fmt::Debug, thread};

mod calib;
mod calib_pinv;
mod centroids;
mod closed_loop;
mod dispersed_fringe_sensor;
mod mode;
mod reconstructor;
mod wave_sensor;

pub use calib::{Calib, CalibBuilder};
pub use calib_pinv::CalibPinv;
pub use closed_loop::ClosedLoopCalibrate;
pub use mode::CalibrationMode;
pub use reconstructor::Reconstructor;

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
    type Sensor;
    fn push_pull<F>(
        &mut self,
        optical_model: &mut OpticalModel<<Self as PushPull<SID>>::Sensor>,
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
    <<T as CalibrateSegment<M, SID>>::Sensor as FromBuilder>::ComponentBuilder;

/// Segment calibration
pub trait CalibrateSegment<M: GmtMx, const SID: u8>
where
    Self: PushPull<SID, Sensor = <Self as CalibrateSegment<M, SID>>::Sensor>,
{
    type Sensor: FromBuilder;
    fn calibrate(
        optical_model: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: CalibrationMode,
    ) -> Result<Calib>;
}

/// Mirror calibration
pub trait Calibrate<M: GmtMx>
where
    <<Self as Calibrate<M>>::Sensor as FromBuilder>::ComponentBuilder: Clone + Send + 'static,
    Self: CalibrateSegment<M, 1, Sensor = <Self as Calibrate<M>>::Sensor>,
    Self: CalibrateSegment<M, 2, Sensor = <Self as Calibrate<M>>::Sensor>,
    Self: CalibrateSegment<M, 3, Sensor = <Self as Calibrate<M>>::Sensor>,
    Self: CalibrateSegment<M, 4, Sensor = <Self as Calibrate<M>>::Sensor>,
    Self: CalibrateSegment<M, 5, Sensor = <Self as Calibrate<M>>::Sensor>,
    Self: CalibrateSegment<M, 6, Sensor = <Self as Calibrate<M>>::Sensor>,
    Self: CalibrateSegment<M, 7, Sensor = <Self as Calibrate<M>>::Sensor>,
{
    type Sensor: FromBuilder;
    fn calibrate(
        optical_model: &OpticalModelBuilder<SensorBuilder<M, Self>>,
        calib_mode: CalibrationMode,
    ) -> Result<Reconstructor>
    where
        <<Self as Calibrate<M>>::Sensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
    {
        let ci: Result<Vec<_>> = thread::scope(|s| {
            let c1 = s.spawn(|| {
                <Self as CalibrateSegment<M, 1>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                )
            });
            let c2 = s.spawn(|| {
                <Self as CalibrateSegment<M, 2>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                )
            });
            let c3 = s.spawn(|| {
                <Self as CalibrateSegment<M, 3>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                )
            });
            let c4 = s.spawn(|| {
                <Self as CalibrateSegment<M, 4>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                )
            });
            let c5 = s.spawn(|| {
                <Self as CalibrateSegment<M, 5>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                )
            });
            let c6 = s.spawn(|| {
                <Self as CalibrateSegment<M, 6>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                )
            });
            let c7 = s.spawn(|| {
                <Self as CalibrateSegment<M, 7>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                )
            });
            // let mut ci = vec![];
            // for c in [c1, c2, c3, c4, c5, c6, c7] {
            //     ci.push(c.join().unwrap().unwrap());
            // }
            // ci
            [c1, c2, c3, c4, c5, c6, c7]
                .into_iter()
                .map(|c| c.join().unwrap())
                .collect()
        });
        // let c1 = <Self as CalibrateSegment<M, 1>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c2 = <Self as CalibrateSegment<M, 2>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c3 = <Self as CalibrateSegment<M, 3>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c4 = <Self as CalibrateSegment<M, 4>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c5 = <Self as CalibrateSegment<M, 5>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c6 = <Self as CalibrateSegment<M, 6>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c7 = <Self as CalibrateSegment<M, 7>>::calibrate(optical_model.clone(), calib_mode)?;
        // let ci = vec![c1, c2, c3, c4, c5, c6, c7];
        Ok(Reconstructor::new(ci?))
    }
}
