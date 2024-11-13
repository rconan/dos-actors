/*!
# Sensor calibration

The calibration module implements the calibration procedures for several sensors.

The calibration implementation is given by the [Calibration] trait for the data processing
corresponding to a particular sensor.

The calibration is performed segment wise leading to 7 calibration matrices, each one saved in [Calib]
and the 7 [Calib]s are saved in [Reconstructor].

# Examples

Calibration of the 6 rigid body motions of all M1 segments with the [WaveSensor](crate::sensors::WaveSensor)

```
use gmt_dos_clients_crseo::{OpticalModel,
    sensors::WaveSensor, calibration::{Calibration, CalibrationMode}};
use crseo::{gmt::GmtM1, Source, FromBuilder};

let omb = OpticalModel::<WaveSensor>::builder()
    .source(Source::builder().pupil_sampling(256));
let calib = <WaveSensor as Calibration<GmtM1>>::calibrate(&omb,
    CalibrationMode::RBM([Some(1e-6);6]));

# Ok::<(),Box<dyn std::error::Error>>(())
```

*/

use crate::{
    centroiding::CentroidsError, optical_model::OpticalModelError, OpticalModel,
    OpticalModelBuilder,
};
use algebra::CalibProps;
use crseo::{gmt::GmtMx, CrseoError, FromBuilder, Propagation};
use gmt_dos_clients_io::gmt_m1::segment::{BendingModes, RBM};
use interface::{Read, UniqueIdentifier, Update, Write};
use std::{fmt::Debug, sync::Arc, thread};

pub mod algebra;
mod closed_loop;
pub mod correction;
pub mod estimation;
pub mod mode;
mod processing;

pub use algebra::{Calib, ClosedLoopCalib, ClosedLoopReconstructor, Reconstructor};
pub use closed_loop::ClosedLoopCalibration;
pub use mode::{CalibrationMode, MirrorMode, MixedMirrorMode, Modality};

#[derive(Debug, thiserror::Error)]
pub enum CalibrationError {
    #[error("failed to build optical model")]
    OpticalModel(#[from] OpticalModelError),
    #[error("failed to build centroids")]
    Centroids(#[from] CentroidsError),
    #[error("failed to build optical model")]
    Crseo(#[from] CrseoError),
    #[error("global calibration of {0} failed")]
    GlobalCalibration(CalibrationMode),
}

type Result<T> = std::result::Result<T, CalibrationError>;

/// Trait alias for M1 or M2 [CalibrationSegment]s
pub trait CalibrateAssembly<M: GmtMx, S: FromBuilder>:
    CalibrationSegment<M, 1, Sensor = S>
    + CalibrationSegment<M, 2, Sensor = S>
    + CalibrationSegment<M, 3, Sensor = S>
    + CalibrationSegment<M, 4, Sensor = S>
    + CalibrationSegment<M, 5, Sensor = S>
    + CalibrationSegment<M, 6, Sensor = S>
    + CalibrationSegment<M, 7, Sensor = S>
{
}

impl<
        M: GmtMx,
        S: FromBuilder,
        T: CalibrationSegment<M, 1, Sensor = S>
            + CalibrationSegment<M, 2, Sensor = S>
            + CalibrationSegment<M, 3, Sensor = S>
            + CalibrationSegment<M, 4, Sensor = S>
            + CalibrationSegment<M, 5, Sensor = S>
            + CalibrationSegment<M, 6, Sensor = S>
            + CalibrationSegment<M, 7, Sensor = S>,
    > CalibrateAssembly<M, S> for T
{
}

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

pub trait PushPullAlt<const SID: u8>
where
    <Self as PushPullAlt<SID>>::Sensor: Propagation,
    OpticalModel<Self::Sensor>: Write<Self::Input>,
    Self: Read<Self::Input> + Write<Self::Output>,
{
    type Sensor;
    type Input: UniqueIdentifier;
    type Output: UniqueIdentifier;
    fn push_pull(
        &mut self,
        optical_model: &mut OpticalModel<<Self as PushPullAlt<SID>>::Sensor>,
        cmd: &[f64],
        calib_mode: &CalibrationMode,
    ) -> Arc<<Self::Output as UniqueIdentifier>::DataType> {
        match calib_mode {
            CalibrationMode::RBM(_) => {
                <OpticalModel<Self::Sensor> as Read<RBM<SID>>>::read(
                    optical_model,
                    cmd.to_vec().into(),
                );
            }
            CalibrationMode::Modes { .. } => {
                <OpticalModel<Self::Sensor> as Read<BendingModes<SID>>>::read(
                    optical_model,
                    cmd.to_vec().into(),
                );
            }
            _ => unimplemented!(),
        }
        optical_model.update();
        <OpticalModel<Self::Sensor> as Write<Self::Input>>::write(optical_model)
            .map(|data| <Self as Read<Self::Input>>::read(self, data));
        optical_model.update();
        <Self as Write<Self::Output>>::write(self)
            .unwrap()
            .into_arc()
    }
}

type SensorBuilder<M, T> = <<T as Calibration<M>>::Sensor as FromBuilder>::ComponentBuilder;
type SegmentSensorBuilder<M, T, const SID: u8> =
    <<T as CalibrationSegment<M, SID>>::Sensor as FromBuilder>::ComponentBuilder;

/// Segment calibration
pub trait CalibrationSegment<M: GmtMx, const SID: u8>
where
    Self: PushPull<SID, Sensor = <Self as CalibrationSegment<M, SID>>::Sensor>,
{
    type Sensor: FromBuilder;
    fn calibrate(
        optical_model: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: CalibrationMode,
    ) -> Result<Calib>;
}

/// Mirror calibration
pub trait Calibration<M: GmtMx>
where
    <<Self as Calibration<M>>::Sensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
    Self: CalibrateAssembly<M, <Self as Calibration<M>>::Sensor>,
{
    type Sensor: FromBuilder;
    fn calibrate(
        optical_model: &OpticalModelBuilder<SensorBuilder<M, Self>>,
        mirror_mode: impl Into<MirrorMode>,
    ) -> Result<Reconstructor>
// where
        // <<Self as Calibration<M>>::Sensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
    {
        let mut mode_iter = Into::<MirrorMode>::into(mirror_mode).into_iter();
        let ci: Result<Vec<_>> = thread::scope(|s| {
            let c1 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    if calib_mode.is_empty() {
                        Ok(Calib::empty(1, calib_mode.n_mode(), calib_mode))
                    } else {
                        <Self as CalibrationSegment<M, 1>>::calibrate(
                            optical_model.clone(),
                            calib_mode,
                        )
                    }
                })
            });
            let c2 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    if calib_mode.is_empty() {
                        Ok(Calib::empty(2, calib_mode.n_mode(), calib_mode))
                    } else {
                        <Self as CalibrationSegment<M, 2>>::calibrate(
                            optical_model.clone(),
                            calib_mode,
                        )
                    }
                })
            });
            let c3 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    if calib_mode.is_empty() {
                        Ok(Calib::empty(3, calib_mode.n_mode(), calib_mode))
                    } else {
                        <Self as CalibrationSegment<M, 3>>::calibrate(
                            optical_model.clone(),
                            calib_mode,
                        )
                    }
                })
            });
            let c4 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    if calib_mode.is_empty() {
                        Ok(Calib::empty(4, calib_mode.n_mode(), calib_mode))
                    } else {
                        <Self as CalibrationSegment<M, 4>>::calibrate(
                            optical_model.clone(),
                            calib_mode,
                        )
                    }
                })
            });
            let c5 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    if calib_mode.is_empty() {
                        Ok(Calib::empty(5, calib_mode.n_mode(), calib_mode))
                    } else {
                        <Self as CalibrationSegment<M, 5>>::calibrate(
                            optical_model.clone(),
                            calib_mode,
                        )
                    }
                })
            });
            let c6 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    if calib_mode.is_empty() {
                        Ok(Calib::empty(6, calib_mode.n_mode(), calib_mode))
                    } else {
                        <Self as CalibrationSegment<M, 6>>::calibrate(
                            optical_model.clone(),
                            calib_mode,
                        )
                    }
                })
            });
            let c7 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    if calib_mode.is_empty() {
                        Ok(Calib::empty(7, calib_mode.n_mode(), calib_mode))
                    } else {
                        <Self as CalibrationSegment<M, 7>>::calibrate(
                            optical_model.clone(),
                            calib_mode,
                        )
                    }
                })
            }); // let mut ci = vec![];
                // for c in [c1, c2, c3, c4, c5, c6, c7] {
                //     ci.push(c.join().unwrap().unwrap());
                // }
                // ci
            [c1, c2, c3, c4, c5, c6, c7]
                .into_iter()
                .filter_map(|c| c.map(|c| c.join().unwrap()))
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

    fn calibrate_serial(
        optical_model: &OpticalModelBuilder<SensorBuilder<M, Self>>,
        mirror_mode: impl Into<MirrorMode>,
    ) -> Result<Reconstructor>
    where
        <<Self as Calibration<M>>::Sensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
    {
        let mut mode_iter = Into::<MirrorMode>::into(mirror_mode).into_iter();
        let c1 = mode_iter.next().unwrap().map(|calib_mode| {
            if calib_mode.is_empty() {
                Ok(Calib::empty(1, calib_mode.n_mode(), calib_mode))
            } else {
                <Self as CalibrationSegment<M, 1>>::calibrate(optical_model.clone(), calib_mode)
            }
        });
        let c2 = mode_iter.next().unwrap().map(|calib_mode| {
            if calib_mode.is_empty() {
                Ok(Calib::empty(2, calib_mode.n_mode(), calib_mode))
            } else {
                <Self as CalibrationSegment<M, 2>>::calibrate(optical_model.clone(), calib_mode)
            }
        });
        let c3 = mode_iter.next().unwrap().map(|calib_mode| {
            if calib_mode.is_empty() {
                Ok(Calib::empty(3, calib_mode.n_mode(), calib_mode))
            } else {
                <Self as CalibrationSegment<M, 3>>::calibrate(optical_model.clone(), calib_mode)
            }
        });
        let c4 = mode_iter.next().unwrap().map(|calib_mode| {
            if calib_mode.is_empty() {
                Ok(Calib::empty(4, calib_mode.n_mode(), calib_mode))
            } else {
                <Self as CalibrationSegment<M, 4>>::calibrate(optical_model.clone(), calib_mode)
            }
        });
        let c5 = mode_iter.next().unwrap().map(|calib_mode| {
            if calib_mode.is_empty() {
                Ok(Calib::empty(5, calib_mode.n_mode(), calib_mode))
            } else {
                <Self as CalibrationSegment<M, 5>>::calibrate(optical_model.clone(), calib_mode)
            }
        });
        let c6 = mode_iter.next().unwrap().map(|calib_mode| {
            if calib_mode.is_empty() {
                Ok(Calib::empty(6, calib_mode.n_mode(), calib_mode))
            } else {
                <Self as CalibrationSegment<M, 6>>::calibrate(optical_model.clone(), calib_mode)
            }
        });
        let c7 = mode_iter.next().unwrap().map(|calib_mode| {
            if calib_mode.is_empty() {
                Ok(Calib::empty(7, calib_mode.n_mode(), calib_mode))
            } else {
                <Self as CalibrationSegment<M, 7>>::calibrate(optical_model.clone(), calib_mode)
            }
        }); // let mut ci = vec![];
            // for c in [c1, c2, c3, c4, c5, c6, c7] {
            //     ci.push(c.join().unwrap().unwrap());
            // }
            // ci
        let ci: Result<Vec<_>> = [c1, c2, c3, c4, c5, c6, c7]
            .into_iter()
            .filter_map(|c| c.map(|c| c))
            .collect();
        Ok(Reconstructor::new(ci?))
    }
}

mod global;
pub use global::GlobalCalibration;
