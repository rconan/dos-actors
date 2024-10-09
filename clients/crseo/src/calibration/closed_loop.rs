use std::{sync::Arc, thread};

use crseo::FromBuilder;
use faer::ColRef;

use crate::{OpticalModel, OpticalModelBuilder};

use super::{Calib, CalibrationMode, ClosedLoopCalib, MirrorMode, PushPull, Reconstructor, Result};

mod dispersed_fringe_sensor;
mod linear_model;

/// Trait alias for M1 [ClosedLoopCalibrateSegment]s with M2
pub trait ClosedLoopCalibrateAssembly<W: FromBuilder, S: FromBuilder>:
    ClosedLoopCalibrateSegment<W, 1, Sensor = S>
    + ClosedLoopCalibrateSegment<W, 2, Sensor = S>
    + ClosedLoopCalibrateSegment<W, 3, Sensor = S>
    + ClosedLoopCalibrateSegment<W, 4, Sensor = S>
    + ClosedLoopCalibrateSegment<W, 5, Sensor = S>
    + ClosedLoopCalibrateSegment<W, 6, Sensor = S>
    + ClosedLoopCalibrateSegment<W, 7, Sensor = S>
{
}
impl<
        W: FromBuilder,
        S: FromBuilder,
        T: ClosedLoopCalibrateSegment<W, 1, Sensor = S>
            + ClosedLoopCalibrateSegment<W, 2, Sensor = S>
            + ClosedLoopCalibrateSegment<W, 3, Sensor = S>
            + ClosedLoopCalibrateSegment<W, 4, Sensor = S>
            + ClosedLoopCalibrateSegment<W, 5, Sensor = S>
            + ClosedLoopCalibrateSegment<W, 6, Sensor = S>
            + ClosedLoopCalibrateSegment<W, 7, Sensor = S>,
    > ClosedLoopCalibrateAssembly<W, S> for T
{
}

type SegmentSensorBuilder<T, W, const SID: u8> =
    <<T as ClosedLoopCalibrateSegment<W, SID>>::Sensor as FromBuilder>::ComponentBuilder;
type SegmentClosedLoopSensorBuilder<T> = <T as FromBuilder>::ComponentBuilder;
// type Sensor<T, W, const SID: u8> = <T as ClosedLoopCalibrateSegment<W, SID>>::Sensor;

/// Actuator push and pull
pub trait ClosedLoopPushPull<const SID: u8> {
    type Sensor;
    fn push_pull(
        &mut self,
        optical_model: &mut OpticalModel<<Self as ClosedLoopPushPull<SID>>::Sensor>,
        s: f64,
        cmd: &[f64],
        calib_mode: &CalibrationMode,
        c: ColRef<'_, f64>,
    ) -> Arc<Vec<f64>>;
}

/// Closed-loop segment calibration
pub trait ClosedLoopCalibrateSegment<ClosedLoopSensor: FromBuilder, const SID: u8>
where
    Self:
        PushPull<SID, Sensor = <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, SID>>::Sensor>,
{
    type Sensor: FromBuilder;

    fn calibrate(
        optical_model: OpticalModelBuilder<SegmentSensorBuilder<Self, ClosedLoopSensor, SID>>,
        calib_mode: CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<
            <ClosedLoopSensor as FromBuilder>::ComponentBuilder,
        >,
        closed_loop_calib_mode: CalibrationMode,
    ) -> Result<ClosedLoopCalib>;
}

type SensorBuilder<T, W> = <<T as ClosedLoopCalibrate<W>>::Sensor as FromBuilder>::ComponentBuilder;
type ClosedLoopSensorBuilder<T> = <T as FromBuilder>::ComponentBuilder;

/// Closed-loop  calibration
pub trait ClosedLoopCalibrate<ClosedLoopSensor: FromBuilder>
where
    Self: ClosedLoopCalibrateAssembly<
        ClosedLoopSensor,
        <Self as ClosedLoopCalibrate<ClosedLoopSensor>>::Sensor,
    >,
{
    type Sensor: FromBuilder;

    fn calibrate(
        optical_model: &OpticalModelBuilder<SensorBuilder<Self, ClosedLoopSensor>>,
        mirror_mode: impl Into<MirrorMode>,
        closed_loop_optical_model: &OpticalModelBuilder<ClosedLoopSensorBuilder<ClosedLoopSensor>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> Result<Reconstructor<CalibrationMode, ClosedLoopCalib>>
    where
        <<Self as ClosedLoopCalibrate<ClosedLoopSensor>>::Sensor as FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
        <ClosedLoopSensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
    {
        let mut mode_iter = Into::<MirrorMode>::into(mirror_mode).into_iter();

        let mat_ci: Result<Vec<_>> = thread::scope(|s| {
            let h1 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 1>>::calibrate(
                        optical_model.clone(),
                        calib_mode,
                        closed_loop_optical_model.clone(),
                        closed_loop_calib_mode,
                    )
                })
            });
            let h2 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 2>>::calibrate(
                        optical_model.clone(),
                        calib_mode,
                        closed_loop_optical_model.clone(),
                        closed_loop_calib_mode,
                    )
                })
            });
            let h3 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 3>>::calibrate(
                        optical_model.clone(),
                        calib_mode,
                        closed_loop_optical_model.clone(),
                        closed_loop_calib_mode,
                    )
                })
            });
            let h4 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 4>>::calibrate(
                        optical_model.clone(),
                        calib_mode,
                        closed_loop_optical_model.clone(),
                        closed_loop_calib_mode,
                    )
                })
            });
            let h5 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 5>>::calibrate(
                        optical_model.clone(),
                        calib_mode,
                        closed_loop_optical_model.clone(),
                        closed_loop_calib_mode,
                    )
                })
            });
            let h6 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 6>>::calibrate(
                        optical_model.clone(),
                        calib_mode,
                        closed_loop_optical_model.clone(),
                        closed_loop_calib_mode,
                    )
                })
            });
            let h7 = mode_iter.next().unwrap().map(|calib_mode| {
                s.spawn(move || {
                    <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 7>>::calibrate(
                        optical_model.clone(),
                        calib_mode,
                        closed_loop_optical_model.clone(),
                        closed_loop_calib_mode,
                    )
                })
            }); // let mut ci = vec![];
                // for c in [c1, c2, c3, c4, c5, c6, c7] {
                //     ci.push(c.join().unwrap().unwrap());
                // }
                // ci
            [h1, h2, h3, h4, h5, h6, h7]
                .into_iter()
                .filter_map(|h| h.map(|h| h.join().unwrap()))
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
        Ok(Reconstructor::<CalibrationMode, ClosedLoopCalib>::new(
            mat_ci?,
        ))
        // mat_ci.map(|(mat, calib)| (mat, Reconstructor::new(calib)))
    }

    fn calibrate_serial(
        optical_model: &OpticalModelBuilder<SensorBuilder<Self, ClosedLoopSensor>>,
        mirror_mode: impl Into<MirrorMode>,
        closed_loop_optical_model: &OpticalModelBuilder<ClosedLoopSensorBuilder<ClosedLoopSensor>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> Result<Reconstructor<CalibrationMode, ClosedLoopCalib>>
    where
        <<Self as ClosedLoopCalibrate<ClosedLoopSensor>>::Sensor as FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
        <ClosedLoopSensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
    {
        let mut mode_iter = Into::<MirrorMode>::into(mirror_mode).into_iter();

        let h1 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 1>>::calibrate(
                optical_model.clone(),
                calib_mode,
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h2 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 2>>::calibrate(
                optical_model.clone(),
                calib_mode,
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h3 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 3>>::calibrate(
                optical_model.clone(),
                calib_mode,
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h4 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 4>>::calibrate(
                optical_model.clone(),
                calib_mode,
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h5 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 5>>::calibrate(
                optical_model.clone(),
                calib_mode,
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h6 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 6>>::calibrate(
                optical_model.clone(),
                calib_mode,
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h7 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 7>>::calibrate(
                optical_model.clone(),
                calib_mode,
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let mat_ci: Result<Vec<_>> = [h1, h2, h3, h4, h5, h6, h7]
            .into_iter()
            .filter_map(|h| h)
            .collect();

        Ok(Reconstructor::<CalibrationMode, ClosedLoopCalib>::new(
            mat_ci?,
        ))
        // mat_ci.map(|(mat, calib)| (mat, Reconstructor::new(calib)))
    }
}
