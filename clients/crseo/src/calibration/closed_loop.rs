use std::thread;

use crseo::FromBuilder;

use crate::OpticalModelBuilder;

use super::{Calib, CalibrationMode, PushPull, Reconstructor, Result};

mod calib;
mod dispersed_fringe_sensor;
mod linear_model;

pub use calib::ClosedLoopCalib;

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
type Sensor<T, W, const SID: u8> = <T as ClosedLoopCalibrateSegment<W, SID>>::Sensor;

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
        optical_model: OpticalModelBuilder<SensorBuilder<Self, ClosedLoopSensor>>,
        calib_mode: CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<ClosedLoopSensorBuilder<ClosedLoopSensor>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> Result<Reconstructor<ClosedLoopCalib>>
    where
        <<Self as ClosedLoopCalibrate<ClosedLoopSensor>>::Sensor as FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
        <ClosedLoopSensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
    {
        let mat_ci: Result<Vec<_>> = thread::scope(|s| {
            let h1 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 1>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h2 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 2>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h3 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 3>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h4 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 4>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h5 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 5>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h6 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 6>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h7 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<ClosedLoopSensor, 7>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            // let mut ci = vec![];
            // for c in [c1, c2, c3, c4, c5, c6, c7] {
            //     ci.push(c.join().unwrap().unwrap());
            // }
            // ci
            [h1, h2, h3, h4, h5, h6, h7]
                .into_iter()
                .map(|h| h.join().unwrap())
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
        Ok(Reconstructor::<ClosedLoopCalib>::new(mat_ci?))
        // mat_ci.map(|(mat, calib)| (mat, Reconstructor::new(calib)))
    }
}
