use std::thread;

use crseo::FromBuilder;
use faer::Mat;

use crate::OpticalModelBuilder;

use super::{Calib, CalibrationMode, PushPull, Reconstructor, Result};

mod dispersed_fringer_sensor;

type SegmentSensorBuilder<T, const SID: u8> =
    <<T as ClosedLoopCalibrateSegment<SID>>::Sensor as FromBuilder>::ComponentBuilder;
type SegmentClosedLoopSensorBuilder<T, const SID: u8> =
    <<T as ClosedLoopCalibrateSegment<SID>>::ClosedLoopSensor as FromBuilder>::ComponentBuilder;
type Sensor<T, const SID: u8> = <T as ClosedLoopCalibrateSegment<SID>>::Sensor;

/// Closed-loop segment calibration
pub trait ClosedLoopCalibrateSegment<const SID: u8>
where
    Self: PushPull<SID, Sensor = <Self as ClosedLoopCalibrateSegment<SID>>::Sensor>,
{
    type Sensor: FromBuilder;
    type ClosedLoopSensor: FromBuilder;

    fn calibrate(
        optical_model: OpticalModelBuilder<SegmentSensorBuilder<Self, SID>>,
        calib_mode: CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<SegmentClosedLoopSensorBuilder<Self, SID>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> Result<(Mat<f64>, Calib)>;
}

type SensorBuilder<T> = <<T as ClosedLoopCalibrate>::Sensor as FromBuilder>::ComponentBuilder;
type ClosedLoopSensorBuilder<T> =
    <<T as ClosedLoopCalibrate>::ClosedLoopSensor as FromBuilder>::ComponentBuilder;

/// Closed-loop  calibration
pub trait ClosedLoopCalibrate
where
    Self: ClosedLoopCalibrateSegment<
        1,
        Sensor = <Self as ClosedLoopCalibrate>::Sensor,
        ClosedLoopSensor = <Self as ClosedLoopCalibrate>::ClosedLoopSensor,
    >,
    Self: ClosedLoopCalibrateSegment<
        2,
        Sensor = <Self as ClosedLoopCalibrate>::Sensor,
        ClosedLoopSensor = <Self as ClosedLoopCalibrate>::ClosedLoopSensor,
    >,
    Self: ClosedLoopCalibrateSegment<
        3,
        Sensor = <Self as ClosedLoopCalibrate>::Sensor,
        ClosedLoopSensor = <Self as ClosedLoopCalibrate>::ClosedLoopSensor,
    >,
    Self: ClosedLoopCalibrateSegment<
        4,
        Sensor = <Self as ClosedLoopCalibrate>::Sensor,
        ClosedLoopSensor = <Self as ClosedLoopCalibrate>::ClosedLoopSensor,
    >,
    Self: ClosedLoopCalibrateSegment<
        5,
        Sensor = <Self as ClosedLoopCalibrate>::Sensor,
        ClosedLoopSensor = <Self as ClosedLoopCalibrate>::ClosedLoopSensor,
    >,
    Self: ClosedLoopCalibrateSegment<
        6,
        Sensor = <Self as ClosedLoopCalibrate>::Sensor,
        ClosedLoopSensor = <Self as ClosedLoopCalibrate>::ClosedLoopSensor,
    >,
    Self: ClosedLoopCalibrateSegment<
        7,
        Sensor = <Self as ClosedLoopCalibrate>::Sensor,
        ClosedLoopSensor = <Self as ClosedLoopCalibrate>::ClosedLoopSensor,
    >,
{
    type Sensor: FromBuilder;
    type ClosedLoopSensor: FromBuilder;

    fn calibrate(
        optical_model: OpticalModelBuilder<SensorBuilder<Self>>,
        calib_mode: CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<ClosedLoopSensorBuilder<Self>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> Result<(Vec<Mat<f64>>, Reconstructor)>
    where
        <<Self as ClosedLoopCalibrate>::Sensor as FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
        <<Self as ClosedLoopCalibrate>::ClosedLoopSensor as FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
    {
        let mat_ci: Result<(Vec<_>, Vec<_>)> = thread::scope(|s| {
            let h1 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<1>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h2 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<2>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h3 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<3>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h4 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<4>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h5 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<5>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h6 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<6>>::calibrate(
                    optical_model.clone(),
                    calib_mode.clone(),
                    closed_loop_optical_model.clone(),
                    closed_loop_calib_mode.clone(),
                )
            });
            let h7 = s.spawn(|| {
                <Self as ClosedLoopCalibrateSegment<7>>::calibrate(
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
        // Ok(Reconstructor::new(ci?))
        mat_ci.map(|(mat, calib)| (mat, Reconstructor::new(calib)))
    }
}
