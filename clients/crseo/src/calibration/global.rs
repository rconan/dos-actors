use crseo::{
    gmt::{GmtM1, GmtM2, GmtMx},
    imaging::ImagingBuilder,
    FromBuilder, Imaging,
};
use gmt_dos_clients_io::optics::{
    Dev, Frame, M1GlobalTipTilt, M2GlobalTipTilt, SensorData, Wavefront,
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

use crate::{
    centroiding::{CentroidKind, CentroidsProcessing},
    sensors::{builders::WaveSensorBuilder, WaveSensor},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};

use super::{Calib, CalibrationError, CalibrationMode, Reconstructor};

/// Global mode calibration
///
/// Calibration of modes that applies to an entire mirror
/// as opposed to [Calibration](super::Calibration) that calibrates segment modes
pub trait GlobalCalibration<M: GmtMx>
where
    <<Self as GlobalCalibration<M>>::Sensor as FromBuilder>::ComponentBuilder: Clone + Send + Sync,
{
    type Sensor: FromBuilder;
    fn calibrate(
        optical_model: &OpticalModelBuilder<
            <<Self as GlobalCalibration<M>>::Sensor as FromBuilder>::ComponentBuilder,
        >,
        mode: CalibrationMode,
    ) -> super::Result<Reconstructor>;
}
pub trait GmtGlobalTipTilt {
    type UID: UniqueIdentifier<DataType = Vec<f64>>;
}
impl GmtGlobalTipTilt for GmtM1 {
    type UID = M1GlobalTipTilt;
}
impl GmtGlobalTipTilt for GmtM2 {
    type UID = M2GlobalTipTilt;
}

impl<K, M> GlobalCalibration<M> for CentroidsProcessing<K>
where
    OpticalModel<Imaging>: Read<<M as GmtGlobalTipTilt>::UID>,
    K: CentroidKind,
    M: GmtMx + GmtGlobalTipTilt,
    CentroidsProcessing<K>: Write<SensorData>,
{
    type Sensor = Imaging;
    fn calibrate(
        builder: &OpticalModelBuilder<ImagingBuilder>,
        mode: CalibrationMode,
    ) -> super::Result<Reconstructor> {
        if let CalibrationMode::GlobalTipTilt(tt) = mode {
            let mut centroids =
                CentroidsProcessing::<K>::try_from(builder.sensor.as_ref().unwrap())?;

            builder.initialize(&mut centroids);

            let mut optical_model = builder.clone().build()?;

            let mut c = vec![];

            for mut cmd in [vec![tt, 0f64], vec![0f64, tt]] {
                <OpticalModel<_> as Read<<M as GmtGlobalTipTilt>::UID>>::read(
                    &mut optical_model,
                    Data::new(cmd.clone()),
                );
                optical_model.update();
                <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut optical_model)
                    .map(|data| <Self as Read<Frame<Dev>>>::read(&mut centroids, data));
                centroids.update();
                let push = <Self as Write<SensorData>>::write(&mut centroids)
                    .unwrap()
                    .into_arc();
                optical_model
                    .sensor_mut()
                    .as_mut()
                    .map(|sensor| sensor.reset());

                cmd.iter_mut().for_each(|x| *x *= -1f64);
                <OpticalModel<_> as Read<<M as GmtGlobalTipTilt>::UID>>::read(
                    &mut optical_model,
                    cmd.into(),
                );
                optical_model.update();
                <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut optical_model)
                    .map(|data| <Self as Read<Frame<Dev>>>::read(&mut centroids, data));
                centroids.update();
                let pull = <Self as Write<SensorData>>::write(&mut centroids)
                    .unwrap()
                    .into_arc();
                optical_model
                    .sensor_mut()
                    .as_mut()
                    .map(|sensor| sensor.reset());

                let diff: Vec<_> = push
                    .iter()
                    .zip(pull.iter())
                    .map(|(x, y)| 0.5 * (x - y) / tt)
                    .collect();
                c.extend(diff);
            }
            let calib = Calib {
                sid: 0,
                n_mode: 2,
                c,
                mask: vec![true; 2],
                mode,
                runtime: Default::default(),
                n_cols: Some(2),
            };
            Ok(Reconstructor::from(calib))
        } else {
            Err(CalibrationError::GlobalCalibration(mode))
        }
    }
}
impl<M> GlobalCalibration<M> for WaveSensor
where
    OpticalModel<WaveSensor>: Read<<M as GmtGlobalTipTilt>::UID>,
    M: GmtMx + GmtGlobalTipTilt,
    // CentroidsProcessing<K>: Write<SensorData>,
{
    type Sensor = WaveSensor;
    fn calibrate(
        builder: &OpticalModelBuilder<WaveSensorBuilder>,
        mode: CalibrationMode,
    ) -> super::Result<Reconstructor> {
        if let CalibrationMode::GlobalTipTilt(tt) = mode {
            // let mut centroids =
            // CentroidsProcessing::<K>::try_from(builder.sensor.as_ref().unwrap())?;

            // builder.initialize(&mut centroids);

            let mut optical_model = builder.clone().build()?;

            let mut mask = vec![
                true;
                (optical_model.src.size * optical_model.src.pupil_sampling.pow(2))
                    as usize
            ];
            let mut c = vec![];

            for mut cmd in [vec![tt, 0f64], vec![0f64, tt]] {
                <OpticalModel<_> as Read<<M as GmtGlobalTipTilt>::UID>>::read(
                    &mut optical_model,
                    Data::new(cmd.clone()),
                );
                optical_model.update();
                // <OpticalModel<_> as Write<Wavefrontc>::write(&mut optical_model)
                // .map(|data| <Self as Read<Frame<Dev>>>::read(&mut centroids, data));
                // centroids.update();
                let push = <OpticalModel<_> as Write<Wavefront>>::write(&mut optical_model)
                    .unwrap()
                    .into_arc();

                mask.iter_mut()
                    .zip(optical_model.src.amplitude().into_iter())
                    .for_each(|(m, a)| *m = *m && (a > 0f32));
                // optical_model
                //     .sensor_mut()
                //     .as_mut()
                //     .map(|sensor| sensor.reset());

                cmd.iter_mut().for_each(|x| *x *= -1f64);
                <OpticalModel<_> as Read<<M as GmtGlobalTipTilt>::UID>>::read(
                    &mut optical_model,
                    cmd.into(),
                );
                optical_model.update();
                // <OpticalModel<_> as Write<Frame<Dev>>>::write(&mut optical_model)
                // .map(|data| <Self as Read<Frame<Dev>>>::read(&mut centroids, data));
                // centroids.update();
                let pull = <OpticalModel<_> as Write<Wavefront>>::write(&mut optical_model)
                    .unwrap()
                    .into_arc();

                mask.iter_mut()
                    .zip(optical_model.src.amplitude().into_iter())
                    .for_each(|(m, a)| *m = *m && (a > 0f32));

                // optical_model
                //     .sensor_mut()
                //     .as_mut()
                //     .map(|sensor| sensor.reset());

                let diff: Vec<_> = push
                    .iter()
                    .zip(pull.iter())
                    .zip(&mask)
                    .filter_map(|((x, y), m)| m.then_some(0.5 * (x - y) / tt))
                    .collect();
                c.extend(diff);
            }
            let calib = Calib {
                sid: 0,
                n_mode: 2,
                c,
                mask,
                mode,
                runtime: Default::default(),
                n_cols: Some(2),
            };
            Ok(Reconstructor::from(calib))
        } else {
            Err(CalibrationError::GlobalCalibration(mode))
        }
    }
}
