use crate::{Result, SensorBuilder};

use super::optical_model::SensorFn;
use super::OpticalModel;
use crseo::{
    wavefrontsensor::{LensletArray, Model},
    Builder, GmtBuilder, ShackHartmannBuilder, SourceBuilder, WavefrontSensor,
    WavefrontSensorBuilder,
};
use gmt_dos_clients::interface::{Data, Write};
use gmt_dos_clients_io::gmt_m2::fsm::M2FSMTipTilt;
use nalgebra as na;

impl<M> SensorBuilder for ShackHartmannBuilder<M>
where
    M: 'static + Model,
{
    fn build(
        self,
        gmt_builder: GmtBuilder,
        src_builder: SourceBuilder,
        threshold: f64,
    ) -> Result<Box<dyn WavefrontSensor>> {
        let mut src = self.guide_stars(Some(src_builder)).build()?;
        let LensletArray { n_side_lenslet, .. } = self.lenslet_array;
        let n = n_side_lenslet.pow(2) * self.n_sensor;
        let mut valid_lenslets: Vec<i32> = (1..=7).fold(vec![0i32; n as usize], |mut a, sid| {
            let mut gmt = gmt_builder.clone().build().unwrap();
            src.reset();
            src.through(gmt.keep(&mut [sid])).xpupil();
            let mut sensor = Builder::build(self.clone()).unwrap();
            sensor.calibrate(&mut src, threshold);
            let valid_lenslets: Vec<f32> = sensor.lenslet_mask().into();
            /*valid_lenslets.chunks(48).for_each(|row| {
                row.iter().for_each(|val| print!("{val:.2},"));
                println!("");
            });
            println!("");*/
            a.iter_mut()
                .zip(&valid_lenslets)
                .filter(|(_, v)| **v > 0.)
                .for_each(|(a, _)| {
                    *a += 1;
                });
            a
        });
        /*
        valid_lenslets.chunks(48).for_each(|row| {
            row.iter().for_each(|val| print!("{val}"));
            println!("");
        });*/
        valid_lenslets
            .iter_mut()
            .filter(|v| **v > 1)
            .for_each(|v| *v = 0);
        //dbg!(valid_lenslets.iter().cloned().sum::<i32>());
        let mut sensor = Builder::build(self).unwrap();
        let mut gmt = gmt_builder.build()?;
        src.reset();
        src.through(&mut gmt).xpupil();
        sensor.set_valid_lenslet(&valid_lenslets);
        sensor.set_reference_slopes(&mut src);
        Ok(Box::new(sensor))
    }
}

impl Write<super::SensorData> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::SensorData>> {
        match (&mut self.sensor, &mut self.segment_wise_sensor) {
            (None, None) => None,
            (None, Some(sensor)) => match &self.sensor_fn {
                SensorFn::None => {
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    Some(Data::new(data))
                }
                SensorFn::Fn(f) => {
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    Some(Data::new(f(data)))
                }
                SensorFn::Matrix(mat) => {
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    let v = na::DVector::from_vec(data);
                    let y = mat * v;
                    Some(Data::new(y.as_slice().to_vec()))
                }
                SensorFn::Calibration(pinv) => (sensor.left_multiply(pinv))
                    .map(|x| {
                        (*sensor).reset();
                        x.iter().map(|x| *x as f64).collect()
                    })
                    .map(|y| Data::new(y)),
            },
            (Some(sensor), None) => match &self.sensor_fn {
                SensorFn::None => {
                    (*sensor).readout();
                    self.frame = (*sensor).frame();
                    (*sensor).process();
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    Some(Data::new(data))
                }
                SensorFn::Fn(f) => {
                    (*sensor).readout();
                    self.frame = (*sensor).frame();
                    (*sensor).process();
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    Some(Data::new(f(data)))
                }
                SensorFn::Matrix(mat) => {
                    (*sensor).readout();
                    self.frame = (*sensor).frame();
                    (*sensor).process();
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    let v = na::DVector::from_vec(data);
                    let y = mat * v;
                    Some(Data::new(y.as_slice().to_vec()))
                }
                _ => unimplemented!(),
            },
            (Some(_), Some(_)) => panic!("expected a single sensor type, found both"),
        }
    }
}
impl Write<M2FSMTipTilt> for OpticalModel {
    fn write(&mut self) -> Option<Data<M2FSMTipTilt>> {
        if let Some(sensor) = &mut self.sensor {
            match &self.sensor_fn {
                SensorFn::None => {
                    (*sensor).readout();
                    self.frame = (*sensor).frame();
                    (*sensor).process();
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    Some(Data::new(data))
                }
                SensorFn::Fn(f) => {
                    (*sensor).readout();
                    self.frame = (*sensor).frame();
                    (*sensor).process();
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    Some(Data::new(f(data)))
                }
                SensorFn::Matrix(mat) => {
                    (*sensor).readout();
                    self.frame = (*sensor).frame();
                    (*sensor).process();
                    let data: Vec<f64> = (*sensor).data();
                    (*sensor).reset();
                    let v = na::DVector::from_vec(data);
                    let y = mat * v;
                    Some(Data::new(y.as_slice().to_vec()))
                }
                SensorFn::Calibration(pinv) => {
                    let data = sensor.left_multiply(pinv);
                    (*sensor).reset();
                    data.map(|x| x.into_iter().map(|x| x as f64).collect())
                        .map(|y| Data::new(y))
                }
            }
        } else {
            None
        }
    }
}
impl Write<super::DetectorFrame> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::DetectorFrame>> {
        if let Some(sensor) = &mut self.sensor {
            if self.frame.is_none() {
                self.frame = sensor.frame();
            }
            if let Some(frame) = &self.frame.take() {
                Some(Data::new(frame.to_vec()))
            } else {
                None
            }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use std::ops::DerefMut;

    use crate::clients::ceo::{OpticalModel, OpticalModelOptions, ShackHartmannOptions};
    use crseo::{Builder, FromBuilder};
    use skyangle::Conversion;
    #[test]
    fn sensor_calibration() {
        let mut optical_model = OpticalModel::builder()
            .source(
                crseo::Source::builder()
                    .zenith_azimuth(vec![6f32.from_arcmin()], vec![30f32.to_radians()]),
            )
            .options(vec![OpticalModelOptions::ShackHartmann {
                options: ShackHartmannOptions::Geometric(*crseo::SH24::<crseo::Geometric>::new()),
                flux_threshold: 0.8,
            }])
            .build()
            .unwrap();
        optical_model.src.through(&mut optical_model.gmt).xpupil();
        println!("WFE RMS: {:.0?}nm", optical_model.src.wfe_rms_10e(-9));
        if let Some(sensor) = &mut optical_model.sensor {
            sensor.deref_mut().propagate(&mut optical_model.src);
            sensor.process();
            let data: Vec<f64> = (*sensor).data();
            let h = data.len() / 2;
            let (cx, cy) = data.split_at(h);
            let data_norm = (cx.iter().zip(cy).map(|(x, y)| x * x + y * y).sum::<f64>() / h as f64)
                .sqrt()
                .to_mas();
            println!("WFS data norm: {data_norm}mas");
            assert!(data_norm < 1e-1);
        }
    }
}
