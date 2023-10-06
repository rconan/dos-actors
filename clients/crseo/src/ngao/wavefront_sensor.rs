use std::{
    ops::Mul,
    sync::{Arc, Mutex},
};

use crseo::{
    wavefrontsensor::{Calibration, DataRef, Slopes},
    Propagation, SegmentWiseSensor, Source,
};
use interface::{Data, Read, UniqueIdentifier, Update, Write, UID};

pub enum GuideStar {}
impl UniqueIdentifier for GuideStar {
    type DataType = Arc<Mutex<Source>>;
}

#[derive(UID)]
pub enum PistonMode {}

#[derive(UID)]
#[uid(data = Vec<f32>)]
pub enum SensorData {}

pub struct WavefrontSensor<T, const NO: usize = 1> {
    sensor: T,
    calib: Calibration,
    n: usize,
}

unsafe impl<T, const NO: usize> Send for WavefrontSensor<T, NO> {}
unsafe impl<T, const NO: usize> Sync for WavefrontSensor<T, NO> {}

impl<T: SegmentWiseSensor, const NO: usize> WavefrontSensor<T, NO> {
    pub fn new(sensor: T, calib: Calibration) -> Self {
        Self {
            sensor,
            calib,
            n: 0,
        }
    }
}

impl<T, const NO: usize> Update for WavefrontSensor<T, NO> {}

impl<T: SegmentWiseSensor, const NO: usize> Read<GuideStar> for WavefrontSensor<T, NO> {
    fn read(&mut self, data: Data<GuideStar>) {
        let src = &mut (*data.lock().unwrap());
        self.n += 1;
        self.sensor.propagate(src);
    }
}

impl<T, U, const NO: usize> Write<U> for WavefrontSensor<T, NO>
where
    for<'a> &'a Calibration: Mul<&'a T, Output = Option<Vec<f32>>>,
    T: SegmentWiseSensor,
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        // dbg!((std::any::type_name::<T>(), self.n, self.calib.nrows()));
        if self.n < NO {
            Some(vec![0f64; self.calib.nrows()].into())
        } else {
            (&self.calib * &self.sensor).map(|x| {
                self.sensor.reset();
                self.n = 0;
                Data::new(x.into_iter().map(|x| x as f64).collect())
            })
        }
    }
}

impl<T, const NO: usize> Write<SensorData> for WavefrontSensor<T, NO>
where
    for<'a> Slopes: From<(&'a DataRef, &'a T)>,
    T: SegmentWiseSensor,
{
    fn write(&mut self) -> Option<Data<SensorData>> {
        let data: Vec<f32> = self
            .calib
            .iter()
            .map(|s| Slopes::from((&s.data_ref, &self.sensor)))
            .flat_map(|s| Vec::<f32>::from(s))
            .collect();
        // self.sensor.reset();
        Some(Data::new(data))
    }
}

pub struct ShackHartmann(pub crseo::ShackHartmann<crseo::Diffractive>);

unsafe impl Send for ShackHartmann {}
unsafe impl Sync for ShackHartmann {}

impl Update for ShackHartmann {}

impl Read<GuideStar> for ShackHartmann {
    fn read(&mut self, data: Data<GuideStar>) {
        let src = &mut (*data.lock().unwrap());
        self.0.propagate(src);
    }
}

#[derive(UID)]
#[uid(data = Vec<f32>)]
pub enum Frame {}

impl Write<Frame> for ShackHartmann {
    fn write(&mut self) -> Option<Data<Frame>> {
        <crseo::ShackHartmann<crseo::Diffractive> as crseo::WavefrontSensor>::frame(&self.0).map(
            |frame| {
                <crseo::ShackHartmann<crseo::Diffractive> as crseo::WavefrontSensor>::reset(
                    &mut self.0,
                );
                frame.into()
            },
        )
    }
}
