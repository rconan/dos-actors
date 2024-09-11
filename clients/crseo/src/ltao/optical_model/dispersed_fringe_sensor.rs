use std::ops::{Deref, DerefMut};

use crseo::SegmentPistonSensor;
use gmt_dos_clients_io::optics::{dispersed_fringe_sensor::DfsFftFrame, Dev, Frame, Host};
use interface::{Data, Size, Write};

use crate::ltao::SensorPropagation;

use super::{OpticalModel, Result};

mod builder;
pub use builder::DispersedFringeSensorBuidler;

mod processing;
pub use processing::DispersedFringeSensorProcessing;

pub struct DispersedFringeSensor<const C: usize, const F: usize>(SegmentPistonSensor);
impl<const C: usize, const F: usize> Deref for DispersedFringeSensor<C, F> {
    type Target = SegmentPistonSensor;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const C: usize, const F: usize> DerefMut for DispersedFringeSensor<C, F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const C: usize, const F: usize> SensorPropagation for DispersedFringeSensor<C, F> {
    fn propagate(&mut self, src: &mut crseo::Source) {
        if self.n_camera_frame() == C {
            self.camera_reset();
        }
        if self.n_fft_frame() == F {
            self.fft_reset();
        }
        src.through(&mut self.0);
        if self.n_camera_frame() == C {
            self.fft();
        }
    }
}

impl<const C: usize, const F: usize> Write<Frame<Dev>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn write(&mut self) -> Option<Data<Frame<Dev>>> {
        self.sensor
            .as_mut()
            .map(|sensor| Data::new(sensor.frame().clone()))
    }
}

impl<const C: usize, const F: usize> Write<Frame<Host>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn write(&mut self) -> Option<Data<Frame<Host>>> {
        self.sensor
            .as_mut()
            .map(|sensor| { Vec::<f32>::from(&mut sensor.frame()) }.into())
    }
}

impl<const C: usize, const F: usize> Size<Frame<Host>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn len(&self) -> usize {
        self.sensor
            .as_ref()
            .map_or_else(|| 0, |dfs| dfs.frame_size().pow(2))
    }
}

impl<const C: usize, const F: usize> Write<DfsFftFrame<Dev>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn write(&mut self) -> Option<Data<DfsFftFrame<Dev>>> {
        self.sensor
            .as_mut()
            .map(|sensor| Data::new(sensor.fft_frame().clone()))
    }
}

impl<const C: usize, const F: usize> Write<DfsFftFrame<Host>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn write(&mut self) -> Option<Data<DfsFftFrame<Host>>> {
        self.sensor
            .as_mut()
            .map(|sensor| Data::new(Vec::<f32>::from(&mut sensor.fft_frame())))
    }
}

impl<const C: usize, const F: usize> Size<DfsFftFrame<Host>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn len(&self) -> usize {
        self.sensor
            .as_ref()
            .map_or_else(|| 0, |dfs| dfs.fft_size().pow(2))
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{FromBuilder, Source};
    use interface::Update;

    use crate::OpticalModel;

    use super::*;

    #[test]
    fn dfs() -> std::result::Result<(), Box<dyn Error>> {
        let mut om = OpticalModel::<DispersedFringeSensor<1, 1>>::builder()
            .source(Source::builder().size(2))
            .sensor(DispersedFringeSensor::<1, 1>::builder())
            .build()?;
        om.update();

        let frame: Vec<_> = om.sensor().unwrap().frame().into();

        serde_pickle::to_writer(
            &mut std::fs::File::create("dfs.pkl")?,
            &frame,
            Default::default(),
        )?;

        Ok(())
    }
}
