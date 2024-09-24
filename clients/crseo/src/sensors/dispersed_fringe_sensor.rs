use crate::OpticalModel;
use crseo::{FromBuilder, SegmentPistonSensor};
use gmt_dos_clients_io::optics::{dispersed_fringe_sensor::DfsFftFrame, Dev, Frame, Host};
use interface::{Data, Size, Write};
use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

mod processing;
pub use processing::DispersedFringeSensorProcessing;

use super::{builders::DispersedFringeSensorBuilder, SensorPropagation};

/// GMT AGWS dispersed fringe sensor model
///
/// The number of frames that are co-added before resetting the camera is given by `C`
/// and the number of frame FFTs that are co-added is given by `F`.
///
/// # Examples:
///
/// Build a dispersed fringe sensor with the default [DispersedFringeSensorBuilder]
/// coadding 10 of the frame FFTs.
/// ```no_run
/// use gmt_dos_clients_crseo::sensors::DispersedFringeSensor;
/// use crseo::{Builder, FromBuilder};
///
/// let dfs = DispersedFringeSensor::<1,10>::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
pub struct DispersedFringeSensor<const C: usize = 1, const F: usize = 1>(
    pub(super) SegmentPistonSensor,
);

impl<const C: usize, const F: usize> FromBuilder for DispersedFringeSensor<C, F> {
    type ComponentBuilder = DispersedFringeSensorBuilder<C, F>;
}

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
impl<const C: usize, const F: usize> Display for DispersedFringeSensor<C, F> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
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
        // let q: Vec<f32> = self.frame().into();
        // dbg!(q.len());
        if self.n_camera_frame() == C {
            self.fft();
        }
    }
}

impl<const C: usize, const F: usize> Write<Frame<Dev>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn write(&mut self) -> Option<Data<Frame<Dev>>> {
        self.sensor_mut()
            .map(|sensor| Data::new(sensor.frame().clone()))
    }
}

impl<const C: usize, const F: usize> Write<Frame<Host>>
    for OpticalModel<DispersedFringeSensor<C, F>>
{
    fn write(&mut self) -> Option<Data<Frame<Host>>> {
        self.sensor_mut()
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

impl<const C: usize, const F: usize> Display for OpticalModel<DispersedFringeSensor<F, C>> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "- OPTICAL MODEL -")?;
        self.gmt.fmt(f)?;
        self.src.fmt(f)?;
        if let Some(atm) = &self.atm {
            atm.fmt(f)?;
        }
        self.sensor.as_ref().unwrap().fmt(f)?;
        writeln!(f, "DFS camera reset @{C} & FFT reset @{F} in sample #")?;
        writeln!(f, "-----------------")?;
        Ok(())
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
