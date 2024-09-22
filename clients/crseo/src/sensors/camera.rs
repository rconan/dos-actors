use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use crseo::{FromBuilder, Imaging};
use skyangle::Conversion;

use crate::{OpticalModel, SensorPropagation};

mod builder;
pub use builder::CameraBuilder;
mod interface;

pub struct Camera<const I: usize = 1>(Imaging);

impl<const I: usize> Deref for Camera<I> {
    type Target = Imaging;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const I: usize> DerefMut for Camera<I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const I: usize> FromBuilder for Camera<I> {
    type ComponentBuilder = CameraBuilder<I>;
}

impl<const I: usize> SensorPropagation for Camera<I> {
    fn propagate(&mut self, src: &mut crseo::Source) {
        if self.n_frame() as usize == I {
            self.reset();
        }
        src.through(&mut self.0);
    }
}

impl<const I: usize> Display for Camera<I> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl<const I: usize> OpticalModel<Camera<I>> {
    /// Returns the camera pixel scale in radians
    pub fn pixel_scale(&self) -> f64 {
        self.sensor.as_ref().unwrap().pixel_scale(&self.src) as f64
    }
    /// Returns the camera field-of-view in radians
    pub fn field_of_view(&self) -> f64 {
        self.sensor.as_ref().unwrap().field_of_view(&self.src) as f64
    }
}

impl<const I: usize> Display for OpticalModel<Camera<I>> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "- OPTICAL MODEL -")?;
        self.gmt.fmt(f)?;
        self.src.fmt(f)?;
        if let Some(atm) = &self.atm {
            atm.fmt(f)?;
        }
        self.sensor.as_ref().unwrap().fmt(f)?;
        writeln!(
            f,
            r#"Pixel scale: {:.0}mas, Field-of-view: {:.3}""#,
            self.pixel_scale().to_mas(),
            self.field_of_view().to_arcsec()
        )?;
        writeln!(f, "-----------------")?;
        Ok(())
    }
}
