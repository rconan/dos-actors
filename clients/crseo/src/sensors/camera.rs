use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use crseo::{FromBuilder, Imaging};
use skyangle::Conversion;

use crate::OpticalModel;

use super::{builders::CameraBuilder, SensorPropagation};

mod interface;

/// Optical model camera
///
/// [Camera] is a newtype around [crseo Imaging](https://docs.rs/crseo/latest/crseo/imaging).
///
/// The number of frames that are co-added before resetting the camera is given by `I`.
///
/// # Examples:
///
/// Build a camera with the default [CameraBuilder] and without co-adding the frames.
/// ```
/// use gmt_dos_clients_crseo::sensors::Camera;
/// use crseo::{Builder, FromBuilder};
///
/// let cam = Camera::<1>::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
pub struct Camera<const I: usize = 1>(pub(super) Imaging);

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
            // println!("resetting ({})", self.n_frame());
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
            r#"Pixel scale: {:.3}mas, Field-of-view: {:.3}""#,
            self.pixel_scale().to_mas(),
            self.field_of_view().to_arcsec()
        )?;
        writeln!(f, "-----------------")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use ::interface::{Update, Write};
    use crseo::{
        imaging::{ImagingBuilder, LensletArray},
        Builder, Gmt, Source,
    };
    use gmt_dos_clients_io::optics::{Frame, Host};

    use super::*;

    #[test]
    fn imgr_flux() -> Result<(), Box<dyn std::error::Error>> {
        let n_gs = 3;
        let flux0 = {
            let mut gmt = Gmt::builder().build()?;
            let mut src = Source::builder()
                .size(n_gs)
                .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs])
                .build()?;
            println!("{src}");

            let imgr_builder = ImagingBuilder::default().n_sensor(n_gs);

            let mut imgr = imgr_builder.build()?;
            println!("{imgr}");

            src.through(&mut gmt).xpupil().through(&mut imgr);

            let frame = imgr.frame();
            Vec::<f32>::from(&frame).iter().sum::<f32>()
        };

        let mut om: OpticalModel<Camera> = OpticalModel::<Camera<1>>::builder()
            .source(
                Source::builder()
                    .size(n_gs)
                    .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs]),
            )
            .sensor(Camera::<1>::builder().n_sensor(n_gs))
            .build()?;
        om.update();
        println!("{om}");
        <OpticalModel<Camera<1>> as Write<Frame<Host>>>::write(&mut om)
            .map(|data| data.iter().sum::<f32>())
            .map(|x| assert_eq!(x, flux0));

        Ok(())
    }

    #[test]
    fn sh_flux() -> Result<(), Box<dyn std::error::Error>> {
        let n_gs = 3;
        let n_lenslet = 48;
        let n_px_lenslet = 32;
        let flux0 = {
            let mut gmt = Gmt::builder().build()?;
            let mut src = Source::builder()
                .size(n_gs)
                .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs])
                .pupil_sampling(n_lenslet * n_px_lenslet + 1)
                .build()?;
            println!("{src}");

            let imgr_builder = ImagingBuilder::default().n_sensor(n_gs).lenslet_array(
                LensletArray::default()
                    .n_side_lenslet(n_lenslet)
                    .n_px_lenslet(n_px_lenslet),
            );

            let mut imgr = imgr_builder.build()?;
            println!("{imgr}");

            src.through(&mut gmt).xpupil().through(&mut imgr);

            let frame = imgr.frame();
            Vec::<f32>::from(&frame).iter().sum::<f32>()
        };

        let mut om: OpticalModel<Camera> = OpticalModel::<Camera<1>>::builder()
            .source(
                Source::builder()
                    .size(n_gs)
                    .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs]),
            )
            .sensor(
                Camera::<1>::builder().n_sensor(n_gs).lenslet_array(
                    LensletArray::default()
                        .n_side_lenslet(n_lenslet)
                        .n_px_lenslet(n_px_lenslet),
                ),
            )
            .build()?;
        om.update();
        println!("{om}");
        <OpticalModel<Camera<1>> as Write<Frame<Host>>>::write(&mut om)
            .map(|data| data.iter().sum::<f32>())
            .map(|x| assert_eq!(x, flux0));

        Ok(())
    }
}
