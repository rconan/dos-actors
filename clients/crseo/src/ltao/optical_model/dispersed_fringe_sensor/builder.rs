use crseo::{segment_piston_sensor::SegmentPistonSensorBuilder, Builder, FromBuilder};

use crate::{OpticalModel, OpticalModelBuilder};

use super::{DispersedFringeSensor, Result};

#[derive(Default, Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DispersedFringeSensorBuidler<const C: usize, const F: usize>(SegmentPistonSensorBuilder);

impl<const C: usize, const F: usize> Builder for DispersedFringeSensorBuidler<C, F> {
    type Component = DispersedFringeSensor<C, F>;

    fn build(self) -> crseo::Result<Self::Component> {
        Ok(DispersedFringeSensor(self.0.build()?))
    }
}

impl<const C: usize, const F: usize> FromBuilder for DispersedFringeSensor<C, F> {
    type ComponentBuilder = DispersedFringeSensorBuidler<C, F>;
}

impl<const C: usize, const F: usize> OpticalModelBuilder<DispersedFringeSensorBuidler<C, F>> {
    pub fn build(self) -> Result<OpticalModel<DispersedFringeSensor<C, F>>> {
        let dfs = self
            .sensor
            .unwrap()
            .0
            .gmt(self.gmt.clone())
            .src(self.src.clone())
            .build()?;
        Ok(OpticalModel {
            gmt: self.gmt.build()?,
            src: self.src.build()?,
            sensor: Some(DispersedFringeSensor(dfs)),
            atm: self.atm_builder.map(|atm| atm.build()).transpose()?,
            tau: self.sampling_frequency.map_or_else(|| 0f64, |x| x.recip()),
        })
    }
}

impl<const C: usize, const F: usize> DispersedFringeSensorBuidler<C, F> {
    pub fn gmt(mut self, gmt: crseo::gmt::GmtBuilder) -> Self {
        self.0 = self.0.gmt(gmt);
        self
    }
    pub fn src(mut self, src: crseo::source::SourceBuilder) -> Self {
        self.0 = self.0.src(src);
        self
    }
    pub fn lenslet_size(mut self, lenslet_size: f64) -> Self {
        self.0 = self.0.lenslet_size(lenslet_size);
        self
    }
    pub fn dispersion(mut self, dispersion: f64) -> Self {
        self.0 = self.0.dispersion(dispersion);
        self
    }
    pub fn field_of_view(mut self, field_of_view: f64) -> Self {
        self.0 = self.0.field_of_view(field_of_view);
        self
    }
    pub fn nyquist_factor(mut self, nyquist_factor: f64) -> Self {
        self.0 = self.0.nyquist_factor(nyquist_factor);
        self
    }
    pub fn bin_image(mut self, bin_image: usize) -> Self {
        self.0 = self.0.bin_image(bin_image);
        self
    }
    pub fn malloc_dft(mut self, malloc_dft: bool) -> Self {
        self.0 = self.0.malloc_dft(malloc_dft);
        self
    }
    pub fn middle_mask_width(mut self, middle_mask_width: f64) -> Self {
        self.0 = self.0.middle_mask_width(middle_mask_width);
        self
    }
}
