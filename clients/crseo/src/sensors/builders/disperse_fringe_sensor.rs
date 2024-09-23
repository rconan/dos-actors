use crseo::{segment_piston_sensor::SegmentPistonSensorBuilder, Builder};

use crate::{
    sensors::{DispersedFringeSensor, DispersedFringeSensorProcessing},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};

use super::SensorBuilderProperty;

/// [DispersedFringeSensor] builder
///
/// The number of frames that are co-added before resetting the camera is given by `C`
/// and the number of frame FFTs that are co-added is given by `F`.
///
/// # Examples:
///
/// Build a dispersed fringe sensor with the default values for [SegmentPistonSensorBuilder]
///
/// ```no_run
/// use gmt_dos_clients_crseo::sensors::DispersedFringeSensor;
/// use crseo::{Builder, FromBuilder};
///
/// let dfs = DispersedFringeSensor::<1,1>::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
#[derive(Default, Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DispersedFringeSensorBuilder<const C: usize, const F: usize>(SegmentPistonSensorBuilder);

impl<const C: usize, const F: usize> Builder for DispersedFringeSensorBuilder<C, F> {
    type Component = DispersedFringeSensor<C, F>;

    fn build(self) -> crseo::Result<Self::Component> {
        Ok(DispersedFringeSensor(self.0.build()?))
    }
}

// impl<const C: usize, const F: usize> OpticalModelBuilder<DispersedFringeSensorBuidler<C, F>> {
//     pub fn build(self) -> Result<OpticalModel<DispersedFringeSensor<C, F>>> {
//         let dfs = self
//             .sensor
//             .unwrap()
//             .0
//             .gmt(self.gmt.clone())
//             .src(self.src.clone())
//             .build()?;
//         Ok(OpticalModel {
//             gmt: self.gmt.build()?,
//             src: self.src.build()?,
//             sensor: Some(DispersedFringeSensor(dfs)),
//             atm: self.atm_builder.map(|atm| atm.build()).transpose()?,
//             tau: self.sampling_frequency.map_or_else(|| 0f64, |x| x.recip()),
//         })
//     }
// }

impl<const C: usize, const F: usize> SensorBuilderProperty for DispersedFringeSensorBuilder<C, F> {}

impl<const C: usize, const F: usize> DeviceInitialize<DispersedFringeSensorProcessing>
    for OpticalModelBuilder<DispersedFringeSensorBuilder<C, F>>
{
    fn initialize(&mut self, device: &mut DispersedFringeSensorProcessing) {
        let mut om = self
            .clone_with_sensor(self.sensor.as_ref().unwrap().clone_into::<1, 1>())
            .build()
            .unwrap();
        println!("{om}");
        <OpticalModel<DispersedFringeSensor<1, 1>> as interface::Update>::update(&mut om);
        let mut dfsp0 = DispersedFringeSensorProcessing::from(om.sensor_mut().unwrap());
        device.set_reference(dfsp0.intercept());
    }
}

impl<const C: usize, const F: usize> DispersedFringeSensorBuilder<C, F> {
    /// Sets the GMT builder
    pub fn gmt(mut self, gmt: crseo::gmt::GmtBuilder) -> Self {
        self.0 = self.0.gmt(gmt);
        self
    }
    ///  Sets the source builder
    pub fn source(mut self, src: crseo::source::SourceBuilder) -> Self {
        self.0 = self.0.src(src);
        self
    }
    /// Sets the size of a lenslet
    pub fn lenslet_size(mut self, lenslet_size: f64) -> Self {
        self.0 = self.0.lenslet_size(lenslet_size);
        self
    }
    /// Sets the dispersion in rd/m
    pub fn dispersion(mut self, dispersion: f64) -> Self {
        self.0 = self.0.dispersion(dispersion);
        self
    }
    /// Sets the field-of-view in rd
    pub fn field_of_view(mut self, field_of_view: f64) -> Self {
        self.0 = self.0.field_of_view(field_of_view);
        self
    }
    /// Sets the nyquist factor
    pub fn nyquist_factor(mut self, nyquist_factor: f64) -> Self {
        self.0 = self.0.nyquist_factor(nyquist_factor);
        self
    }
    /// Sets the image binning factor
    pub fn bin_image(mut self, bin_image: usize) -> Self {
        self.0 = self.0.bin_image(bin_image);
        self
    }
    /// Sets the DFT memory pre-allocation flag
    pub fn malloc_dft(mut self, malloc_dft: bool) -> Self {
        self.0 = self.0.malloc_dft(malloc_dft);
        self
    }
    /// Sets the lenslet mask width
    pub fn middle_mask_width(mut self, middle_mask_width: f64) -> Self {
        self.0 = self.0.middle_mask_width(middle_mask_width);
        self
    }
    /// Clones the builder into another one with different constants `C` and `F`
    pub fn clone_into<const CO: usize, const FO: usize>(
        &self,
    ) -> DispersedFringeSensorBuilder<CO, FO> {
        DispersedFringeSensorBuilder(self.0.clone())
    }
}
