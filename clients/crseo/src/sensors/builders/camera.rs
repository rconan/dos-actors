use crseo::{
    imaging::{Detector, ImagingBuilder, LensletArray},
    Builder,
};

use crate::{sensors::Camera, OpticalModelBuilder};

use super::SensorBuilderProperty;

/// [Camera] builder
///
/// [CameraBuilder] is a newtype around [ImagingBuilder].
///
/// The number of frames that are co-added before resetting the camera is given by `I`.
///
/// # Examples:
///
/// Build a camera with the default values for [ImagingBuilder]
///
/// ```
/// use gmt_dos_clients_crseo::sensors::Camera;
/// use crseo::{Builder, FromBuilder};
///
/// let cam = Camera::<1>::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
#[derive(Default, Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CameraBuilder<const I: usize = 1>(pub(crate) ImagingBuilder);

impl<const I: usize> From<OpticalModelBuilder<CameraBuilder<I>>>
    for OpticalModelBuilder<ImagingBuilder>
{
    fn from(omc: OpticalModelBuilder<CameraBuilder<I>>) -> Self {
        let OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sensor,
            sampling_frequency,
        } = omc;
        OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sensor: sensor.map(|camera| camera.0),
            sampling_frequency,
        }
    }
}

impl<const I: usize> From<&OpticalModelBuilder<CameraBuilder<I>>>
    for OpticalModelBuilder<ImagingBuilder>
{
    fn from(omc: &OpticalModelBuilder<CameraBuilder<I>>) -> Self {
        let OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sensor,
            sampling_frequency,
        } = omc.clone();
        OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sensor: sensor.map(|camera| camera.0),
            sampling_frequency,
        }
    }
}

impl<const I: usize> SensorBuilderProperty for CameraBuilder<I> {
    fn pupil_sampling(&self) -> Option<usize> {
        <ImagingBuilder as SensorBuilderProperty>::pupil_sampling(&self.0)
    }
}

impl<const I: usize> Builder for CameraBuilder<I> {
    type Component = Camera<I>;

    fn build(self) -> crseo::Result<Self::Component> {
        Ok(Camera(self.0.build()?))
    }
}

impl<const I: usize> CameraBuilder<I> {
    /// Sets the # of sensors
    pub fn n_sensor(mut self, n_sensor: usize) -> Self {
        self.0 = ImagingBuilder::n_sensor(self.0, n_sensor);
        self
    }
    /// Sets the [lenslet array][LensletArray] property
    pub fn lenslet_array(mut self, lenslet_array: LensletArray) -> Self {
        self.0 = ImagingBuilder::lenslet_array(self.0, lenslet_array);
        self
    }
    /// Sets the [detector](Detector) property
    pub fn detector(mut self, detector: Detector) -> Self {
        self.0 = ImagingBuilder::detector(self.0, detector);
        self
    }
    /// Lenslet selection based on lenslet flux threshold
    pub fn lenslet_flux(mut self, threshold: f64) -> Self {
        self.0 = ImagingBuilder::lenslet_flux(self.0, threshold);
        self
    }
}
