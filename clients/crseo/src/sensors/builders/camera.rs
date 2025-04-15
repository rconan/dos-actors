use crseo::{
    builders::ImagingBuilder,
    imaging::{Detector, LensletArray},
    Builder,
};

use crate::{sensors::Camera, OpticalModel, OpticalModelBuilder, OpticalModelError};

use super::SensorBuilderProperty;

/// [Camera] builder
///
/// [CameraBuilder] is a newtype around [crseo ImagingBuilder](https://docs.rs/crseo/latest/crseo/imaging).
///
/// The number of frames that are co-added before resetting the camera is given by `I`.
///
/// # Examples
///
/// Build a camera with the default values for [ImagingBuilder](https://docs.rs/crseo/latest/crseo/imaging)
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
    ///
    /// ```
    /// # use gmt_dos_clients_crseo::sensors::Camera;
    /// # use crseo::{Builder, FromBuilder};
    /// let cam = Camera::<1>::builder().n_sensor(3).build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn n_sensor(mut self, n_sensor: usize) -> Self {
        self.0 = ImagingBuilder::n_sensor(self.0, n_sensor);
        self
    }
    /// Sets the [lenslet array](https://docs.rs/crseo/latest/crseo/imaging) property
    ///
    /// Camera with a 48x48 lenslet array with 16 pixel across each lenslet in the exit pupil:
    /// ```
    /// # use gmt_dos_clients_crseo::sensors::Camera;
    /// # use crseo::{Builder, FromBuilder};
    /// use crseo::imaging::LensletArray;
    /// let cam = Camera::<1>::builder()
    ///    .lenslet_array(
    ///       LensletArray::default().n_side_lenslet(48).n_px_lenslet(16))
    ///    .build()?;
    /// assert_eq!(cam.resolution(), 48*16);
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn lenslet_array(mut self, lenslet_array: LensletArray) -> Self {
        self.0 = ImagingBuilder::lenslet_array(self.0, lenslet_array);
        self
    }
    /// Sets the [detector](https://docs.rs/crseo/latest/crseo/imaging) property
    ///
    /// Camera with a 48x48 lenslet array with 16 pixel across each lenslet in the exit pupil
    /// and 8 pixel per lenslet in the camera frame:
    /// ```
    /// # use gmt_dos_clients_crseo::sensors::Camera;
    /// # use crseo::{Builder, FromBuilder};
    /// use crseo::imaging::{Detector, LensletArray};
    /// let cam = Camera::<1>::builder()
    ///    .lenslet_array(
    ///       LensletArray::default().n_side_lenslet(48).n_px_lenslet(24))
    ///    .detector(Detector::default().n_px_imagelet(8))
    ///    .build()?;
    /// assert_eq!(cam.resolution(), 48*8);
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn detector(mut self, detector: Detector) -> Self {
        self.0 = ImagingBuilder::detector(self.0, detector);
        self
    }
    /// Lenslet selection based on lenslet flux threshold (≤1) with respect to the flux of a fully illuminated lenslet
    ///
    /// Camera with a 48x48 lenslet array with 16 pixel across each lenslet in the exit pupil,
    /// 8 pixel per lenslet in the camera frame and a 50% flux threshold:
    /// ```
    /// # use gmt_dos_clients_crseo::sensors::Camera;
    /// # use crseo::{Builder, FromBuilder};
    /// use crseo::imaging::{Detector, LensletArray};
    /// let cam = Camera::<1>::builder()
    ///    .lenslet_array(
    ///       LensletArray::default().n_side_lenslet(48).n_px_lenslet(24))
    ///    .detector(Detector::default().n_px_imagelet(8))
    ///    .lenslet_flux(0.5)
    ///    .build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn lenslet_flux(mut self, threshold: f64) -> Self {
        self.0 = ImagingBuilder::lenslet_flux(self.0, threshold);
        self
    }
    /// Clones the [CameraBuilder] into another [CameraBuilder] with a different frame integration value
    pub fn clone_into<const CO: usize>(&self) -> CameraBuilder<CO> {
        CameraBuilder(self.0.clone())
    }
}

impl<const CI: usize> OpticalModelBuilder<CameraBuilder<CI>> {
    pub fn clone_into<const CO: usize>(&self) -> OpticalModelBuilder<CameraBuilder<CO>> {
        self.clone_with_sensor(self.sensor.as_ref().unwrap().clone_into::<CO>())
    }
}
impl<const I: usize> OpticalModel<Camera<I>> {
    /// Sensor pointing error relative to the source
    ///
    /// The pointing error is given in cartesian coordinates and in radians units
    pub fn sensor_pointing(&mut self, xy: &[(f64, f64)]) -> Result<&mut Self, OpticalModelError> {
        let Some(sensor) = &mut self.sensor else {
            return Err(OpticalModelError::MissingSensor);
        };
        sensor.pointing(xy, &self.src);
        Ok(self)
    }
}
