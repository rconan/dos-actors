use crate::{
    OpticalModelBuilder,
    sensors::{NoSensor, SensorPropagation},
};
use crseo::{Atmosphere, FromBuilder, Gmt, PSSnEstimates, Source};
use interface::{Units, Update};

pub mod builder;
mod imaging;
mod pyramid;
mod read;
mod write;

#[derive(Debug, thiserror::Error)]
pub enum OpticalModelError {
    #[error("failed to build optical model")]
    Crseo(#[from] crseo::error::CrseoError),
    #[error("atmosphere is set but not the sampling frequency")]
    AtmosphereWithoutSamplingFrequency,
    #[error("no sensor has been set")]
    MissingSensor,
    #[error("CUDA device #{device} does not exist ({n} devices)")]
    InvalidDevice { device: i32, n: i32 },
}

/// GMT optical model
///
/// GMT M1 and M2 optical prescriptions.
/// May as well include a [sensor](crate::sensors) and a model of the [atmospheric turbulence](https://docs.rs/crseo/latest/crseo/atmosphere).
///
/// ## Example
///
/// Build a optical model with the default [OpticalModelBuilder]
/// ```
/// use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
///
/// let om = OpticalModel::<NoSensor>::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
pub struct OpticalModel<T = NoSensor> {
    pub(crate) gmt: Gmt,
    pub(crate) src: Source,
    pub(crate) atm: Option<Atmosphere>,
    pub(crate) sensor: Option<T>,
    pub(crate) pssn: Option<Box<dyn PSSnEstimates>>,
    pub(crate) tau: f64,
    pub(crate) phase_offset: Option<Vec<f64>>,
    pub(crate) pointing: Pointing,
    pub(crate) device: i32,
}

/// Telescope pointing error
///
/// The telescope optical axis is offset by `error` `[x,y]` `[rd]` from the source and the
/// instrument is rotated by `rotation` `[rd]` about the optical axis relative to the sky.
/// The source directions seen from the instrument are rotated by `-rotation` and offset by `-error`,
/// the nominal directions (`Source::zenith` and `Source::azimuth`) being those without pointing error.
#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct Pointing {
    pub(crate) error: [f64; 2],
    pub(crate) rotation: f64,
}
impl Pointing {
    /// Sets the source directions to the directions seen from the instrument
    ///
    /// The source is ray traced along these directions and, with a camera in absolute pointing
    /// ([OpticalModel::sensor_pointing](crate::OpticalModel::sensor_pointing)), the images are
    /// offset by the difference between these directions and the camera pointing direction
    pub(crate) fn apply(&self, src: &mut Source) {
        let (zenith, azimuth) = self.directions(src);
        src.update(zenith, azimuth);
    }
    /// Source (zenith, azimuth) directions seen from the instrument
    fn directions(&self, src: &Source) -> (Vec<f64>, Vec<f64>) {
        let (s, c) = self.rotation.sin_cos();
        let [px, py] = self.error;
        src.zenith
            .iter()
            .zip(&src.azimuth)
            .map(|(&z, &a)| {
                let (sa, ca) = (a as f64).sin_cos();
                let (x, y) = (z as f64 * ca, z as f64 * sa);
                (c * x + s * y - px, -s * x + c * y - py)
            })
            .map(|(x, y)| (x.hypot(y), y.atan2(x)))
            .unzip()
    }
}

impl<T> OpticalModel<T> {
    /// Returns a mutable reference to the sensor
    pub fn sensor_mut(&mut self) -> Option<&mut T> {
        self.sensor.as_mut()
    }
    /// Returns an immutable reference to the sensor
    pub fn sensor(&self) -> Option<&T> {
        self.sensor.as_ref()
    }
    /// Returns a mutable reference to the source
    pub fn source(&self) -> &Source {
        &self.src
    }
    /// Returns a immutable reference to the source
    pub fn source_mut(&mut self) -> &mut Source {
        &mut self.src
    }
    pub fn phase_offset(&mut self, phase_offset: &[f64]) -> &mut Self {
        self.phase_offset = Some(phase_offset.to_vec());
        self
    }
    /// Returns the CUDA device of the optical model
    pub fn device(&self) -> i32 {
        self.device
    }
    /// Makes the CUDA device of the optical model the current device of the calling thread
    ///
    /// The CUDA device is a property of the host thread and the actor of the optical model
    /// may run on any thread, so the device is selected at every entry point
    /// of the optical model (update, read, write and drop)
    pub fn select_device(&self) {
        crseo::set_gpu(self.device);
    }
}

/// The GPU memory of the optical model is freed on its own device
impl<T> Drop for OpticalModel<T> {
    fn drop(&mut self) {
        self.select_device();
    }
}
unsafe impl<T> Send for OpticalModel<T> {}
unsafe impl<T> Sync for OpticalModel<T> {}

impl<T> Units for OpticalModel<T> {}

impl<T> OpticalModel<T>
where
    T: FromBuilder,
{
    /// Creates an optical model builder
    pub fn builder() -> OpticalModelBuilder<<T as FromBuilder>::ComponentBuilder> {
        let OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sampling_frequency,
            pssn,
            device,
            ..
        } = OpticalModelBuilder::<NoSensor>::default();
        OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sensor: Some(T::builder()),
            sampling_frequency,
            pssn,
            device,
        }
    }
}

impl<T: SensorPropagation> Update for OpticalModel<T> {
    fn update(&mut self) {
        self.select_device();
        self.src.through(&mut self.gmt).xpupil();
        if let Some(atm) = &mut self.atm {
            atm.secs += self.tau;
            self.src.through(atm);
        }
        if let Some(phase) = self.phase_offset.as_ref() {
            self.src.add(phase);
        }
        if let Some(pssn) = &mut self.pssn {
            self.src.through(pssn);
        }
        if let Some(sensor) = &mut self.sensor {
            sensor.propagate(&mut self.src);
        }
    }
}
