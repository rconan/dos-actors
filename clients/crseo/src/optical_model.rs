use crate::{
    OpticalModelBuilder,
    sensors::{NoSensor, SensorPropagation},
};
use crseo::{Atmosphere, FromBuilder, Gmt, PSSnEstimates, Source};
use interface::{Units, Update, optics::Optics};

pub mod builder;
mod imaging;
mod pyramid;
mod write;
mod read;

#[derive(Debug, thiserror::Error)]
pub enum OpticalModelError {
    #[error("failed to build optical model")]
    Crseo(#[from] crseo::error::CrseoError),
    #[error("atmosphere is set but not the sampling frequency")]
    AtmosphereWithoutSamplingFrequency,
    #[error("no sensor has been set")]
    MissingSensor,
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
}
unsafe impl<T> Send for OpticalModel<T> {}
unsafe impl<T> Sync for OpticalModel<T> {}

impl<T> Units for OpticalModel<T> {}
impl<T> Optics for OpticalModel<T> {}

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
            ..
        } = OpticalModelBuilder::<NoSensor>::default();
        OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sensor: Some(T::builder()),
            sampling_frequency,
            pssn,
        }
    }
}

impl<T: SensorPropagation> Update for OpticalModel<T> {
    fn update(&mut self) {
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


