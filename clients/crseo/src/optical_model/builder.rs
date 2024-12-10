use super::{OpticalModel, OpticalModelError};
use crate::sensors::{
    builders::{SensorBuilderProperty, WaveSensorBuilder},
    NoSensor, SensorPropagation,
};
use crseo::{atmosphere::AtmosphereBuilder, gmt::GmtBuilder, source::SourceBuilder, Builder};
use serde::{Deserialize, Serialize};

/// GMT optical model builder
///
/// # Examples:
///
/// Build a optical model with the default values for [GmtBuilder](crseo::gmt::GmtBuilder)
/// and for [SourceBuilder](crseo::source::SourceBuilder) and without sensor
///
/// ```
/// use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
///
/// let om = OpticalModel::<NoSensor>::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct OpticalModelBuilder<S = NoSensor> {
    pub(crate) gmt: GmtBuilder,
    pub(crate) src: SourceBuilder,
    pub(crate) atm_builder: Option<AtmosphereBuilder>,
    pub(crate) sensor: Option<S>,
    pub(crate) sampling_frequency: Option<f64>,
}

impl<T, S> OpticalModelBuilder<S>
where
    S: Builder<Component = T>,
{
    /// Sets the GMT builder
    ///
    /// ```
    /// use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
    /// use crseo::{Gmt, FromBuilder};
    ///
    /// let om = OpticalModel::<NoSensor>::builder().gmt(Gmt::builder().m1_n_mode(21)).build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn gmt(mut self, builder: GmtBuilder) -> Self {
        self.gmt = builder;
        self
    }
    ///  Sets the source builder
    ///
    /// ```
    /// use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
    /// use crseo::{Source, FromBuilder};
    ///
    /// let om = OpticalModel::<NoSensor>::builder().source(Source::builder().band("K")).build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn source(mut self, builder: SourceBuilder) -> Self {
        self.src = builder;
        self
    }
    ///  Sets the atmosphere builder
    ///
    /// ```
    /// use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
    /// use crseo::{Atmosphere, FromBuilder};
    ///
    /// let om = OpticalModel::<NoSensor>::builder()
    ///     .sampling_frequency(1_000_f64) // 1kHz
    ///     .atmosphere(Atmosphere::builder()).build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn atmosphere(self, atm_builder: AtmosphereBuilder) -> Self {
        Self {
            atm_builder: Some(atm_builder),
            ..self
        }
    }
    /// Sets the optical sensor
    ///
    /// ```
    /// use gmt_dos_clients_crseo::{OpticalModel, sensors::WaveSensor};
    /// use crseo::FromBuilder;
    ///
    /// let om = OpticalModel::<WaveSensor>::builder().sensor(WaveSensor::builder());
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn sensor(mut self, builder: S) -> Self {
        self.sensor = Some(builder);
        self
    }
    /// Sets the sampling frequency in Hz
    ///
    /// ```
    /// use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
    /// use crseo::{Atmosphere, FromBuilder};
    ///
    /// let om = OpticalModel::<NoSensor>::builder()
    ///     .sampling_frequency(1_000_f64) // 1kHz
    ///     .atmosphere(Atmosphere::builder()).build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    pub fn sampling_frequency(self, sampling_frequency: f64) -> Self {
        Self {
            sampling_frequency: Some(sampling_frequency),
            ..self
        }
    }
    /// Clones the builder with a new [sensor](crate::sensors) builder
    /// ```
    /// use gmt_dos_clients_crseo::{OpticalModel, sensors::{NoSensor, Camera}};
    /// use crseo::FromBuilder;
    ///
    /// let omb = OpticalModel::<NoSensor>::builder();
    /// let om_cam: OpticalModel<Camera> = omb.clone_with_sensor(Camera::builder()).build()?;
    /// let om = omb.build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn clone_with_sensor<W>(&self, sensor: W) -> OpticalModelBuilder<W> {
        let Self {
            gmt,
            src,
            atm_builder,
            sampling_frequency,
            ..
        } = self;
        OpticalModelBuilder {
            gmt: gmt.clone(),
            src: src.clone(),
            atm_builder: atm_builder.clone(),
            sensor: Some(sensor),
            sampling_frequency: sampling_frequency.clone(),
        }
    }
    pub fn get_pupil_size_px(&self) -> usize {
        self.src.pupil_sampling.side()
    }
}

impl<T, S> OpticalModelBuilder<S>
where
    S: Builder<Component = T> + SensorBuilderProperty,
    T: SensorPropagation,
{
    pub fn build(self) -> Result<OpticalModel<T>, OpticalModelError> {
        let mut om = OpticalModel {
            gmt: self.gmt.build()?,
            src: if let &Some(n) = &self
                .sensor
                .as_ref()
                .and_then(|sensor| sensor.pupil_sampling())
            {
                self.src.pupil_sampling(n).build()?
            } else {
                self.src.build()?
            },
            atm: match self.atm_builder {
                Some(atm) => {
                    if self.sampling_frequency.is_some() {
                        Some(atm.build()?)
                    } else {
                        return Err(OpticalModelError::AtmosphereWithoutSamplingFrequency);
                    }
                }
                None => None,
            },
            sensor: self.sensor.map(|sensor| sensor.build()).transpose()?,
            tau: self.sampling_frequency.map_or_else(|| 0f64, |x| x.recip()),
        };
        // Propagation to initialize the detector frame in case of bootstrapping
        // <OpticalModel<_> as interface::Update>::update(&mut om);
        Ok(om)
    }
}

impl<T, S> From<&OpticalModelBuilder<S>> for OpticalModelBuilder<WaveSensorBuilder>
where
    S: Builder<Component = T> + SensorBuilderProperty,
{
    fn from(builder: &OpticalModelBuilder<S>) -> Self {
        builder.clone_with_sensor(WaveSensorBuilder::from(builder.clone_with_sensor(NoSensor)))
    }
}
