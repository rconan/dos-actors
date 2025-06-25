use super::{OpticalModel, OpticalModelError};
use crate::sensors::{
    NoSensor, SensorPropagation,
    builders::{SensorBuilderProperty, WaveSensorBuilder},
};
use crseo::{
    Builder, PSSnEstimates,
    builders::{AtmosphereBuilder, GmtBuilder, SourceBuilder},
    pssn::{AtmosphereTelescopeError, PSSnBuilder, TelescopeError},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PSSnKind {
    TelescopeError(PSSnBuilder<TelescopeError>),
    AtmosphereTelescopeError(PSSnBuilder<AtmosphereTelescopeError>),
}

/// GMT optical model builder
///
/// # Examples
///
/// Build a optical model with the default values for [GmtBuilder](https://docs.rs/crseo/latest/crseo/gmt)
/// and for [SourceBuilder](https://docs.rs/crseo/latest/crseo/source) and without sensor

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
    pub(crate) pssn: Option<PSSnKind>,
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
    /// let om = OpticalModel::<NoSensor>::builder()
    ///     .gmt(Gmt::builder().m1_n_mode(21))
    ///     .build()?;
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
    /// let om = OpticalModel::<NoSensor>::builder()
    ///     .source(Source::builder().band("K"))
    ///     .build()?;
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
    ///     .atmosphere(Atmosphere::builder())
    ///     .build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn atmosphere(self, atm_builder: AtmosphereBuilder) -> Self {
        Self {
            pssn: self.pssn.map(|kind| {
                if let PSSnKind::TelescopeError(_) = kind {
                    PSSnKind::AtmosphereTelescopeError(PSSnBuilder::from(&atm_builder))
                } else {
                    kind
                }
            }),
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
    /// let om = OpticalModel::<WaveSensor>::builder()
    ///     .sensor(WaveSensor::builder())
    ///     .build()?;
    /// # Ok::<(),Box<dyn std::error::Error>>(())
    /// ```
    pub fn sensor(mut self, builder: S) -> Self {
        self.sensor = Some(builder);
        self
    }
    /// Enables PSSn estimator
    pub fn with_pssn(mut self) -> Self {
        self.pssn = self.pssn.or(Some(self.atm_builder.as_ref().map_or(
            PSSnKind::TelescopeError(PSSnBuilder::new()),
            |atm_builder| PSSnKind::AtmosphereTelescopeError(PSSnBuilder::from(atm_builder)),
        )));
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
    ///     .atmosphere(Atmosphere::builder())
    ///     .build()?;
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
            pssn,
            ..
        } = self;
        OpticalModelBuilder {
            gmt: gmt.clone(),
            src: src.clone(),
            atm_builder: atm_builder.clone(),
            sensor: Some(sensor),
            sampling_frequency: sampling_frequency.clone(),
            pssn: pssn.clone(),
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
        let om = OpticalModel {
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
            pssn: if let Some(kind) = self.pssn {
                match kind {
                    PSSnKind::TelescopeError(pssn_builder) => {
                        Some(Box::new(pssn_builder.build()?) as Box<dyn PSSnEstimates>)
                    }
                    PSSnKind::AtmosphereTelescopeError(pssn_builder) => {
                        Some(Box::new(pssn_builder.build()?) as Box<dyn PSSnEstimates>)
                    }
                }
            } else {
                None
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
            phase_offset: None,
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

#[cfg(test)]
mod tests {
    use std::error::Error;

    use gmt_dos_clients_io::optics::PSSn;
    use interface::{Update, Write};

    use super::*;

    #[test]
    fn builder() {
        assert!(OpticalModel::<NoSensor>::builder().build().is_ok());
    }

    #[test]
    fn pssn() -> Result<(), Box<dyn Error>> {
        let mut om = OpticalModel::<NoSensor>::builder().with_pssn().build()?;
        om.update();
        let pssn = <_ as Write<PSSn>>::write(&mut om).unwrap();
        assert_eq!(pssn[0], 1.0);
        Ok(())
    }
}
