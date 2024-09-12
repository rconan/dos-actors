use super::OpticalModel;
use crate::sensors::NoSensor;
use crate::SensorBuilderProperty;
use crseo::atmosphere::AtmosphereBuilder;
use crseo::gmt::GmtBuilder;
use crseo::source::SourceBuilder;
use crseo::Builder;

#[derive(Debug, Default, Clone)]
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
    pub fn gmt(mut self, builder: GmtBuilder) -> Self {
        self.gmt = builder;
        self
    }
    pub fn source(mut self, builder: SourceBuilder) -> Self {
        self.src = builder;
        self
    }
    pub fn atmosphere(self, atm_builder: AtmosphereBuilder) -> Self {
        Self {
            atm_builder: Some(atm_builder),
            ..self
        }
    }
    pub fn sensor(mut self, builder: S) -> Self {
        self.sensor = Some(builder);
        self
    }
    pub fn sampling_frequency(self, sampling_frequency: f64) -> Self {
        Self {
            sampling_frequency: Some(sampling_frequency),
            ..self
        }
    }
}
impl<T, S> OpticalModelBuilder<S>
where
    S: Builder<Component = T> + SensorBuilderProperty,
{
    pub fn build(self) -> super::Result<OpticalModel<T>> {
        Ok(OpticalModel {
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
            atm: self.atm_builder.map(|atm| atm.build()).transpose()?,
            sensor: self.sensor.map(|sensor| sensor.build()).transpose()?,
            tau: self.sampling_frequency.map_or_else(|| 0f64, |x| x.recip()),
        })
    }
}
