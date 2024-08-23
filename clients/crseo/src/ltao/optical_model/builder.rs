use super::OpticalModel;
use crate::ltao::SensorBuilderProperty;
use crate::NoSensor;
use crseo::gmt::GmtBuilder;
use crseo::source::SourceBuilder;
use crseo::Builder;

#[derive(Debug, Default, Clone)]
pub struct OpticalModelBuilder<S = NoSensor> {
    pub(crate) gmt: GmtBuilder,
    pub(crate) src: SourceBuilder,
    pub(crate) sensor: Option<S>,
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
    pub fn sensor(mut self, builder: S) -> Self {
        self.sensor = Some(builder);
        self
    }
}
impl<T, S> OpticalModelBuilder<S>
where
    S: Builder<Component = T> + SensorBuilderProperty,
{
    pub fn build(self) -> super::Result<OpticalModel<T>> {
        Ok(OpticalModel {
            gmt: self.gmt.build()?,
            src: if let Some(sensor) = &self.sensor {
                self.src.pupil_sampling(sensor.pupil_sampling()).build()?
            } else {
                self.src.build()?
            },
            sensor: if let Some(sensor) = self.sensor {
                Some(sensor.build()?)
            } else {
                None
            },
        })
    }
}
