use crseo::{
    Builder,
    builders::{GmtBuilder, SourceBuilder},
};

use crate::sensors::SegmentGradientSensor;

use super::{SensorBuilderProperty, WaveSensorBuilder};

#[derive(Debug, Clone)]
pub struct SegmentGradientSensorBuilder(WaveSensorBuilder);

impl Default for SegmentGradientSensorBuilder {
    fn default() -> Self {
        Self(WaveSensorBuilder::default().with_segment_gradient())
    }
}

impl SegmentGradientSensorBuilder {
    pub fn gmt(mut self, gmt: GmtBuilder) -> Self {
        self.0.omb = self.0.omb.gmt(gmt);
        self
    }
    pub fn source(mut self, source: SourceBuilder) -> Self {
        self.0.omb = self.0.omb.source(source);
        self
    }
}

impl Builder for SegmentGradientSensorBuilder {
    type Component = SegmentGradientSensor;

    fn build(self) -> crseo::Result<Self::Component> {
        Ok(SegmentGradientSensor(self.0.build()?))
    }
}

impl SensorBuilderProperty for SegmentGradientSensorBuilder {
    fn pupil_sampling(&self) -> Option<usize> {
        Some(self.0.omb.src.pupil_sampling.side())
    }
}
