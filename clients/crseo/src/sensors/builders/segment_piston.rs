use crseo::{gmt::GmtBuilder, source::SourceBuilder, Builder};

use crate::sensors::SegmentPistonSensor;

use super::{SensorBuilderProperty, WaveSensorBuilder};

#[derive(Debug, Clone)]
pub struct SegmentPistonSensorBuilder(WaveSensorBuilder);

impl Default for SegmentPistonSensorBuilder {
    fn default() -> Self {
        Self(WaveSensorBuilder::default().with_segment_piston())
    }
}

impl SegmentPistonSensorBuilder {
    pub fn gmt(mut self, gmt: GmtBuilder) -> Self {
        self.0.omb = self.0.omb.gmt(gmt);
        self
    }
    pub fn source(mut self, source: SourceBuilder) -> Self {
        self.0.omb = self.0.omb.source(source);
        self
    }
}

impl Builder for SegmentPistonSensorBuilder {
    type Component = SegmentPistonSensor;

    fn build(self) -> crseo::Result<Self::Component> {
        Ok(SegmentPistonSensor(self.0.build()?))
    }
}

impl SensorBuilderProperty for SegmentPistonSensorBuilder {
    fn pupil_sampling(&self) -> Option<usize> {
        Some(self.0.omb.src.pupil_sampling.side())
    }
}
