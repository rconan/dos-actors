use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use crseo::FromBuilder;
use gmt_dos_clients_io::optics::SegmentTipTilt;
use interface::{Data, Write};

use crate::OpticalModel;

use super::{SensorPropagation, WaveSensor, builders::SegmentGradientSensorBuilder};

#[derive(Debug, Clone)]
pub struct SegmentGradientSensor(pub(crate) WaveSensor);

impl Deref for SegmentGradientSensor {
    type Target = WaveSensor;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for SegmentGradientSensor {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Display for SegmentGradientSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "segment gradient sensor ({},{:?})",
            self.amplitude.len(),
            self.segment_gradient.as_ref().map(|s| s.len())
        )
    }
}

impl FromBuilder for SegmentGradientSensor {
    type ComponentBuilder = SegmentGradientSensorBuilder;
}

impl SensorPropagation for SegmentGradientSensor {
    fn propagate(&mut self, src: &mut crseo::Source) {
        <WaveSensor as SensorPropagation>::propagate(&mut self.0, src);
    }
}

impl Write<SegmentTipTilt> for OpticalModel<SegmentGradientSensor> {
    fn write(&mut self) -> Option<Data<SegmentTipTilt>> {
        self.sensor
            .as_ref()
            .unwrap()
            .segment_gradient
            .as_ref()
            .map(|sp| sp.clone().into())
    }
}
