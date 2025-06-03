use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use crseo::FromBuilder;
use gmt_dos_clients_io::optics::SegmentPiston;
use interface::{Data, Write};

use crate::OpticalModel;

use super::{SensorPropagation, WaveSensor, builders::SegmentPistonSensorBuilder};

#[derive(Debug, Clone)]
pub struct SegmentPistonSensor(pub(crate) WaveSensor);

impl Deref for SegmentPistonSensor {
    type Target = WaveSensor;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for SegmentPistonSensor {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Display for SegmentPistonSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "segment piston sensor ({},{:?})",
            self.amplitude.len(),
            self.segment_piston.as_ref().map(|s| s.len())
        )
    }
}

impl FromBuilder for SegmentPistonSensor {
    type ComponentBuilder = SegmentPistonSensorBuilder;
}

impl SensorPropagation for SegmentPistonSensor {
    fn propagate(&mut self, src: &mut crseo::Source) {
        <WaveSensor as SensorPropagation>::propagate(&mut self.0, src);
    }
}

impl Write<SegmentPiston> for OpticalModel<SegmentPistonSensor> {
    fn write(&mut self) -> Option<Data<SegmentPiston>> {
        self.sensor
            .as_ref()
            .unwrap()
            .segment_piston
            .as_ref()
            .map(|sp| sp.clone().into())
    }
}
