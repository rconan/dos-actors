use std::fmt::Display;

use crate::OpticalModel;
use crseo::{Builder, CrseoError, FromBuilder};

use super::{builders::SensorBuilderProperty, SensorPropagation};

/// A sensor that is not
///
/// The sensor type that is used for an [OpticalModel] without a sensor
#[derive(Debug, Default, Clone)]
pub struct NoSensor;
impl SensorBuilderProperty for NoSensor {}

impl SensorPropagation for NoSensor {
    fn propagate(&mut self, _src: &mut crseo::Source) {}
}

impl Builder for NoSensor {
    type Component = NoSensor;
    fn build(self) -> std::result::Result<NoSensor, CrseoError> {
        Ok(NoSensor)
    }
}

impl FromBuilder for NoSensor {
    type ComponentBuilder = NoSensor;
}

impl Display for OpticalModel<NoSensor> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "- OPTICAL MODEL -")?;
        self.gmt.fmt(f)?;
        self.src.fmt(f)?;
        writeln!(f, "-----------------")?;
        Ok(())
    }
}
