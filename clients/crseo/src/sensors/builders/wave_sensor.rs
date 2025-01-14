use crseo::{gmt::GmtBuilder, source::SourceBuilder, Builder, CrseoError};
use interface::Update;

use crate::{
    sensors::{NoSensor, WaveSensor},
    OpticalModel, OpticalModelBuilder,
};

use super::SensorBuilderProperty;

/// [WaveSensor] builder
///
/// # Examples:
///
/// Build a [WaveSensor] with the default values for [OpticalModelBuilder] without sensor
///
/// ```
/// use gmt_dos_clients_crseo::sensors::WaveSensor;
/// use crseo::{Builder, FromBuilder};
///
/// let wave = WaveSensor::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Default, Clone)]
pub struct WaveSensorBuilder {
    pub(crate) omb: OpticalModelBuilder<NoSensor>,
    segment_piston: bool,
    segment_gradient: bool,
}

impl From<OpticalModelBuilder<NoSensor>> for WaveSensorBuilder {
    fn from(omb: OpticalModelBuilder<NoSensor>) -> Self {
        Self {
            omb,
            ..Default::default()
        }
    }
}

impl From<&OpticalModelBuilder<NoSensor>> for WaveSensorBuilder {
    fn from(omb: &OpticalModelBuilder<NoSensor>) -> Self {
        Self {
            omb: omb.clone(),
            ..Default::default()
        }
    }
}

impl WaveSensorBuilder {
    /// Sets the GMT builder
    pub fn gmt(mut self, gmt: GmtBuilder) -> Self {
        self.omb = self.omb.gmt(gmt);
        self
    }
    ///  Sets the source builder
    pub fn source(mut self, source: SourceBuilder) -> Self {
        self.omb = self.omb.source(source);
        self
    }
    /// Computes the segment piston from the wavefront
    pub fn with_segment_piston(mut self) -> Self {
        self.segment_piston = true;
        self
    }
    /// Computes the segment average gradient from the wavefront
    pub fn with_segment_gradient(mut self) -> Self {
        self.segment_gradient = true;
        self
    }
}

impl Builder for WaveSensorBuilder {
    type Component = WaveSensor;
    fn build(self) -> std::result::Result<Self::Component, CrseoError> {
        let Self {
            omb,
            segment_piston,
            segment_gradient,
        } = self;
        let mut optical_model: OpticalModel<NoSensor> = omb.build().unwrap();
        optical_model.update();
        let amplitude: Vec<_> = optical_model
            .src
            .amplitude()
            .into_iter()
            .map(|x| x as f64)
            .collect();
        let phase: Vec<_> = optical_model
            .src
            .phase()
            .iter()
            .map(|x| *x as f64)
            .collect();
        let n = phase.len();
        let reference = WaveSensor {
            amplitude,
            phase,
            reference: None,
            segment_piston: segment_piston.then(|| optical_model.src.segment_piston()),
            segment_gradient: segment_gradient.then(|| optical_model.src.segment_gradients()),
        };

        Ok(WaveSensor {
            reference: Some(Box::new(reference)),
            amplitude: vec![0f64; n],
            phase: vec![0f64; n],
            segment_piston: None,
            segment_gradient: None,
        })
    }
}

impl SensorBuilderProperty for WaveSensorBuilder {
    fn pupil_sampling(&self) -> Option<usize> {
        Some(self.omb.src.pupil_sampling.side())
    }
}
