use crate::ltao::{SensorBuilderProperty, SensorProperty};
use crate::{NoSensor, OpticalModel, OpticalModelBuilder};
use crseo::{Builder, CrseoError, FromBuilder, Propagation, Source};
use gmt_dos_clients_io::optics::Wavefront;
use interface::{Data, Size, Update, Write};

#[derive(Debug, Default, Clone)]
pub struct WaveSensorBuilder(pub OpticalModelBuilder<NoSensor>);

#[derive(Debug, Default)]
pub struct WaveSensor {
    reference: Option<Box<WaveSensor>>,
    amplitude: Vec<f64>,
    phase: Vec<f64>,
}

impl WaveSensor {
    pub fn phase(&self) -> &[f64] {
        self.phase.as_slice()
    }
    pub fn amplitude(&self) -> &[f64] {
        self.amplitude.as_slice()
    }
}

impl From<OpticalModel<NoSensor>> for WaveSensor {
    fn from(mut optical_model: OpticalModel<NoSensor>) -> Self {
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
        };
        Self {
            reference: Some(Box::new(reference)),
            amplitude: vec![0f64; n],
            phase: vec![0f64; n],
        }
    }
}

impl Write<Wavefront> for OpticalModel<WaveSensor> {
    fn write(&mut self) -> Option<Data<Wavefront>> {
        Some(self.sensor.as_ref()?.phase.clone().into())
    }
}
impl Size<Wavefront> for OpticalModel<WaveSensor> {
    fn len(&self) -> usize {
        self.sensor.as_ref().unwrap().phase.len()
    }
}

impl SensorBuilderProperty for WaveSensorBuilder {
    fn pupil_sampling(&self) -> usize {
        self.0.src.pupil_sampling.side()
    }
}
impl SensorProperty for WaveSensor {
    fn reset(&mut self) {
        unimplemented!()
    }
}

impl Propagation for WaveSensor {
    fn propagate(&mut self, src: &mut Source) {
        let iter = self.amplitude.iter_mut().zip(&mut self.phase);
        let src_iter = src.amplitude().into_iter().zip(src.phase().iter());
        src_iter.zip(iter).for_each(|((src_a, src_p), (a, p))| {
            *a = src_a as f64;
            *p = *src_p as f64;
        });
        if let Some(reference) = self.reference.as_ref() {
            let iter = self.amplitude.iter_mut().zip(&mut self.phase);
            let ref_iter = reference.amplitude.iter().zip(reference.phase.iter());
            ref_iter.zip(iter).for_each(|((ref_a, ref_p), (a, p))| {
                if *ref_a > 0. && *a > 0. {
                    *p -= *ref_p
                } else {
                    *a = 0.;
                    *p = 0.;
                }
            });
        }
    }
    fn time_propagate(&mut self, _secs: f64, _src: &mut Source) {
        unimplemented!()
    }
}

impl Builder for WaveSensorBuilder {
    type Component = WaveSensor;
    fn build(self) -> std::result::Result<Self::Component, CrseoError> {
        let Self(omb) = self;
        let om: OpticalModel<NoSensor> = omb.build().unwrap();
        Ok(om.into())
    }
}

impl FromBuilder for WaveSensor {
    type ComponentBuilder = WaveSensorBuilder;
}
