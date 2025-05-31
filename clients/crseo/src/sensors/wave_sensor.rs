use std::fmt::Display;

use crate::OpticalModel;
use crseo::{FromBuilder, Source};
use gmt_dos_clients_io::optics::{SegmentPiston, SegmentTipTilt, Wavefront, WfeRms};
use interface::{Data, Size, Update, Write};

use super::{NoSensor, SensorPropagation, builders::WaveSensorBuilder};

/// Complex amplitude sensor
///
/// A sensor that records the amplitude and phase of a [crseo Source](https://docs.rs/crseo/latest/crseo/source) wavefront.
///
/// The phase of the wavefront is referenced with respect to the phase that
/// corresponds to an ideally collimated GMT.
///
/// # Examples:
///
/// Build a [WaveSensor] with the default [WaveSensorBuilder]
/// ```
/// use gmt_dos_clients_crseo::sensors::WaveSensor;
/// use crseo::{Builder, FromBuilder};
///
/// let wave = WaveSensor::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
#[derive(Debug, Default, Clone)]
pub struct WaveSensor {
    pub(crate) reference: Option<Box<WaveSensor>>,
    pub(crate) amplitude: Vec<f64>,
    pub(crate) phase: Vec<f64>,
    pub(crate) segment_piston: Option<Vec<f64>>,
    pub(crate) segment_gradient: Option<Vec<f64>>,
    pub(crate) n_src: usize,
}

impl Display for WaveSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "wave sensor ({})", self.amplitude.len())
    }
}

impl Display for OpticalModel<WaveSensor> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "- OPTICAL MODEL -")?;
        self.gmt.fmt(f)?;
        self.src.fmt(f)?;
        if let Some(atm) = &self.atm {
            atm.fmt(f)?;
        }
        self.sensor.as_ref().unwrap().fmt(f)?;

        writeln!(f, "-----------------")?;
        Ok(())
    }
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
            segment_piston: None,
            segment_gradient: None,
            n_src: optical_model.src.size as usize,
        };
        Self {
            reference: Some(Box::new(reference)),
            amplitude: vec![0f64; n],
            phase: vec![0f64; n],
            segment_piston: None,
            segment_gradient: None,
            n_src: optical_model.src.size as usize,
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
impl<const E: i32> Write<WfeRms<E>> for OpticalModel<WaveSensor> {
    fn write(&mut self) -> Option<Data<WfeRms<E>>> {
        let sensor = self.sensor.as_ref().unwrap();
        let mut iter = sensor
            .phase
            .iter()
            .zip(sensor.amplitude.iter().map(|&a| a > 0.));
        let mut wfe_rms = Vec::<f64>::new();
        let n_px = sensor.amplitude.len() / sensor.n_src;
        for _ in 0..sensor.n_src {
            let (s, ss, n) = iter
                .by_ref()
                .take(n_px)
                .filter_map(|(&p, m)| m.then_some(p))
                .fold((0f64, 0f64, 0usize), |(s, ss, i), x| {
                    (s + x, ss + x * x, i + 1)
                });
            let n = n as f64;
            wfe_rms.push(((ss - s * s / n) / n).sqrt() * 10_f64.powi(-E));
        }
        Some(wfe_rms.into())
    }
}
impl Write<SegmentPiston> for OpticalModel<WaveSensor> {
    fn write(&mut self) -> Option<Data<SegmentPiston>> {
        self.sensor
            .as_ref()
            .unwrap()
            .segment_piston
            .as_ref()
            .map(|sp| sp.clone().into())
    }
}

impl Write<SegmentTipTilt> for OpticalModel<WaveSensor> {
    fn write(&mut self) -> Option<Data<SegmentTipTilt>> {
        self.sensor
            .as_ref()
            .unwrap()
            .segment_gradient
            .as_ref()
            .map(|sp| sp.clone().into())
    }
}

impl SensorPropagation for WaveSensor {
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
            if let Some(segment_piston) = reference.segment_piston.as_ref() {
                self.segment_piston = Some(
                    src.segment_piston()
                        .into_iter()
                        .zip(segment_piston)
                        .map(|(x, y)| x - y)
                        .collect(),
                )
            }
            if let Some(segment_gradient) = reference.segment_gradient.as_ref() {
                self.segment_gradient = Some(
                    src.segment_gradients()
                        .into_iter()
                        .zip(segment_gradient)
                        .map(|(x, y)| x - y)
                        .collect(),
                )
            }
        }
    }
    // fn time_propagate(&mut self, _secs: f64, _src: &mut Source) {
    //     unimplemented!()
    // }
}

impl FromBuilder for WaveSensor {
    type ComponentBuilder = WaveSensorBuilder;
}
