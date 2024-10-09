use std::{f32::consts::PI, iter::repeat};

use crseo::imaging;
use gmt_dos_clients_io::optics::{
    dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
    Dev,
};
use interface::{Data, Read, Update, Write};
use serde::{Deserialize, Serialize};

use crate::sensors::DispersedFringeSensor;

const O: [f32; 12] = [
    0.,
    -PI / 3.,
    PI / 3.,
    0.,
    -PI / 3.,
    PI / 3.,
    -PI / 3.,
    PI / 3.,
    0.,
    -PI / 3.,
    PI / 3.,
    0.,
];

/// [Dispersed Fringe Sensor](DispersedFringeSensor) fftlet
#[derive(Debug, Clone)]
pub struct Fftlet {
    x: Vec<f32>,
    y: Vec<f32>,
    image: Vec<f32>,
}

impl Fftlet {
    /// Returns the intercept
    pub fn intercept(&self) -> Option<f64> {
        let (s, sy) = self
            .x
            .iter()
            .zip(self.y.iter())
            .zip(self.image.iter())
            .fold((0f32, 0f32), |(mut s, mut sy), ((x, y), i)| {
                s += i;
                sy += i * y * x.signum();
                (s, sy)
            });
        if s > 0. {
            Some(sy as f64 / s as f64)
        } else {
            None
        }
    }
}

/// [Dispersed Fringe Sensor](DispersedFringeSensor) processing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DispersedFringeSensorProcessing {
    data: Vec<Vec<f32>>,
    n: usize,
    threshold: Option<f32>,
    mask_radius: Option<f32>,
    pub intercept: Vec<f64>,
    reference: Option<Vec<f64>>,
}

impl DispersedFringeSensorProcessing {
    /// Creates a new instance
    pub fn new() -> Self {
        Self {
            data: vec![],
            n: 0,
            threshold: Some(0.2),
            mask_radius: Some(0.05),
            intercept: vec![],
            reference: None,
        }
    }
}

impl DispersedFringeSensorProcessing {
    /// Sets the reference intercepts
    pub fn set_reference(&mut self, dfs: &DispersedFringeSensorProcessing) -> &mut Self {
        self.reference = Some(dfs.intercept.clone());
        self
    }
    /// Sets the frame relative theshold
    pub fn threshold(self, t: f64) -> Self {
        Self {
            threshold: Some(t as f32),
            ..self
        }
    }
    /// Sets the radius of the imagelet mask
    pub fn mask_radius(self, r: f64) -> Self {
        Self {
            mask_radius: Some(r as f32),
            ..self
        }
    }
}

impl<const C: usize, const F: usize> From<&mut DispersedFringeSensor<C, F>>
    for DispersedFringeSensorProcessing
{
    fn from(sps: &mut DispersedFringeSensor<C, F>) -> Self {
        let frame = sps.fft_frame();
        let mut this = DispersedFringeSensorProcessing::new();
        this.process(&frame);
        this
    }
}

impl<const C: usize, const F: usize> From<&mut crate::OpticalModel<DispersedFringeSensor<C, F>>>
    for DispersedFringeSensorProcessing
{
    fn from(om: &mut crate::OpticalModel<DispersedFringeSensor<C, F>>) -> Self {
        Self::from(om.sensor_mut().unwrap())
    }
}

impl DispersedFringeSensorProcessing {
    /// Process a frame
    pub fn process(&mut self, frame: &imaging::Frame) -> &mut Self {
        let n = frame.resolution;
        let q = frame.n_px_camera;
        let l = frame.resolution / frame.n_px_camera;
        let n_fftlet = 12 * ((l * l) / 12);
        let data = Vec::<f32>::from(frame);
        let mut chop_data = vec![];
        let mut count = 0;
        'outer: for j in 0..l {
            for i in 0..l {
                chop_data.push(
                    data.chunks(n)
                        .skip(i * q)
                        .take(q)
                        .flat_map(|data| {
                            data.iter().skip(j * q).take(q).cloned().collect::<Vec<_>>()
                        })
                        .collect::<Vec<_>>(),
                );
                count += 1;
                if count == n_fftlet {
                    break 'outer;
                }
            }
        }
        self.data = chop_data;
        self.n = q;
        self
    }
    /// Returns the flux
    pub fn flux(&self) -> Vec<f32> {
        self.data.iter().map(|data| data.iter().sum()).collect()
    }
    /// Return an iterator over the FFTlet `(x,y)` coordinates
    pub fn xy(&self, o: f32) -> impl Iterator<Item = (f32, f32)> {
        let n = self.n;

        let x = (0..n)
            .flat_map(move |i| repeat(i).take(n))
            .map(move |x| (x as f32 - 0.5 * (n - 1) as f32) / (n - 1) as f32);
        let y = (0..n)
            .cycle()
            .take(n * n)
            .map(move |x| (x as f32 - 0.5 * (n - 1) as f32) / (n - 1) as f32);

        x.zip(y).map(move |(x, y)| {
            let (so, co) = o.sin_cos();
            (co * x - so * y, so * x + co * y)
        })
    }
    /// Returns an FFTlet
    pub fn fftlet(
        &self,
        angle: f32,
        data: &[f32],
        radius: Option<f32>,
        threshold: Option<f32>,
    ) -> Fftlet {
        // let flux = self.flux()[i];
        let ((x, y), image): ((Vec<f32>, Vec<f32>), Vec<f32>) = if let Some(r) = radius {
            self.xy(angle)
                .zip(data.iter())
                .filter_map(|((x, y), data)| {
                    if x.hypot(y) > r {
                        Some(((x, y), data))
                    } else {
                        None
                    }
                })
                .unzip()
        } else {
            (self.xy(angle).unzip(), data.iter().map(|i| *i).collect())
        };
        if let Some(t) = threshold {
            let max_intensity = image
                .iter()
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap()
                * t;
            let ((x, y), image): ((Vec<f32>, Vec<f32>), Vec<f32>) = x
                .into_iter()
                .zip(y.into_iter())
                .zip(image.into_iter())
                .filter_map(|((x, y), image)| {
                    if image > max_intensity {
                        Some(((x, y), image))
                    } else {
                        None
                    }
                })
                .unzip();
            Fftlet { x, y, image }
        } else {
            Fftlet { x, y, image }
        }
    }
    /// Computes the intercepts
    pub fn intercept(&mut self) -> &mut Self {
        let intercept: Option<Vec<f64>> = self
            .data
            .iter()
            .zip(O.iter().cycle())
            .map(|(data, &angle)| {
                let fftlet = self.fftlet(angle, data.as_slice(), self.mask_radius, self.threshold);
                fftlet.intercept()
            })
            .collect();
        if let Some(intercept) = intercept {
            self.intercept = intercept;
            // Subtract reference intercept
            if let Some(r) = &self.reference {
                self.intercept
                    .iter_mut()
                    .zip(r.iter())
                    .for_each(|(i, r)| *i -= r);
            }
        } else {
            self.intercept = vec![0f64; self.data.len()];
        }
        self
    }
    /// Returns the FFTlets with optional radius and threshold filters
    pub fn fftlets(&self, radius: Option<f32>, threshold: Option<f32>) -> Vec<Fftlet> {
        self.data
            .iter()
            .zip(O.iter().cycle())
            .map(|(data, &angle)| self.fftlet(angle, data.as_slice(), radius, threshold))
            .collect()
    }
}

impl Update for DispersedFringeSensorProcessing {
    fn update(&mut self) {
        self.intercept();
    }
}

impl Read<DfsFftFrame<Dev>> for DispersedFringeSensorProcessing {
    fn read(&mut self, data: Data<DfsFftFrame<Dev>>) {
        self.process(data.into_arc().as_ref());
    }
}

impl Write<Intercepts> for DispersedFringeSensorProcessing {
    fn write(&mut self) -> Option<Data<Intercepts>> {
        Some(self.intercept.clone().into())
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{FromBuilder, Source};

    use crate::OpticalModel;

    use super::*;

    #[test]
    fn process() -> Result<(), Box<dyn Error>> {
        let mut om = OpticalModel::<DispersedFringeSensor<1, 1>>::builder()
            .source(Source::builder().size(2))
            .sensor(DispersedFringeSensor::<1, 1>::builder())
            .build()?;
        om.update();

        let mut processing = DispersedFringeSensorProcessing::from(&mut om);

        processing.intercept();
        dbg!(processing.intercept.len());

        serde_pickle::to_writer(
            &mut std::fs::File::create("processing.pkl")?,
            &processing,
            Default::default(),
        )?;

        Ok(())
    }

    #[test]
    fn fftlets() -> Result<(), Box<dyn Error>> {
        let mut om = OpticalModel::<DispersedFringeSensor<1, 1>>::builder()
            .source(Source::builder().size(2))
            .sensor(DispersedFringeSensor::<1, 1>::builder())
            .build()?;
        om.update();

        let processing = DispersedFringeSensorProcessing::from(&mut om);

        serde_pickle::to_writer(
            &mut std::fs::File::create("fftlets.pkl")?,
            &processing.fftlets(None, None),
            Default::default(),
        )?;

        Ok(())
    }
}
