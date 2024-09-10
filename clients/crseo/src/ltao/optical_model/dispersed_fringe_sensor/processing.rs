use std::{f32::consts::PI, iter::repeat};

use crseo::imaging;
use gmt_dos_clients_io::optics::{
    dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
    Dev,
};
use interface::{Data, Read, Update, Write};
use serde::{Deserialize, Serialize};

use super::DispersedFringeSensor;

const O: [f32; 12] = [
    0.,
    -PI / 3.,
    0.,
    -PI / 3.,
    PI / 3.,
    -PI / 3.,
    PI / 3.,
    -PI / 3.,
    PI / 3.,
    0.,
    PI / 3.,
    0.,
];

#[derive(Debug, Clone, Serialize)]
pub struct Fftlet {
    x: Vec<f32>,
    y: Vec<f32>,
    image: Vec<f32>,
}

impl Fftlet {
    pub fn intercept(&self) -> f64 {
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
        sy as f64 / s as f64
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DispersedFringeSensorProcessing<const C: usize, const F: usize> {
    data: Vec<Vec<f32>>,
    n: usize,
    threshold: Option<f32>,
    mask_radius: Option<f32>,
    pub intercept: Vec<f64>,
    reference: Option<Vec<f64>>,
}

impl<const C: usize, const F: usize> DispersedFringeSensorProcessing<C, F> {
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

impl<const C: usize, const F: usize> DispersedFringeSensorProcessing<C, F> {
    pub fn set_reference(&mut self, dfs: &DispersedFringeSensorProcessing<C, F>) -> &mut Self {
        self.reference = Some(dfs.intercept.clone());
        self
    }
    pub fn threshold(self, t: f64) -> Self {
        Self {
            threshold: Some(t as f32),
            ..self
        }
    }
    pub fn mask_radius(self, r: f64) -> Self {
        Self {
            mask_radius: Some(r as f32),
            ..self
        }
    }
}

impl<const C: usize, const F: usize> From<&mut DispersedFringeSensor<C, F>>
    for DispersedFringeSensorProcessing<C, F>
{
    fn from(sps: &mut DispersedFringeSensor<C, F>) -> Self {
        let mut frame = sps.fft_frame();
        let n = frame.resolution;
        let q = n / 4;
        let data = Vec::<f32>::from(&mut frame);
        let mut chop_data = vec![];
        for i in 0..4 {
            for j in 0..3 {
                chop_data.push(
                    data.chunks(n)
                        .skip(i * q)
                        .take(q)
                        .flat_map(|data| {
                            data.iter().skip(j * q).take(q).cloned().collect::<Vec<_>>()
                        })
                        .collect::<Vec<_>>(),
                )
            }
        }
        Self {
            data: chop_data,
            n: q,
            threshold: Some(0.2),
            mask_radius: Some(0.05),
            ..Self::new()
        }
    }
}

impl<const C: usize, const F: usize> DispersedFringeSensorProcessing<C, F> {
    pub fn process(&mut self, frame: &imaging::Frame) -> &mut Self {
        let n = frame.resolution;
        let q = n / 4;
        let data = Vec::<f32>::from(frame);
        let mut chop_data = vec![];
        for i in 0..4 {
            for j in 0..3 {
                chop_data.push(
                    data.chunks(n)
                        .skip(i * q)
                        .take(q)
                        .flat_map(|data| {
                            data.iter().skip(j * q).take(q).cloned().collect::<Vec<_>>()
                        })
                        .collect::<Vec<_>>(),
                )
            }
        }
        self.data = chop_data;
        self.n = q;
        self
    }
    pub fn flux(&self) -> Vec<f32> {
        self.data.iter().map(|data| data.iter().sum()).collect()
    }

    pub fn xy(&self, i: usize) -> impl Iterator<Item = (f32, f32)> {
        let n = self.n;

        let x = (0..n)
            .flat_map(move |i| repeat(i).take(n))
            .map(move |x| (x as f32 - 0.5 * (n - 1) as f32) / (n - 1) as f32);
        let y = (0..n)
            .cycle()
            .take(n * n)
            .map(move |x| (x as f32 - 0.5 * (n - 1) as f32) / (n - 1) as f32);

        x.zip(y).map(move |(x, y)| {
            let (so, co) = O[i].sin_cos();
            (co * x - so * y, so * x + co * y)
        })
    }
    pub fn fftlet(&self, i: usize, radius: Option<f32>, threshold: Option<f32>) -> Fftlet {
        // let flux = self.flux()[i];
        let ((x, y), image): ((Vec<f32>, Vec<f32>), Vec<f32>) = if let Some(r) = radius {
            self.xy(i)
                .zip(self.data[i].iter())
                .filter_map(|((x, y), data)| {
                    if x.hypot(y) > r {
                        Some(((x, y), data))
                    } else {
                        None
                    }
                })
                .unzip()
        } else {
            (
                self.xy(i).unzip(),
                self.data[i].iter().map(|i| *i).collect(),
            )
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
    pub fn intercept(&mut self) -> &mut Self {
        self.intercept = (0..12)
            .map(|i| {
                let fftlet = self.fftlet(i, self.mask_radius, self.threshold);
                fftlet.intercept()
            })
            .collect();
        if let Some(r) = &self.reference {
            self.intercept
                .iter_mut()
                .zip(r.iter())
                .for_each(|(i, r)| *i -= r);
        }
        self
    }
}

impl<const C: usize, const F: usize> Update for DispersedFringeSensorProcessing<C, F> {
    fn update(&mut self) {
        self.intercept();
    }
}

impl<const C: usize, const F: usize> Read<DfsFftFrame<Dev>>
    for DispersedFringeSensorProcessing<C, F>
{
    fn read(&mut self, data: Data<DfsFftFrame<Dev>>) {
        self.process(data.into_arc().as_ref());
    }
}

impl<const C: usize, const F: usize> Write<Intercepts> for DispersedFringeSensorProcessing<C, F> {
    fn write(&mut self) -> Option<Data<Intercepts>> {
        Some(self.intercept.clone().into())
    }
}
