use epaint::{Color32, ColorImage};
use serde::{Deserialize, Serialize};

use crate::payload::Payload;

#[derive(Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub(crate) enum SignalData {
    Signal {
        tag: String,
        tau: f64,
        points: Vec<[f64; 2]>,
    },
    Signals(Vec<SignalData>),
    Image {
        tag: String,
        time: f64,
        size: [usize; 2],
        image: Option<ColorImage>,
        quantiles: Option<Quantiles>,
    },
}

impl From<&Payload> for SignalData {
    fn from(payload: &Payload) -> Self {
        match payload {
            Payload::Signal { tag, tau, .. } => Self::Signal {
                tag: tag.clone(),
                tau: *tau,
                points: vec![[0f64; 2]],
            },
            Payload::Image { tag, size, .. } => Self::Image {
                tag: tag.clone(),
                time: 0f64,
                size: *size,
                image: None,
                quantiles: None,
            },
            Payload::Signals { tag, tau, value } => Self::Signals(
                value
                    .iter()
                    .map(|_| Self::Signal {
                        tag: tag.clone(),
                        tau: *tau,
                        points: vec![[0f64; 2]],
                    })
                    .collect(),
            ),
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Quantiles {
    pub minimum: f64,
    pub lower_whisker: f64,
    pub quartile1: f64,
    pub median: f64,
    pub quartile3: f64,
    pub upper_whisker: f64,
    pub maximum: f64,
}
impl Quantiles {
    pub fn new(data: &[f64]) -> Self {
        let mut sample = data.to_vec();
        sample.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let minimum = sample[0];
        let maximum = *sample.last().unwrap();
        let quartile1 = Self::quartile(0.25, &sample);
        let median = Self::quartile(0.5, &sample);
        let quartile3 = Self::quartile(0.75, &sample);
        let iqr = quartile3 - quartile1;
        let mut lower_whisker = quartile1 - 1.5 * iqr;
        let mut upper_whisker = quartile3 + 1.5 * iqr;
        if minimum > lower_whisker {
            lower_whisker = minimum;
        }
        if maximum < upper_whisker {
            upper_whisker = maximum;
        }
        Self {
            minimum,
            lower_whisker,
            quartile1,
            median,
            quartile3,
            upper_whisker,
            maximum,
        }
    }
    pub fn quartile(p: f64, sample: &[f64]) -> f64 {
        let n = (1 + sample.len()) as f64;
        let k = (p * n).floor();
        let a = (p * n) - k;
        let k = k as usize;
        sample[k] + a * (sample[k + 1] - sample[k])
    }
}

impl SignalData {
    pub fn add_payload(&mut self, payload: &Payload) {
        match (payload, self) {
            (Payload::Signal { value, .. }, SignalData::Signal { tau, points, .. }) => {
                let &[x, _y] = points.last().unwrap();
                points.push([x, *value]);
                points.push([x + *tau, *value]);
            }
            (Payload::Signals { value, .. }, SignalData::Signals(signals)) => {
                assert_eq!(value.len(), signals.len());
                value
                    .into_iter()
                    .zip(signals.into_iter())
                    .for_each(|(value, signal)| {
                        if let SignalData::Signal { tau, points, .. } = signal {
                            let &[x, _y] = points.last().unwrap();
                            points.push([x, *value]);
                            points.push([x + *tau, *value]);
                        }
                    });
            }
            (
                Payload::Image {
                    tau,
                    size,
                    pixels,
                    minmax,
                    mask,
                    ..
                },
                SignalData::Image {
                    time,
                    image: texture,
                    quantiles,
                    ..
                },
            ) => {
                let mut img = ColorImage::new(*size, Color32::TRANSPARENT);
                let colormap = colorous::CIVIDIS;
                match mask {
                    Some(mask) => {
                        let px_quantiles = Quantiles::new(pixels);
                        let Quantiles {
                            minimum: min,
                            maximum: max,
                            ..
                        } = px_quantiles;
                        let range = max - min;
                        mask.iter()
                            .zip(img.pixels.iter_mut())
                            .filter(|(&m, _)| m)
                            .zip(pixels)
                            .map(|((_, u), v)| (u, (v - min) / range))
                            .map(|(u, t)| (u, colormap.eval_continuous(t)))
                            .for_each(|(px, rgb)| {
                                let colorous::Color { r, g, b } = rgb;
                                *px = Color32::from_rgb(r, g, b);
                            });
                        *quantiles = Some(px_quantiles);
                    }
                    None => {
                        let (min, max) = if let Some((min, max)) = minmax {
                            (*min, *max)
                        } else {
                            (payload.min(), payload.max())
                        };
                        let range = max - min;
                        pixels
                            .iter()
                            .map(|v| (v - min) / range)
                            .map(|t| colormap.eval_continuous(t))
                            .zip(img.pixels.iter_mut())
                            .for_each(|(rgb, px)| {
                                let colorous::Color { r, g, b } = rgb;
                                *px = Color32::from_rgb(r, g, b);
                            });
                    }
                };
                *time += tau;
                texture.replace(img);
            }
            _ => todo!(),
        };
    }
}

unsafe impl Send for SignalData {}
unsafe impl Sync for SignalData {}
