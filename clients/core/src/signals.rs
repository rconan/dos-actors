use super::{Data, TimerMarker, UniqueIdentifier, Update, Write};
// use linya::{Bar, Progress};
use std::ops::Add;

#[cfg(feature = "noise")]
use rand_distr::{Distribution, Normal, NormalError};

/// Signal types
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub enum Signal {
    /// A constant signal
    Constant(f64),
    /// A sinusoidal signal
    Sinusoid {
        amplitude: f64,
        sampling_frequency_hz: f64,
        frequency_hz: f64,
        phase_s: f64,
    },
    /// A ramp of the form y=ax+b
    Ramp { a: f64, b: f64 },
    /// A sigmoid
    Sigmoid {
        amplitude: f64,
        sampling_frequency_hz: f64,
    },
    /// White noise
    #[cfg(feature = "noise")]
    #[cfg_attr(feature = "serde", serde(skip))]
    WhiteNoise(Normal<f64>),
    /// A symphony?
    Composite(Vec<Signal>),
}

impl From<f64> for Signal {
    fn from(val: f64) -> Self {
        Signal::Constant(val)
    }
}

#[cfg(feature = "noise")]
impl Signal {
    /// Create a white noise signal with a standard deviation equal to one
    pub fn white_noise() -> Result<Self, NormalError> {
        Ok(Signal::WhiteNoise(Normal::new(0f64, 1f64)?))
    }
    /// Sets white noise standard deviation
    pub fn std_dev(self, sigma: f64) -> Result<Self, NormalError> {
        if let Signal::WhiteNoise(noise) = self {
            Ok(Signal::WhiteNoise(Normal::new(noise.mean(), sigma)?))
        } else {
            Ok(self)
        }
    }
    /// Adds bias to white noise
    pub fn bias(self, bias: f64) -> Result<Self, NormalError> {
        if let Signal::WhiteNoise(noise) = self {
            Ok(Signal::WhiteNoise(Normal::new(bias, noise.std_dev())?))
        } else {
            Ok(self)
        }
    }
}
impl Signal {
    /// Returns the signal value at step `i`
    pub fn get(&self, i: usize) -> f64 {
        use Signal::*;
        match self {
            Constant(val) => *val,
            Sinusoid {
                amplitude,
                sampling_frequency_hz,
                frequency_hz,
                phase_s,
            } => {
                (2f64
                    * std::f64::consts::PI
                    * (phase_s + i as f64 * frequency_hz / sampling_frequency_hz))
                    .sin()
                    * amplitude
            }
            Ramp { a, b } => a * i as f64 + b,
            Sigmoid {
                amplitude,
                sampling_frequency_hz,
            } => {
                let u = i as f64 / sampling_frequency_hz - 0.75;
                let r = (1. + (-5. * u).exp()).recip();
                amplitude * r * r
            }
            #[cfg(feature = "noise")]
            WhiteNoise(noise) => noise.sample(&mut rand::thread_rng()),
            Composite(signals) => signals.iter().map(|signal| signal.get(i)).sum(),
        }
    }
}

/// Multiplex signals generator
#[derive(Debug, Clone)]
pub struct Signals {
    size: usize,
    pub signals: Vec<Signal>,
    pub step: usize,
    pub n_step: usize,
}
impl Signals {
    /// Create a signal generator with `n` channels for `n_step` iterations
    ///
    /// Each channel is set to 0 valued [Signal::Constant]
    pub fn new(n: usize, n_step: usize) -> Self {
        let signals: Vec<_> = vec![Signal::Constant(0f64); n];
        Self {
            size: n,
            signals,
            step: 0,
            n_step,
        }
    }
    // pub fn progress(&mut self) {
    //     self.progress_bar = Some(<T as Progress>::progress(
    //         "Signals",
    //         self.n_step - self.step,
    //     ));
    // }
    #[deprecated(note = "please use `channels` instead")]
    /// Sets the same [Signal] for all outputs
    pub fn signals(self, signal: Signal) -> Self {
        let signals = vec![signal.clone(); self.size];
        Self { signals, ..self }
    }
    #[deprecated(note = "please use `channel` instead")]
    /// Sets the [Signal] of output #`k`
    pub fn output_signal(self, k: usize, output_signal: Signal) -> Self {
        let mut signals = self.signals;
        signals[k] = output_signal;
        Self { signals, ..self }
    }
    /// Sets the same [Signal] for all outputs
    pub fn channels<S: Into<Signal>>(self, signal: S) -> Self {
        let signals = vec![signal.into().clone(); self.size];
        Self { signals, ..self }
    }
    /// Sets the [Signal] of output #`k`
    pub fn channel<S: Into<Signal>>(self, k: usize, output_signal: S) -> Self {
        let mut signals = self.signals;
        signals[k] = output_signal.into();
        Self { signals, ..self }
    }
}

impl From<(Vec<f64>, usize)> for Signals {
    fn from((data, n_step): (Vec<f64>, usize)) -> Self {
        let n = data.len();
        data.into_iter()
            .enumerate()
            .fold(Signals::new(n, n_step), |s, (i, v)| {
                s.channel(i, Signal::Constant(v))
            })
    }
}
impl From<(&[f64], usize)> for Signals {
    fn from((data, n_step): (&[f64], usize)) -> Self {
        let n = data.len();
        data.iter()
            .enumerate()
            .fold(Signals::new(n, n_step), |s, (i, v)| {
                s.channel(i, Signal::Constant(*v))
            })
    }
}

impl Add for Signal {
    type Output = Signal;

    fn add(self, rhs: Self) -> Self::Output {
        if let Signal::Composite(mut signals) = self {
            signals.push(rhs);
            Signal::Composite(signals)
        } else {
            Signal::Composite(vec![self, rhs])
        }
    }
}
impl TimerMarker for Signals {}
impl Update for Signals {}
impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for Signals {
    fn write(&mut self) -> Option<Data<U>> {
        // log::debug!("write {:?}", self.size);
        if self.step < self.n_step {
            let i = self.step;
            let data = self.signals.iter().map(|signal| signal.get(i)).collect();
            self.step += 1;
            Some(Data::new(data))
        } else {
            None
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum SignalsError {
    #[error("Two many signal channels, should be only 1")]
    OneSignal,
}
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct OneSignal {
    pub signal: Signal,
    pub step: usize,
    pub n_step: usize,
}
impl TryFrom<Signals> for OneSignal {
    type Error = SignalsError;
    fn try_from(mut signals: Signals) -> Result<Self, Self::Error> {
        if signals.signals.len() > 1 {
            Err(SignalsError::OneSignal)
        } else {
            Ok(OneSignal {
                signal: signals.signals.remove(0),
                step: signals.step,
                n_step: signals.n_step,
            })
        }
    }
}
impl Update for OneSignal {}
impl<U: UniqueIdentifier<DataType = f64>> Write<U> for OneSignal {
    fn write(&mut self) -> Option<Data<U>> {
        if self.step < self.n_step {
            let i = self.step;
            let data = self.signal.get(i);
            self.step += 1;
            Some(Data::new(data))
        } else {
            None
        }
    }
}
