/*!
# [Actor](crate::Actor)s clients

The module holds the implementation of the different clients that can be assigned to [Actor]s.

Any structure can become a client to an Actor if it implements the [Update] trait with either or both [Read] and [Write] traits.

# Example

## Logging

A simple logger with a single entry:
```
use dos_actors::prelude::*;
let logging = Logging::<f64>::default();
```
A logger with 2 entries and pre-allocated with 1000 elements:
```
use dos_actors::prelude::*;
let logging = Logging::<f64>::default().n_entry(2).capacity(1_000);
```
## Signals

A constant signal for 100 steps
```
use dos_actors::prelude::*;
let signal = Signals::new(1, 100).signals(Signal::Constant(3.14));
```

A 2 outputs signal made of a constant and a sinusoid for 100 steps
```
use dos_actors::prelude::*;
let signal = Signals::new(2, 100)
               .output_signal(0, Signal::Constant(3.14))
               .output_signal(1, Signal::Sinusoid{
                                        amplitude: 1f64,
                                        sampling_frequency_hz: 1000f64,
                                        frequency_hz: 20f64,
                                        phase_s: 0f64
               });
```
## Rate transitionner

A sample-and-hold rate transition for a named output/input pair sampling a [Vec]
```
use dos_actors::prelude::*;
#[derive(UID)]
enum MyIO {};
let sampler = Sampler::<Vec<f64>, MyIO>::default();
```

[Actor]: crate::actor
*/

#[cfg(feature = "windloads")]
pub mod windloads;

#[cfg(feature = "fem")]
pub mod fem;

#[cfg(feature = "mount-ctrl")]
pub mod mount;

#[cfg(feature = "m1-ctrl")]
pub mod m1;

#[cfg(feature = "apache-arrow")]
pub mod arrow_client;

#[cfg(feature = "fsm")]
pub mod fsm;

#[cfg(feature = "crseo")]
pub mod ceo;

#[cfg(feature = "lom")]
pub mod lom;

#[cfg(feature = "dta")]
pub mod dta;

pub mod gmt_state;

#[cfg(feature = "ceo")]
mod dome_seeing;

use crate::{
    io::{Data, Read, Write},
    UniqueIdentifier, Update,
};
use linya::{Bar, Progress};
use std::{
    any::type_name,
    fmt::Display,
    marker::PhantomData,
    mem::take,
    ops::{Add, Mul, Sub, SubAssign},
    sync::{Arc, Mutex},
};

mod signals;
#[doc(inline)]
pub use signals::{Signal, Signals};

struct ProgressBar {
    progress: Arc<Mutex<Progress>>,
    bar: Bar,
}

/// Simple digital timer
pub struct Timer {
    tick: usize,
    progress_bar: Option<ProgressBar>,
}
impl Timer {
    /// Initializes the timer based on the duration in # of samples
    pub fn new(duration: usize) -> Self {
        Self {
            tick: 1 + duration,
            progress_bar: None,
        }
    }
    pub fn progress(self) -> Self {
        let mut progress = Progress::new();
        let bar: Bar = progress.bar(self.tick, "Timer:");
        Self {
            progress_bar: Some(ProgressBar {
                progress: Arc::new(Mutex::new(progress)),
                bar,
            }),
            ..self
        }
    }
    pub fn progress_with(self, progress: Arc<Mutex<Progress>>) -> Self {
        let bar: Bar = progress.lock().unwrap().bar(self.tick, "Timer:");
        Self {
            progress_bar: Some(ProgressBar { progress, bar }),
            ..self
        }
    }
}
impl Update for Timer {
    fn update(&mut self) {
        if let Some(pb) = self.progress_bar.as_mut() {
            pb.progress.lock().unwrap().inc_and_draw(&pb.bar, 1)
        }
        self.tick -= 1;
    }
}
pub enum Tick {}
pub type Void = ();
impl UniqueIdentifier for Tick {
    type Data = Void;
}
impl Write<Void, Tick> for Timer {
    fn write(&mut self) -> Option<Arc<Data<Tick>>> {
        if self.tick > 0 {
            Some(Arc::new(Data::new(())))
        } else {
            None
        }
    }
}
pub(crate) trait TimerMarker {}
impl<T: TimerMarker> Read<Void, Tick> for T {
    fn read(&mut self, _: Arc<Data<Tick>>) {}
}

/// Simple data logging
///
/// Accumulates all the inputs in a single [Vec]
#[derive(Debug)]
pub struct Logging<T> {
    data: Vec<T>,
    n_sample: usize,
    n_entry: usize,
}

impl<T> std::ops::Deref for Logging<T> {
    type Target = Vec<T>;
    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl<T> Default for Logging<T> {
    fn default() -> Self {
        Self {
            n_entry: 1,
            data: Vec::new(),
            n_sample: 0,
        }
    }
}
impl<T> Logging<T> {
    /// Sets the # of entries to be logged (default: 1)
    pub fn n_entry(self, n_entry: usize) -> Self {
        Self { n_entry, ..self }
    }
    /// Pre-allocates the size of the vector holding the data
    pub fn capacity(self, capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
            ..self
        }
    }
    /// Returns the # of time samples
    pub fn len(&self) -> usize {
        self.n_sample / self.n_entry
    }
    /// Returns the sum of the entry sizes
    pub fn n_data(&self) -> usize {
        self.data.len() / self.len()
    }
    /// Checks if the logger is empty
    pub fn is_empty(&self) -> bool {
        self.n_sample == 0
    }
    /// Returns data chunks the size of the entries
    pub fn chunks(&self) -> impl Iterator<Item = &[T]> {
        self.data.chunks(self.n_data())
    }
}

impl<T> Display for Logging<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Logging: ({}x{})={}",
            self.n_data(),
            self.len(),
            self.data.len()
        )
    }
}

impl<T> Update for Logging<T> {}
impl<T: Clone, U: UniqueIdentifier<Data = Vec<T>>> Read<Vec<T>, U> for Logging<T> {
    fn read(&mut self, data: Arc<Data<U>>) {
        log::debug!("receive {} input: {:}", type_name::<U>(), data.len(),);
        self.data.extend((**data).clone());
        self.n_sample += 1;
    }
}

/// Sample-and-hold rate transitionner
#[derive(Debug)]
pub struct Sampler<T, U: UniqueIdentifier<Data = T>, V: UniqueIdentifier<Data = T> = U> {
    input: Arc<Data<U>>,
    output: PhantomData<V>,
}
impl<T, U: UniqueIdentifier<Data = T>, V: UniqueIdentifier<Data = T>> Sampler<T, U, V> {
    /// Creates a new sampler with initial condition
    pub fn new(init: T) -> Self {
        Self {
            input: Arc::new(Data::new(init)),
            output: PhantomData,
        }
    }
}
impl<T: Default, U: UniqueIdentifier<Data = T>, V: UniqueIdentifier<Data = T>> Default
    for Sampler<T, U, V>
{
    fn default() -> Self {
        Self {
            input: Arc::new(Data::new(T::default())),
            output: PhantomData,
        }
    }
}
impl<T, U: UniqueIdentifier<Data = T>, V: UniqueIdentifier<Data = T>> Update for Sampler<T, U, V> {}
impl<T, U: UniqueIdentifier<Data = T>, V: UniqueIdentifier<Data = T>> Read<T, U>
    for Sampler<T, U, V>
{
    fn read(&mut self, data: Arc<Data<U>>) {
        self.input = data;
    }
}
impl<T: Clone, U: UniqueIdentifier<Data = T>, V: UniqueIdentifier<Data = T>> Write<T, V>
    for Sampler<T, U, V>
{
    fn write(&mut self) -> Option<Arc<Data<V>>> {
        Some(Arc::new(Data::new((**self.input).clone())))
    }
}

/// Concatenates data into a [Vec]
pub struct Concat<T>(Vec<T>);
impl<T: Default> Default for Concat<T> {
    fn default() -> Self {
        Self(Vec::new())
    }
}
impl<T> Update for Concat<T> {}
impl<T: Clone + Default, U: UniqueIdentifier<Data = T>> Read<T, U> for Concat<T> {
    fn read(&mut self, data: Arc<Data<U>>) {
        self.0.push((*data).clone());
    }
}
impl<T: Clone, U: UniqueIdentifier<Data = Vec<T>>> Write<Vec<T>, U> for Concat<T> {
    fn write(&mut self) -> Option<Arc<Data<U>>> {
        Some(Arc::new(Data::new(take(&mut self.0))))
    }
}

/// Integral controller
#[derive(Default)]
pub struct Integrator<T, U: UniqueIdentifier<Data = Vec<T>>> {
    gain: Vec<T>,
    mem: Vec<T>,
    zero: Vec<T>,
    uid: PhantomData<U>,
}
impl<T, U: UniqueIdentifier<Data = Vec<T>>> Integrator<T, U>
where
    T: Default + Clone,
{
    /// Creates a new integral controller
    pub fn new(n_data: usize) -> Self {
        Self {
            gain: vec![Default::default(); n_data],
            mem: vec![Default::default(); n_data],
            zero: vec![Default::default(); n_data],
            uid: PhantomData,
        }
    }
    /// Sets a unique gain
    pub fn gain(self, gain: T) -> Self {
        Self {
            gain: vec![gain; self.mem.len()],
            ..self
        }
    }
    /// Sets the gain vector
    pub fn gain_vector(self, gain: Vec<T>) -> Self {
        assert_eq!(
            gain.len(),
            self.mem.len(),
            "gain vector length error: expected {} found {}",
            gain.len(),
            self.mem.len()
        );
        Self { gain, ..self }
    }
    /// Sets the integrator zero point
    pub fn zero(self, zero: Vec<T>) -> Self {
        Self { zero, ..self }
    }
}
impl<T, U: UniqueIdentifier<Data = Vec<T>>> Update for Integrator<T, U> {}
impl<T, U: UniqueIdentifier<Data = Vec<T>>> Read<Vec<T>, U> for Integrator<T, U>
where
    T: Copy + Mul<Output = T> + Sub<Output = T> + SubAssign,
{
    fn read(&mut self, data: Arc<Data<U>>) {
        self.mem
            .iter_mut()
            .zip(&self.gain)
            .zip(&self.zero)
            .zip(&**data)
            .for_each(|(((x, g), z), u)| *x -= *g * (*u - *z));
    }
}
impl<T, V: UniqueIdentifier<Data = Vec<T>>, U: UniqueIdentifier<Data = Vec<T>>> Write<Vec<T>, V>
    for Integrator<T, U>
where
    T: Copy + Add<Output = T>,
{
    fn write(&mut self) -> Option<Arc<Data<V>>> {
        let y: Vec<T> = self
            .mem
            .iter()
            .zip(&self.zero)
            .map(|(m, z)| *m + *z)
            .collect();
        Some(Arc::new(Data::new(y)))
    }
}

/// Discrete data sets
pub struct Source<T> {
    n: usize,
    data: Vec<T>,
}
impl<T> Source<T> {
    pub fn new(data: Vec<T>, n: usize) -> Self {
        Source { n, data }
    }
}
impl<T> Update for Source<T> {}

impl<T, V> Write<Vec<T>, V> for Source<T>
where
    V: UniqueIdentifier<Data = Vec<T>>,
{
    fn write(&mut self) -> Option<Arc<Data<V>>> {
        if self.data.is_empty() {
            None
        } else {
            let y: Vec<T> = self.data.drain(..self.n).collect();
            Some(Arc::new(Data::new(y)))
        }
    }
}

#[cfg(feature = "nalgebra")]
mod gain {
    use super::{Arc, Data, Read, UniqueIdentifier, Update, Write};
    use nalgebra as na;
    /// Gain
    pub struct Gain {
        u: na::DVector<f64>,
        y: na::DVector<f64>,
        mat: na::DMatrix<f64>,
    }
    impl Gain {
        pub fn new(mat: na::DMatrix<f64>) -> Self {
            Self {
                u: na::DVector::zeros(mat.ncols()),
                y: na::DVector::zeros(mat.nrows()),
                mat,
            }
        }
    }
    impl Update for Gain {
        fn update(&mut self) {
            self.y = &self.mat * &self.u;
        }
    }
    impl<U: UniqueIdentifier<Data = Vec<f64>>> Read<Vec<f64>, U> for Gain {
        fn read(&mut self, data: Arc<Data<U>>) {
            self.u = na::DVector::from_row_slice(&data);
        }
    }
    impl<U: UniqueIdentifier<Data = Vec<f64>>> Write<Vec<f64>, U> for Gain {
        fn write(&mut self) -> Option<Arc<Data<U>>> {
            Some(Arc::new(Data::new(self.y.as_slice().to_vec())))
        }
    }
}
#[cfg(feature = "nalgebra")]
pub use gain::Gain;
