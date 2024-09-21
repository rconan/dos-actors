/*!
# [Actor]s clients

The module holds the implementation of the different clients that can be assigned to [Actor]s.

Any structure can become a client to an Actor if it implements the [Update] trait with either or both [Read] and [Write] traits.

# Example

## Logging

A simple logger with a single entry:
```
use gmt_dos_clients::Logging;
let logging = Logging::<f64>::default();
```
A logger with 2 entries and pre-allocated with 1000 elements:
```
use gmt_dos_clients::Logging;
let logging = Logging::<f64>::default().n_entry(2).capacity(1_000);
```
## Signals

A constant signal for 100 steps
```
use gmt_dos_clients::{Signals, Signal};
let signal: Signals = Signals::new(1, 100).signals(Signal::Constant(3.14));
```

A 2 outputs signal made of a constant and a sinusoid for 100 steps
```
use gmt_dos_clients::{Signals, Signal};
let signal: Signals = Signals::new(2, 100)
               .output_signal(0, Signal::Constant(3.14))
               .output_signal(1, Signal::Sinusoid{
                                        amplitude: 1f64,
                                        sampling_frequency_hz: 1000f64,
                                        frequency_hz: 20f64,
                                        phase_s: 0f64
               });
```
## Rate transitionner

A rate transition actor for a named output/input pair sampling a [Vec]
```
use gmt_dos_clients::Sampler;
#[derive(interface::UID)]
enum MyIO {};
let sampler = Sampler::<Vec<f64>, MyIO>::default();
```

## Alias to input/output UID

Creating an alias to an already existing [UniqueIdentifier] (UID)
```
use std::sync::Arc;
use interface::{Data, Write, UniqueIdentifier,Size, UID, Update};

// Original UID
#[derive(UID)]
#[uid(data = u8)]
pub enum A {}
pub struct Client {}
impl Update for Client {}
impl Write<A> for Client {
    fn write(&mut self) -> Option<Data<A>> {
        Some(Data::new(10u8))
    }
}
impl Size<A> for Client {
    fn len(&self) -> usize {
        123
    }
}

// A alias with `Write` and `Size` trait implementation for `Client`
#[derive(UID)]
#[uid(data = u8)]
#[alias(name = A, client = Client, traits = Write ,Size)]
pub enum B {}

let _: <A as UniqueIdentifier>::DataType = 1u8;
let _: <B as UniqueIdentifier>::DataType = 2u8;

let mut client = Client {};
 println!(
    "Client Write<B>: {:?}",
    <Client as Write<B>>::write(&mut client)
);
println!(
    "Client Size<B>: {:?}",
    <Client as Size<B>>::len(&mut client)
);
```

[Actor]: https://docs.rs/gmt_dos-actors
[Update]: https://docs.rs/gmt_dos-actors-clients_interface/latest/gmt_dos_actors-clients_interface/struct.Update.html
[Read]: https://docs.rs/gmt_dos-actors-clients_interface/latest/gmt_dos_actors-clients_interface/struct.Read.html
[Write]: https://docs.rs/gmt_dos-actors-clients_interface/latest/gmt_dos_actors-clients_interface/struct.Write.html
[UniqueIdentifier]: https://docs.rs/gmt_dos-actors-clients_interface/latest/gmt_dos_actors-clients_interface/struct.UniqueIdentifier.html
*/

pub use interface::Tick;
use interface::{Data, Read, TimerMarker, UniqueIdentifier, Update, Write};
use std::mem::take;

pub mod fill;
pub mod low_pass_filter;
pub mod multiplex;
pub mod signals;
pub use signals::{OneSignal, Signal, Signals};
pub mod timer;
pub use timer::Timer;
pub mod logging;
pub use logging::Logging;
pub mod sampler;
pub use sampler::Sampler;
pub mod pulse;
pub use pulse::Pulse;
pub mod integrator;
pub use integrator::{Integrator, Offset};
pub mod smooth;
pub use smooth::{Smooth, Weight};
pub mod average;
pub use average::Average;
#[cfg(feature = "nalgebra")]
mod gain;
#[cfg(feature = "nalgebra")]
pub use gain::Gain;
pub mod foh;
pub mod fun;
pub mod gif;
pub mod leftright;
pub mod once;
pub mod operator;
pub mod print;
pub mod select;

/// Concatenates data into a [Vec]
pub struct Concat<T>(Vec<T>);
impl<T: Default> Default for Concat<T> {
    fn default() -> Self {
        Self(Vec::new())
    }
}
impl<T> Update for Concat<T> where T: Send + Sync {}
impl<T, U> Read<U> for Concat<T>
where
    T: Clone + Default + Send + Sync,
    U: UniqueIdentifier<DataType = T>,
{
    fn read(&mut self, data: Data<U>) {
        self.0.push((*data).clone());
    }
}
impl<T, U> Write<U> for Concat<T>
where
    T: Clone + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(Data::new(take(&mut self.0)))
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
impl<T> TimerMarker for Source<T> {}
impl<T> Update for Source<T> where T: Send + Sync {}

impl<T, V> Write<V> for Source<T>
where
    V: UniqueIdentifier<DataType = Vec<T>>,
    T: Send + Sync,
{
    fn write(&mut self) -> Option<Data<V>> {
        if self.data.is_empty() {
            None
        } else {
            let y: Vec<T> = self.data.drain(..self.n).collect();
            Some(Data::new(y))
        }
    }
}

pub trait Progress {
    fn progress<S: Into<String>>(name: S, len: usize) -> Self;
    fn increment(&mut self);
    fn finish(&mut self) {}
}

impl Progress for indicatif::ProgressBar {
    fn progress<S: Into<String>>(name: S, len: usize) -> Self {
        let progress = indicatif::ProgressBar::new(len as u64);
        progress.set_style(
            indicatif::ProgressStyle::with_template(
                "{msg} [{eta_precise}] {bar:50.cyan/blue} {percent:>3}%",
            )
            .unwrap(),
        );
        progress.set_message(name.into());
        // let bar: Bar = progress.bar(self.tick, "Timer:");
        progress
    }
    #[inline]
    fn increment(&mut self) {
        self.inc(1)
    }
    #[inline]
    fn finish(&mut self) {
        Self::finish(self);
    }
}
