//! # Integrator client
//!
//! An integral controller that implements the following relationship
//! between the input `u` and the output `y`:
//! `y = y - g * u` where `g` is the controller gain.
//!
//! An optional offset `o` can be passed to the client and
//! `y` will be transform as `y = y - o`.
//! The offset must be inside an [Option].

use super::{Data, Read, UniqueIdentifier, Update, Write};
use std::{
    fmt::{Debug, Display},
    marker::PhantomData,
    ops::{Add, AddAssign, Mul, Sub, SubAssign},
    sync::Arc,
};

/// Integral controller
#[derive(Clone, Debug)]
pub struct Integrator<U: UniqueIdentifier> {
    u: Arc<U::DataType>,
    gain: U::DataType,
    mem: U::DataType,
    zero: U::DataType,
    skip: usize,
    chunks: Option<usize>,
    uid: PhantomData<U>,
    leak: Option<U::DataType>,
}
impl<U, T> Display for Integrator<U>
where
    U: UniqueIdentifier<DataType = Vec<T>>,
    T: Display + Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "integral controller (gain={},leak={:?})",
            self.gain[0],
            self.leak.as_ref().map(|l| &l[0])
        )?;

        Ok(())
    }
}
impl<T, U> Integrator<U>
where
    T: Default + Clone,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    /// Creates a new integral controller
    pub fn new(n_data: usize) -> Self {
        Self {
            u: Default::default(),
            gain: vec![Default::default(); n_data],
            mem: vec![Default::default(); n_data],
            zero: vec![Default::default(); n_data],
            skip: 0,
            chunks: None,
            uid: PhantomData,
            leak: None,
        }
    }
    /// Sets a unique gain
    pub fn gain(self, gain: T) -> Self {
        Self {
            gain: vec![gain; self.mem.len()],
            ..self
        }
    }
    /// Skips the first n data
    ///
    /// Skip is always applied after chunks
    pub fn skip(mut self, n: usize) -> Self {
        self.skip = n;
        self
    }
    /// Process the data by chunks of size n
    pub fn chunks(mut self, n: usize) -> Self {
        self.chunks = Some(n);
        self
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
    /// Set the forgetting factor for the leaky integrator
    pub fn forgetting_factor(mut self, leak: T) -> Self {
        self.leak = Some(vec![leak]);
        self
    }
    /// Sets the integrator zero point
    pub fn zero(self, zero: Vec<T>) -> Self {
        Self { zero, ..self }
    }
    /// Sets the gain
    pub fn set_gain(&mut self, gain: T) -> &mut Self {
        self.gain = vec![gain; self.mem.len()];
        self
    }
}
impl<T, U> Update for Integrator<U>
where
    T: Copy + Mul<Output = T> + Sub<Output = T> + SubAssign + AddAssign + Debug + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn update(&mut self) {
        if let Some(chunks) = self.chunks {
            self.mem
                .chunks_mut(chunks)
                .zip(self.gain.chunks(chunks))
                .zip(self.zero.chunks(chunks))
                .zip(self.u.chunks(chunks - self.skip))
                .for_each(|(((mem, gain), zero), data)| {
                    mem.iter_mut()
                        .zip(gain)
                        .zip(zero)
                        .skip(self.skip)
                        .zip(data)
                        .for_each(|(((x, g), _z), u)| {
                            if let Some(leak) = &self.leak {
                                *x = leak[0] * *x - *g * (*u)
                            } else {
                                *x -= *g * (*u)
                            }
                        });
                });
        } else {
            self.mem
                .iter_mut()
                .zip(&self.gain)
                .zip(&self.zero)
                .skip(self.skip)
                .zip(&*self.u)
                .for_each(|(((x, g), _z), u)| {
                    if let Some(leak) = &self.leak {
                        *x = leak[0] * *x - *g * (*u)
                    } else {
                        *x -= *g * (*u)
                    }
                });
        }
    }
}
impl<T, U> Read<U> for Integrator<U>
where
    T: Copy + Mul<Output = T> + Sub<Output = T> + SubAssign + AddAssign + Debug + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        self.u = data.as_arc();
        assert_eq!(
            self.u.len(),
            self.mem.len(),
            "gmt_dos-clients::Integrator input size error\nexpected {}, found {}",
            self.mem.len(),
            self.u.len()
        );
    }
}
impl<T, V, U> Write<V> for Integrator<U>
where
    T: Copy + Mul<Output = T> + Sub<Output = T> + SubAssign + Add + AddAssign + Debug + Send + Sync,
    V: UniqueIdentifier<DataType = Vec<T>>,
    U: UniqueIdentifier<DataType = Vec<T>>,
    Vec<T>: FromIterator<<T as Add>::Output>,
{
    fn write(&mut self) -> Option<Data<V>> {
        let y: Vec<T> = self
            .mem
            .iter()
            .zip(&self.zero)
            .map(|(m, z)| *m + *z)
            .collect();
        Some(Data::new(y))
    }
}

/// Offset applied to the output `y`
pub struct Offset<O>(PhantomData<O>);
impl<O: UniqueIdentifier> UniqueIdentifier for Offset<O> {
    type DataType = Option<O::DataType>;
}

impl<T, U, O> Read<Offset<O>> for Integrator<U>
where
    O: UniqueIdentifier<DataType = Vec<T>>,
    U: UniqueIdentifier<DataType = Vec<T>>,
    T: Copy + Mul<Output = T> + Sub<Output = T> + SubAssign + AddAssign + Debug + Send + Sync,
{
    fn read(&mut self, data: Data<Offset<O>>) {
        (&*data).as_ref().map(|data| {
            self.mem.iter_mut().zip(data).for_each(|(m, d)| {
                *m -= *d;
            })
        });
    }
}
