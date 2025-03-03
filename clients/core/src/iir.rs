//! # Infinite Impulse Response (IIR) filter client
//!
//!

use interface::{Data, Read, UniqueIdentifier, Update, Write};
use std::collections::VecDeque;
use std::{
    //fmt::Debug,
    ops::{Add, AddAssign, Mul, Sub},
    //sync::Arc,
};

#[derive(Debug, Clone)]
/// A multi-dimensional IIR filter where the same coefficients are applied to all dimensions
pub struct IIRFilter<T> {
    /// Feed-forward coefficients (b values)
    b_coeffs: Vec<T>,
    /// Feedback coefficients (a values excluding a[0] which is assumed to be 1.0)
    a_coeffs: Vec<T>,
    /// Number of dimensions to filter
    filter_dim: usize,
    /// Input history for each dimension [dimension][sample]
    x_history: Vec<VecDeque<T>>,
    /// Output history for each dimension [dimension][sample]
    y_history: Vec<VecDeque<T>>,
}

impl<T: Default + Clone> IIRFilter<T> {
    /// Create a new multi-dimensional IIR filter with the given coefficients
    pub fn new(b_coeffs: Vec<T>, a_coeffs: Vec<T>, filter_dim: usize) -> Self {
        // Initialize history buffers with zeros
        let x_history = vec![VecDeque::from(vec![T::default(); b_coeffs.len()]); filter_dim];
        let y_history = vec![VecDeque::from(vec![T::default(); a_coeffs.len()]); filter_dim];

        Self {
            b_coeffs,
            a_coeffs,
            filter_dim,
            x_history,
            y_history,
        }
    }

    /// Reset the filter state using iterators
    pub fn reset(&mut self) {
        // Flatten both histories into a single iterator of mutable references and set all to zero
        self.x_history
            .iter_mut()
            .flat_map(|queue| queue.iter_mut())
            .chain(self.y_history.iter_mut().flat_map(|queue| queue.iter_mut()))
            .for_each(|val| *val = T::default());
    }

    /// Change the filter input dimension
    pub fn resize(&mut self, new_filter_dim: usize) {
        if new_filter_dim == self.filter_dim {
            return;
        }

        self.filter_dim = new_filter_dim;
        self.x_history =
            vec![VecDeque::from(vec![T::default(); self.b_coeffs.len()]); new_filter_dim];
        self.y_history =
            vec![VecDeque::from(vec![T::default(); self.a_coeffs.len()]); new_filter_dim];
    }
}

impl<T> Update for IIRFilter<T>
where
    T: Send
        + Sync
        + Sub<Output = T>
        + Add<Output = T>
        + Mul<Output = T>
        + AddAssign
        + Copy
        + std::iter::Sum,
{
    // Process a new multi-dimensional input sample using iterators
    //pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
    fn update(&mut self) {
        // For each dimension
        for dim in 0..self.filter_dim {
            // Apply feed-forward coefficients using zip and sum
            let feed_forward = self
                .b_coeffs
                .iter()
                .zip(self.x_history[dim].iter())
                .map(|(&b, &x)| b * x)
                .sum::<T>();

            // Apply feedback coefficients using zip and sum
            let feedback = self
                .a_coeffs
                .iter()
                .zip(self.y_history[dim].iter())
                .map(|(&a, &y)| a * y)
                .sum::<T>();

            // Calculate output
            let y_i = feed_forward - feedback;

            // Update output history
            self.y_history[dim].pop_back();
            self.y_history[dim].push_front(y_i);
        }
    }
}

impl<T, U> Read<U> for IIRFilter<T>
where
    T: Send
        + Sync
        + Sub<Output = T>
        + Add<Output = T>
        + Mul<Output = T>
        + Copy
        + AddAssign
        + std::iter::Sum,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        let x = data.into_arc();
        assert_eq!(
            x.len(),
            self.filter_dim,
            "gmt_dos-clients::IIR filter input size error:\nexpected {}, found {}!",
            self.filter_dim,
            x.len()
        );

        // Update the input history for each dimension
        for dim in 0..self.filter_dim {
            self.x_history[dim].pop_back();
            self.x_history[dim].push_front(x[dim]);
        }
    }
}
impl<T, U> Write<U> for IIRFilter<T>
where
    T: Copy
        + Send
        + Sync
        + Sub<Output = T>
        + Add<Output = T>
        + Mul<Output = T>
        + AddAssign
        + std::iter::Sum,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        // Return the most recent output from each dimension
        let y: Vec<T> = (0..self.filter_dim)
            .map(|dim| self.y_history[dim][0])
            .collect();
        Some(Data::new(y))
    }
}
