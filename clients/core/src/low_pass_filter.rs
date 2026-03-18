use std::{
    f64,
    fmt::{Debug, Display, LowerExp},
    ops::{Add, AddAssign, Mul, Sub},
    sync::Arc,
};

use interface::{Data, Read, UniqueIdentifier, Update, Write};

#[derive(Debug, Clone)]
pub struct LowPassFilter<T> {
    u: Arc<Vec<T>>,
    y: Vec<T>,
    g: T,
}

impl<T: Default + Clone + Display + LowerExp> Display for LowPassFilter<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "low-pass-filer: n={} , g={:.4e}", self.y.len(), self.g)
    }
}

impl<T: Default + Clone> LowPassFilter<T> {
    /// Creates a new low-pass filter instance from the data size and the gain
    pub fn new(n: usize, g: T) -> Self {
        Self {
            u: Arc::new(vec![T::default(); n]),
            y: vec![T::default(); n],
            g,
        }
    }
}
impl LowPassFilter<f64> {
    /// Creates a new low-pass filter instance from the data size,
    ///  the corner and sampling frequencies in Hz
    pub fn from_corner_frequency(n: usize, corner: f64, sampling: f64) -> Self {
        let g = 1f64 - (-2f64 * f64::consts::PI * corner / sampling).exp();
        Self::new(n, g)
    }
    /// Returns the corner frequency for the given sampling
    pub fn corner_frequency(&self, sampling: f64) -> f64 {
        -0.5 * sampling * f64::consts::FRAC_1_PI * (1f64 - self.g).ln()
    }
}

impl<T> Update for LowPassFilter<T>
where
    T: Send + Sync + Sub<Output = T> + Add<Output = T> + Mul<Output = T> + AddAssign + Copy,
{
    fn update(&mut self) {
        let g = self.g;
        self.u
            .iter()
            .zip(self.y.iter_mut())
            .map(|(u, y)| (*u - *y, y))
            .for_each(|(e, y)| {
                *y += g * e;
            });
    }
}

impl<T, U> Read<U> for LowPassFilter<T>
where
    T: Send + Sync + Sub<Output = T> + Add<Output = T> + Mul<Output = T> + Copy + AddAssign,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        self.u = data.into_arc();
    }
}

impl<T, U> Write<U> for LowPassFilter<T>
where
    T: Copy + Send + Sync + Sub<Output = T> + Add<Output = T> + Mul<Output = T> + AddAssign,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.y.clone().into())
    }
}

#[cfg(test)]
mod tests {
    use std::f64;

    use super::*;

    #[test]
    fn corner_frequency() {
        let lpf = LowPassFilter::from_corner_frequency(3, 1f64, 1e3);
        println!("{lpf}");
        let cf = lpf.corner_frequency(1e3);
        assert!((1. - cf).abs() < f64::EPSILON * 1e3);
    }
}
