//! # Fists-order hold
//!
//! Linear interpolation between 2 successive samples

use std::fmt::Debug;
use std::ops::{Add, Mul, Sub};
use std::sync::Arc;

use interface::{Data, Read, UniqueIdentifier, Update, Write};

/// Fists-order hold
///
/// Linear interpolation between 2 successive samples
#[derive(Default, Debug)]
pub struct FirstOrderHold<T, const NI: usize, const NO: usize> {
    samples: (Option<Arc<Vec<T>>>, Option<Arc<Vec<T>>>),
    step: usize,
}
impl<T, const NI: usize, const NO: usize> FirstOrderHold<T, NI, NO>
where
    T: Clone
        + Default
        + Send
        + Sync
        + From<f64>
        + Mul<Output = T>
        + for<'a> Add<&'a T, Output = T>
        + Debug,
    for<'a> &'a T: Sub<&'a T, Output = T>,
{
    /// Creates a new [FirstOrderHold] instance
    ///
    /// Panics if `NI` is less than `NO`
    #[must_use]
    pub fn new() -> Self {
        if NI < NO {
            panic!("NI={NI} must be greater than or equal to NO={NO}");
        }
        Self {
            samples: (None, None),
            ..Default::default()
        }
    }
    #[inline]
    fn rate(&self) -> usize {
        NI / NO
    }
    fn sample(&mut self) -> Option<Arc<Vec<T>>> {
        match &self.samples {
            // linear interpolation
            (Some(y0), Some(y1)) => {
                // input sampling index
                let i = self.step % (self.rate());
                // fractional delta sample
                let u = i as f64 / self.rate() as f64;
                self.step += 1;
                Some(Arc::new(
                    y0.iter()
                        .zip(y1.iter())
                        .map(|(y0, y1)| (y1 - y0) * T::from(u) + y0)
                        .collect(),
                ))
            }
            // // first sample passes through
            (None, Some(y0)) => Some(y0.clone()),
            _ => None,
        }
    }
}
impl<T, const NI: usize, const NO: usize> Update for FirstOrderHold<T, NI, NO>
where
    T: Clone
        + Default
        + Send
        + Sync
        + From<f64>
        + Mul<Output = T>
        + for<'a> Add<&'a T, Output = T>
        + Debug,
    for<'a> &'a T: Sub<&'a T, Output = T>,
{
}

impl<T, U: UniqueIdentifier<DataType = Vec<T>>, const NI: usize, const NO: usize> Read<U>
    for FirstOrderHold<T, NI, NO>
where
    T: Clone
        + Default
        + Send
        + Sync
        + From<f64>
        + Mul<Output = T>
        + for<'a> Add<&'a T, Output = T>
        + Debug,
    for<'a> &'a T: Sub<&'a T, Output = T>,
{
    fn read(&mut self, data: Data<U>) {
        self.samples.0 = self.samples.1.clone();
        self.samples.1 = Some(data.into_arc());
    }
}

impl<T, U: UniqueIdentifier<DataType = Vec<T>>, const NI: usize, const NO: usize> Write<U>
    for FirstOrderHold<T, NI, NO>
where
    T: Clone
        + Default
        + Send
        + Sync
        + From<f64>
        + Mul<Output = T>
        + for<'a> Add<&'a T, Output = T>
        + Debug,
    for<'a> &'a T: Sub<&'a T, Output = T>,
{
    fn write(&mut self) -> Option<Data<U>> {
        self.sample().map(|y| y.into())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use interface::UID;
    #[derive(UID)]
    pub enum V {}

    #[test]
    fn test_foh() {
        let mut foh = FirstOrderHold::<f64, 2, 1>::new(2);

        <FirstOrderHold<f64, 2, 1> as Read<V>>::read(&mut foh, Data::new(vec![0.0, 1.0]));
        foh.update();
        assert_eq!(
            <FirstOrderHold<f64, 2, 1> as Write<V>>::write(&mut foh)
                .unwrap()
                .iter()
                .cloned()
                .collect::<Vec<_>>(),
            vec![0.0, 1.0]
        );
        assert_eq!(
            <FirstOrderHold<f64, 2, 1> as Write<V>>::write(&mut foh)
                .unwrap()
                .iter()
                .cloned()
                .collect::<Vec<_>>(),
            vec![0.0, 1.0]
        );

        <FirstOrderHold<f64, 2, 1> as Read<V>>::read(&mut foh, Data::new(vec![1.0, 2.0]));
        foh.update();
        assert_eq!(
            <FirstOrderHold<f64, 2, 1> as Write<V>>::write(&mut foh)
                .unwrap()
                .iter()
                .cloned()
                .collect::<Vec<_>>(),
            vec![0.0, 1.0]
        );
        assert_eq!(
            <FirstOrderHold<f64, 2, 1> as Write<V>>::write(&mut foh)
                .unwrap()
                .iter()
                .cloned()
                .collect::<Vec<_>>(),
            vec![0.5, 1.5]
        );

        <FirstOrderHold<f64, 2, 1> as Read<V>>::read(&mut foh, Data::new(vec![2.0, 3.0]));
        foh.update();
        assert_eq!(
            <FirstOrderHold<f64, 2, 1> as Write<V>>::write(&mut foh)
                .unwrap()
                .iter()
                .cloned()
                .collect::<Vec<_>>(),
            vec![1.0, 2.0]
        );
        assert_eq!(
            <FirstOrderHold<f64, 2, 1> as Write<V>>::write(&mut foh)
                .unwrap()
                .iter()
                .cloned()
                .collect::<Vec<_>>(),
            vec![1.5, 2.5]
        );

        // foh.update();
        // assert!(<FirstOrderHold<f64, 2, 1> as Write<V>>::write(&mut foh).is_none());
    }
}
