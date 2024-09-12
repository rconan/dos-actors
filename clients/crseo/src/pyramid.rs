use std::{
    ops::{DivAssign, MulAssign, SubAssign},
    sync::Arc,
};

use crseo::{
    imaging::LensletArray,
    wavefrontsensor::{Pyramid, PyramidBuilder},
    Builder,
};
use interface::UniqueIdentifier;
use serde::Serialize;

pub use calibrating::{PyramidCalibrator, PyramidCommand};

use crate::Processor;

use crseo::Frame;

#[derive(Default, Debug, Serialize)]
pub struct PyramidData<T> {
    sx: Vec<T>,
    sy: Vec<T>,
    flux: Vec<T>,
}

impl<T> IntoIterator for PyramidData<T> {
    type Item = T;

    type IntoIter = std::iter::Chain<std::vec::IntoIter<T>, std::vec::IntoIter<T>>;

    fn into_iter(self) -> Self::IntoIter {
        self.sx.into_iter().chain(self.sy.into_iter())
    }
}

#[derive(Default, Debug)]
pub struct PyramidProcessor<T = f32> {
    pub frame: Arc<Frame<T>>,
    lenslet_array: LensletArray,
}

impl From<&Pyramid> for Processor<PyramidProcessor> {
    fn from(value: &Pyramid) -> Self {
        Self(value.into())
    }
}

impl TryFrom<&PyramidBuilder> for Processor<PyramidProcessor> {
    type Error = crseo::CrseoError;

    fn try_from(pymb: &PyramidBuilder) -> Result<Self, Self::Error> {
        Ok((&pymb.clone().build()?).into())
    }
}

impl<T: Default> PyramidProcessor<T> {
    pub fn new(pym: &Pyramid) -> Self {
        Self {
            lenslet_array: pym.lenslet_array,
            ..Default::default()
        }
    }
}

impl From<&Pyramid> for PyramidProcessor<f32> {
    fn from(pym: &Pyramid) -> Self {
        Self {
            lenslet_array: pym.lenslet_array,
            frame: Arc::new(pym.into()),
        }
    }
}

impl SubAssign for PyramidData<f32> {
    fn sub_assign(&mut self, rhs: Self) {
        self.sx
            .iter_mut()
            .zip(self.sy.iter_mut())
            .zip(rhs.sx.into_iter().zip(rhs.sy.into_iter()))
            .for_each(|((sx, sy), (rhs_sx, rhs_sy))| {
                *sx -= rhs_sx;
                *sy -= rhs_sy;
            });
    }
}

impl DivAssign<f32> for PyramidData<f32> {
    fn div_assign(&mut self, rhs: f32) {
        self.sx
            .iter_mut()
            .zip(self.sy.iter_mut())
            .for_each(|(sx, sy)| {
                *sx /= rhs;
                *sy /= rhs;
            })
    }
}

impl MulAssign<f32> for PyramidData<f32> {
    fn mul_assign(&mut self, rhs: f32) {
        self.sx
            .iter_mut()
            .zip(self.sy.iter_mut())
            .for_each(|(sx, sy)| {
                *sx *= rhs;
                *sy *= rhs;
            })
    }
}

mod calibrating;
mod processing;

/// Pyramid measurements actor data type
pub enum PyramidMeasurements {}
impl UniqueIdentifier for PyramidMeasurements {
    type DataType = PyramidData<f32>;
}
