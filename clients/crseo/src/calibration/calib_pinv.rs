use std::ops::Mul;

use faer::{mat::from_column_major_slice, Mat, MatRef};
use serde::{Deserialize, Serialize};

use super::{Calib, CalibrationMode};

/// Calibration matrix pseudo-inverse
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CalibPinv<T: faer::Entity> {
    pub(crate) mat: Mat<T>,
    pub(crate) cond: T,
    pub(crate) mode: CalibrationMode,
}

impl<T: faer::Entity> CalibPinv<T> {
    /// Returns the condition number of the calibration matrix
    #[inline]
    pub fn cond(&self) -> T {
        self.cond
    }
}

impl Mul<Vec<f64>> for &CalibPinv<f64> {
    type Output = Vec<f64>;
    fn mul(self, rhs: Vec<f64>) -> Self::Output {
        let e = self.mat.as_ref() * from_column_major_slice::<f64>(rhs.as_slice(), rhs.len(), 1);
        let n = e.nrows();
        let iter = e
            .row_iter()
            .flat_map(|r| r.iter().cloned().collect::<Vec<_>>());
        match self.mode {
            CalibrationMode::RBM(tr_xyz) => {
                if n < 6 {
                    let mut out = vec![0.; 6];
                    out.iter_mut()
                        .zip(&tr_xyz)
                        .filter_map(|(out, v)| v.and_then(|_| Some(out)))
                        .zip(iter)
                        .for_each(|(out, e)| *out = e);
                    out
                } else {
                    iter.collect()
                }
            }
            CalibrationMode::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => {
                if n < n_mode {
                    if let Some(end) = end_id {
                        vec![0.; start_idx]
                            .into_iter()
                            .chain(iter)
                            .chain(vec![0.; n_mode - end])
                            .collect()
                    } else {
                        vec![0.; start_idx].into_iter().chain(iter).collect()
                    }
                } else {
                    iter.collect()
                }
            }
        }
    }
}

impl Mul<MatRef<'_, f64>> for &CalibPinv<f64> {
    type Output = Mat<f64>;
    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
        self.mat.as_ref() * rhs
    }
}

impl Mul<MatRef<'_, f64>> for CalibPinv<f64> {
    type Output = Mat<f64>;
    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
        self.mat.as_ref() * rhs
    }
}

impl Mul<&Calib> for &CalibPinv<f64> {
    type Output = Mat<f64>;
    fn mul(self, rhs: &Calib) -> Self::Output {
        self.mat.as_ref() * rhs.mat_ref()
    }
}

impl Mul<&Calib> for &mut CalibPinv<f64> {
    type Output = Mat<f64>;
    fn mul(self, rhs: &Calib) -> Self::Output {
        self.mat.as_ref() * rhs.mat_ref()
    }
}

impl Mul<Calib> for CalibPinv<f64> {
    type Output = Mat<f64>;
    fn mul(self, rhs: Calib) -> Self::Output {
        self.mat.as_ref() * rhs.mat_ref()
    }
}