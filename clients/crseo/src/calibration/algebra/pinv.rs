use std::ops::Mul;

use faer::{Mat, MatRef};
use serde::{Deserialize, Serialize};

use crate::calibration::mode::Modality;

use super::{Calib, CalibProps, CalibrationMode};

/// Calibration matrix pseudo-inverse
///
/// f64he 1st generic parameter `f64` is the type of the matrix values and the
/// 2nd generic parameter `M` indicates if the matrix correspond to a single segment ([CalibrationMode])
/// or to a full mirror ([MirrorMode](super::MirrorMode),[MixedMirrorMode](crate::calibration::MixedMirrorMode)).
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CalibPinv<M = CalibrationMode>
where
    M: Modality,
{
    pub(crate) mat: Mat<f64>,
    pub(crate) cond: f64,
    pub(crate) mode: M,
}

impl<M: Modality> CalibPinv<M> {
    /// Returns the condition number of the calibration matrix
    #[inline]
    pub fn cond(&self) -> f64 {
        self.cond.clone()
    }
    /// f64ransforms the pseudo-inverse matrix
    pub fn transform<F: Fn(MatRef<'_, f64>) -> Mat<f64>>(&mut self, fun: F) {
        self.mat = fun(self.mat.as_ref());
    }
    pub fn mat_ref(&self) -> MatRef<'_, f64> {
        self.mat.as_ref()
    }
}

impl<M: Modality> Mul<Vec<f64>> for &CalibPinv<M> {
    type Output = Vec<f64>;
    fn mul(self, rhs: Vec<f64>) -> Self::Output {
        let e = self.mat.as_ref() * MatRef::from_column_major_slice(rhs.as_slice(), rhs.len(), 1);
        let iter = e
            .row_iter()
            .flat_map(|r| r.iter().cloned().collect::<Vec<_>>());
        /*         match self.mode {
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
            _ => unimplemented!(),
        } */
        self.mode.fill(iter)
    }
}

impl<M: Modality> Mul<MatRef<'_, f64>> for &CalibPinv<M> {
    type Output = Mat<f64>;
    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
        self.mat.as_ref() * rhs
    }
}

impl Mul<MatRef<'_, f64>> for CalibPinv {
    type Output = Mat<f64>;
    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
        self.mat.as_ref() * rhs
    }
}

// impl Mul<&Calib> for &CalibPinv< {
//     type Output = Mat<f64>;
//     fn mul(self, rhs: &Calib) -> Self::Output {
//         self.mat.as_ref() * rhs.mat_ref()
//     }
// }

impl<M: Modality, C: CalibProps<M>> Mul<&C> for &CalibPinv<M> {
    type Output = Mat<f64>;
    fn mul(self, rhs: &C) -> Self::Output {
        self.mat.as_ref() * rhs.mat_ref()
    }
}

impl<M: Modality, C: CalibProps<M>> Mul<&C> for &mut CalibPinv<M> {
    type Output = Mat<f64>;
    fn mul(self, rhs: &C) -> Self::Output {
        self.mat.as_ref() * rhs.mat_ref()
    }
}

// impl Mul<&Calib> for &mut CalibPinv< {
//     type Output = Mat<f64>;
//     fn mul(self, rhs: &Calib) -> Self::Output {
//         self.mat.as_ref() * rhs.mat_ref()
//     }
// }

impl Mul<Calib> for CalibPinv {
    type Output = Mat<f64>;
    fn mul(self, rhs: Calib) -> Self::Output {
        self.mat.as_ref() * rhs.mat_ref()
    }
}

impl Mul<Calib> for &CalibPinv {
    type Output = Mat<f64>;
    fn mul(self, rhs: Calib) -> Self::Output {
        self.mat.as_ref() * rhs.mat_ref()
    }
}
