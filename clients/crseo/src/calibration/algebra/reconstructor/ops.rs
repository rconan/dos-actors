use std::ops::{Div, Mul, SubAssign};

use faer::{Mat, MatRef};

use crate::calibration::{Modality, algebra::{CalibPinv, CalibProps}};

use super::Reconstructor;

impl Mul<Vec<Mat<f64>>> for &Reconstructor {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: Vec<Mat<f64>>) -> Self::Output {
        self.calib
            .iter()
            .zip(rhs)
            .map(|(c, m)| {
                if c.mat_ref().ncols() == m.nrows() {
                    c * m
                } else {
                    Mat::<f64>::new()
                }
            })
            .collect()
    }
}

impl Mul<&Reconstructor> for &[Mat<f64>] {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: &Reconstructor) -> Self::Output {
        self.iter()
            .zip(rhs.calib.iter())
            .map(|(m, rhs)| {
                if m.ncols() == rhs.mat_ref().nrows() {
                    m * rhs.mat_ref()
                } else {
                    Mat::<f64>::new()
                }
            })
            .collect()
    }
}

impl Mul<&Reconstructor> for Vec<Mat<f64>> {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: &Reconstructor) -> Self::Output {
        self.iter()
            .zip(rhs.calib.iter())
            .map(|(m, rhs)| {
                if m.ncols() == rhs.mat_ref().nrows() {
                    m * rhs.mat_ref()
                } else {
                    Mat::<f64>::new()
                }
            })
            .collect()
    }
}

impl Mul<MatRef<'_, f64>> for &Reconstructor {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
        self.calib
            .iter()
            .map(|c| {
                if c.mat_ref().ncols() == rhs.nrows() {
                    c * rhs
                } else {
                    Mat::<f64>::new()
                }
            })
            .collect()
    }
}

impl<M: Modality, C: CalibProps<M>> Mul<Vec<Option<&CalibPinv<M>>>> for &Reconstructor<M, C> {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: Vec<Option<&CalibPinv<M>>>) -> Self::Output {
        self.calib
            .iter()
            .zip(&rhs)
            .map(|(c, rhs)| c.mat_ref() * rhs.expect("no inverse matrix").mat_ref())
            .collect()
    }
}

impl<M: Modality, C: CalibProps<M>> Div<&Reconstructor<M, C>> for MatRef<'_, f64> {
    type Output = Vec<Mat<f64>>;

    fn div(self, rhs: &Reconstructor<M, C>) -> Self::Output {
        rhs.pinv
            .iter()
            .filter_map(|ic| ic.as_ref().map(|ic| ic * self))
            .collect()
    }
}

impl<M: Modality, C: CalibProps<M>> Div<&mut Reconstructor<M, C>> for MatRef<'_, f64> {
    type Output = Vec<Mat<f64>>;

    fn div(self, rhs: &mut Reconstructor<M, C>) -> Self::Output {
        rhs.pinv
            .iter()
            .filter_map(|ic| ic.as_ref().map(|ic| ic * self))
            .collect()
    }
}

impl SubAssign<Vec<Mat<f64>>> for Reconstructor {
    fn sub_assign(&mut self, rhs: Vec<Mat<f64>>) {
        self.calib
            .iter_mut()
            .zip(rhs.into_iter())
            .for_each(|(mut c, r)| c -= r);
        self.pinv = vec![None; self.calib.len()];
    }
}
