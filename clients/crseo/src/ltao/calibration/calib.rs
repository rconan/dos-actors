use crate::CalibrationMode;
use faer::mat::from_column_major_slice;
use faer::{Mat, MatRef};
use std::fmt::Display;
use std::ops::Mul;
use std::time::Duration;

#[derive(Debug)]
pub struct Calib {
    pub(crate) sid: u8,
    pub(crate) n_mode: usize,
    pub(crate) c: Vec<f64>,
    pub(crate) mask: Vec<bool>,
    pub(crate) mode: CalibrationMode,
    pub(crate) runtime: Duration,
}

impl Calib {
    #[inline]
    pub fn n_mode(&self) -> usize {
        self.n_mode
    }
    #[inline]
    pub fn n_cols(&self) -> usize {
        match self.mode {
            CalibrationMode::RBM(tr_xyz) => tr_xyz.iter().filter_map(|&x| x).count(),
            CalibrationMode::Modes {
                n_mode, start_idx, ..
            } => n_mode - start_idx,
        }
    }
    #[inline]
    pub fn n_rows(&self) -> usize {
        self.c.len() / self.n_cols()
    }
    pub fn mat_ref(&self) -> MatRef<'_, f64> {
        from_column_major_slice::<f64>(&self.c, self.n_rows(), self.n_cols())
    }
    pub fn pseudoinverse(&self) -> CalibPinv<f64> {
        let svd = self.mat_ref().svd();
        let s = svd.s_diagonal();
        let cond = s[0] / s[s.nrows() - 1];
        CalibPinv {
            mat: svd.pseudoinverse(),
            cond,
            mode: self.mode,
        }
    }
    pub fn area(&self) -> usize {
        self.mask.iter().filter(|x| **x).count()
    }
    pub fn mask(&self, data: &[f64]) -> Vec<f64> {
        assert_eq!(data.len(), self.mask.len());
        data.iter()
            .zip(self.mask.iter())
            .filter_map(|(x, b)| if *b { Some(*x) } else { None })
            .collect()
    }
}
impl Display for Calib {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Calib S{} ({}, {}) in {:.0?}; non-zeros: {}/{}",
            self.sid,
            self.n_rows(),
            self.n_cols(),
            self.runtime,
            self.area(),
            self.mask.len()
        )
    }
}

#[derive(Debug)]
pub struct CalibPinv<T: faer::Entity> {
    mat: Mat<T>,
    pub(crate) cond: T,
    mode: CalibrationMode,
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
                n_mode, start_idx, ..
            } => {
                if n < n_mode {
                    vec![0.; start_idx].into_iter().chain(iter).collect()
                } else {
                    iter.collect()
                }
            }
        }
    }
}
