use super::CalibPinv;
use crate::CalibrationMode;
use faer::{mat::from_column_major_slice, Mat, MatRef};
use serde::{Deserialize, Serialize};
use std::{
    fmt::Display,
    ops::{Mul, SubAssign},
    time::Duration,
};

#[derive(Debug, Serialize, Deserialize)]
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
    pub fn match_areas(&mut self, other: &mut Calib) {
        assert_eq!(self.mask.len(), other.mask.len());
        let area_a = self.area();
        let area_b = other.area();
        let mask: Vec<_> = self
            .mask
            .iter()
            .zip(other.mask.iter())
            .map(|(&a, &b)| a && b)
            .collect();

        let c_to_area: Vec<_> = self
            .c
            .chunks(area_a)
            .flat_map(|c| {
                self.mask
                    .iter()
                    .zip(&mask)
                    .filter(|&(&ma, _)| ma)
                    .zip(c)
                    .filter(|&((_, &mb), _)| mb)
                    .map(|(_, c)| *c)
            })
            .collect();
        self.c = c_to_area;
        let c_to_area: Vec<_> = other
            .c
            .chunks(area_b)
            .flat_map(|c| {
                other
                    .mask
                    .iter()
                    .zip(&mask)
                    .filter(|&(&ma, _)| ma)
                    .zip(c)
                    .filter(|&((_, &mb), _)| mb)
                    .map(|(_, c)| *c)
            })
            .collect();
        other.c = c_to_area;

        self.mask = mask.clone();
        other.mask = mask;
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

impl Mul<Mat<f64>> for &Calib {
    type Output = Mat<f64>;
    fn mul(self, rhs: Mat<f64>) -> Self::Output {
        self.mat_ref() * rhs
    }
}

impl SubAssign<Mat<f64>> for &mut Calib {
    fn sub_assign(&mut self, rhs: Mat<f64>) {
        let s = self.mat_ref() - rhs;
        self.c = s
            .col_iter()
            .flat_map(|c| c.iter().cloned().collect::<Vec<_>>())
            .collect();
    }
}
