use super::{CalibPinv, CalibProps, CalibrationMode};
use faer::{mat::from_column_major_slice, Mat, MatRef};
use serde::{Deserialize, Serialize};
use std::{
    fmt::Display,
    ops::{Mul, SubAssign},
    time::Duration,
};

mod builder;
pub use builder::CalibBuilder;

/// Calibration matrix
///
/// # Examples
///
/// A fictitious identity calibration matrix that takes RBM Rx and Ry as inputs
/// a returns the same RBM Rx and Ry as outputs.
/// ```
/// use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
/// use skyangle::Conversion;
///
/// let calib = Calib::builder()
///     .c(vec![1f64,0.,0.,1.])
///     .n_mode(6)
///     .mode(CalibrationMode::RBM([
///         None, None, None,
///         Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
///     ]))
///     .mask(vec![false, false, false, true, true, false])
///     .build();
/// ```
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Calib {
    pub(crate) sid: u8,
    pub(crate) n_mode: usize,
    pub(crate) c: Vec<f64>,
    pub(crate) mask: Vec<bool>,
    pub(crate) mode: CalibrationMode,
    pub(crate) runtime: Duration,
    pub(crate) n_cols: Option<usize>,
}

impl From<&Calib> for Vec<i8> {
    fn from(calib: &Calib) -> Self {
        calib
            .mask
            .iter()
            .take(calib.mask.len() / 2)
            .map(|&x| x as i8)
            .collect()
    }
}

impl CalibProps for Calib {
    /// Returns the pseudo-inverse of the calibration matrix
    ///
    /// The pseudo-inverse is computed using the SVD decomposition of the matrix
    /// and the condition number of the matrix is also returned within [CalibPinv].
    /// Returns a reference to the calibration matrix
    /// Return the number of rows
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// # use skyangle::Conversion;
    /// #
    /// # let calib = Calib::builder()
    /// #    .c(vec![1f64,0.,0.,1.])
    /// #    .n_mode(6)
    /// #    .mode(CalibrationMode::RBM([
    /// #        None, None, None,
    /// #        Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    /// #    ]))
    /// #    .mask(vec![false, false, false, true, true, false])
    /// #    .build();
    /// let pinv = calib.pseudoinverse();
    /// assert_eq!(pinv.cond(),1f64);
    /// ```
    fn pseudoinverse(&self) -> CalibPinv<f64> {
        let svd = self.mat_ref().svd();
        let s = svd.s_diagonal();
        let cond = s[0] / s[s.nrows() - 1];
        CalibPinv {
            mat: svd.pseudoinverse(),
            cond,
            mode: self.mode,
        }
    }

    /// Returns the number of non-zero elements in the inputs mask
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// # use skyangle::Conversion;
    /// #
    /// # let calib = Calib::builder()
    /// #    .c(vec![1f64,0.,0.,1.])
    /// #    .n_mode(6)
    /// #    .mode(CalibrationMode::RBM([
    /// #        None, None, None,
    /// #        Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    /// #    ]))
    /// #    .mask(vec![false, false, false, true, true, false])
    /// #    .build();
    /// assert_eq!(calib.area(), 2);
    /// ```
    fn area(&self) -> usize {
        self.mask.iter().filter(|x| **x).count()
    }
    /// Computes the intersection of the mask with the mask on another [Calib]
    ///
    /// Both matrices are filtered according to the mask resulting from the
    /// intersection of their masks.
    fn match_areas(&mut self, other: &mut Calib) {
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
    fn mask_slice(&self) -> &[bool] {
        &self.mask
    }
    /// Applies the mask to the input data
    ///
    /// The mask is applied element-wise to the input data, returning a new
    /// vector with only the elements for which the mask is `true`.
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// # use skyangle::Conversion;
    /// #
    /// # let calib = Calib::builder()
    /// #    .c(vec![1f64,0.,0.,1.])
    /// #    .n_mode(6)
    /// #    .mode(CalibrationMode::RBM([
    /// #        None, None, None,
    /// #        Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    /// #    ]))
    /// #    .mask(vec![false, false, false, true, true, false])
    /// #    .build();
    /// let r_xy = calib.mask(vec![1.,2.,3.,4.,5.,6.].as_slice());
    /// assert_eq!(r_xy,vec![4.,5.]);
    /// ```
    fn mask(&self, data: &[f64]) -> Vec<f64> {
        assert_eq!(data.len(), self.mask_slice().len());
        data.iter()
            .zip(self.mask_slice().iter())
            .filter_map(|(x, b)| if *b { Some(*x) } else { None })
            .collect()
    }
}

impl Calib {
    /// Returns the calibration matrix builder
    /// ```
    /// use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// use skyangle::Conversion;
    ///
    /// let calib = Calib::builder()
    ///     .c(vec![1f64,0.,0.,1.])
    ///     .n_mode(6)
    ///     .mode(CalibrationMode::RBM([
    ///         None, None, None,
    ///         Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    ///     ]))
    ///     .mask(vec![false, false, false, true, true, false])
    ///     .build();
    /// ```
    pub fn builder() -> CalibBuilder {
        CalibBuilder::default()
    }
    /// Return the number of modes
    ///
    /// The number of modes corresponds to the number of degree of freedoms
    /// associated with the probed property, e.g. calibrating Rx and Ry
    /// of M1 RBMS gives `n_mode=6` and `n_cols=2`
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// # use skyangle::Conversion;
    /// #
    /// # let calib = Calib::builder()
    /// #    .c(vec![1f64,0.,0.,1.])
    /// #    .n_mode(6)
    /// #    .mode(CalibrationMode::RBM([
    /// #        None, None, None,
    /// #        Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    /// #    ]))
    /// #    .mask(vec![false, false, false, true, true, false])
    /// #    .build();
    /// assert_eq!(calib.n_mode(), 6);
    /// ```
    #[inline]
    pub fn n_mode(&self) -> usize {
        self.n_mode
    }
    /// Return the number of columns
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// # use skyangle::Conversion;
    /// #
    /// # let calib = Calib::builder()
    /// #    .c(vec![1f64,0.,0.,1.])
    /// #    .n_mode(6)
    /// #    .mode(CalibrationMode::RBM([
    /// #        None, None, None,
    /// #        Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    /// #    ]))
    /// #    .mask(vec![false, false, false, true, true, false])
    /// #    .build();
    /// assert_eq!(calib.n_cols(), 2);
    /// ```
    #[inline]
    pub fn n_cols(&self) -> usize {
        if let Some(n_cols) = self.n_cols {
            return n_cols;
        }
        match self.mode {
            CalibrationMode::RBM(tr_xyz) => tr_xyz.iter().filter_map(|&x| x).count(),
            CalibrationMode::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => end_id.unwrap_or(n_mode) - start_idx,
        }
    }
    /// Return the number of rows
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// # use skyangle::Conversion;
    /// #
    /// # let calib = Calib::builder()
    /// #    .c(vec![1f64,0.,0.,1.])
    /// #    .n_mode(6)
    /// #    .mode(CalibrationMode::RBM([
    /// #        None, None, None,
    /// #        Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    /// #    ]))
    /// #    .mask(vec![false, false, false, true, true, false])
    /// #    .build();
    /// assert_eq!(calib.n_rows(), 2);
    /// ```
    #[inline]
    pub fn n_rows(&self) -> usize {
        self.c.len() / self.n_cols()
    }
    /// Returns a reference to the calibration matrix
    /// Return the number of rows
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, CalibrationMode};
    /// # use skyangle::Conversion;
    /// #
    /// # let calib = Calib::builder()
    /// #    .c(vec![1f64,0.,0.,1.])
    /// #    .n_mode(6)
    /// #    .mode(CalibrationMode::RBM([
    /// #        None, None, None,
    /// #        Some(1f64.from_arcsec()), Some(1f64.from_arcsec()), None
    /// #    ]))
    /// #    .mask(vec![false, false, false, true, true, false])
    /// #    .build();
    /// let mat = calib.mat_ref();
    /// assert_eq!(mat.nrows(), 2);
    /// assert_eq!(mat.ncols(), 2);
    /// ```
    #[inline]
    pub fn mat_ref(&self) -> MatRef<'_, f64> {
        from_column_major_slice::<f64>(&self.c, self.n_rows(), self.n_cols())
    }
}
impl Display for Calib {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.sid > 0 {
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
        } else {
            write!(
                f,
                "Calib ({}, {}) in {:.0?}; non-zeros: {}/{}",
                self.n_rows(),
                self.n_cols(),
                self.runtime,
                self.area(),
                self.mask.len()
            )
        }
    }
}

impl Mul<Mat<f64>> for &Calib {
    type Output = Mat<f64>;
    fn mul(self, rhs: Mat<f64>) -> Self::Output {
        self.mat_ref() * rhs
    }
}

impl Mul<MatRef<'_, f64>> for &Calib {
    type Output = Mat<f64>;
    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
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
