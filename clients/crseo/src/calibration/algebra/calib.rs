use crate::calibration::{
    MirrorMode,
    mode::{MixedMirrorMode, Modality},
};

use super::{Block, CalibPinv, CalibProps, CalibrationMode};
use faer::{Mat, MatRef};
use serde::{Deserialize, Serialize};
use std::{
    fmt::Display,
    iter,
    ops::{Mul, SubAssign},
    time::Duration,
};

mod builder;
pub use builder::CalibBuilder;

/// Calibration matrix
///
/// The generic parameter indicates if the matrix correspond to a single segment ([CalibrationMode])
/// or to a full mirror ([MirrorMode],[MixedMirrorMode]).
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
pub struct Calib<M = CalibrationMode>
where
    M: Modality,
{
    pub(crate) sid: u8,
    pub(crate) n_mode: usize,
    // column wise values
    pub(crate) c: Vec<f64>,
    pub(crate) mask: Vec<bool>,
    pub(crate) mode: M,
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
impl From<Calib<CalibrationMode>> for Calib<MixedMirrorMode> {
    fn from(c: Calib<CalibrationMode>) -> Self {
        let Calib {
            sid,
            n_mode,
            c,
            mask,
            mode,
            runtime,
            n_cols,
        } = c;
        Self {
            sid,
            n_mode,
            c,
            mask,
            mode: mode.into(),
            runtime,
            n_cols,
        }
    }
}
impl From<Calib<MirrorMode>> for Calib<MixedMirrorMode> {
    fn from(c: Calib<MirrorMode>) -> Self {
        let Calib {
            sid,
            n_mode,
            c,
            mask,
            mode,
            runtime,
            n_cols,
        } = c;
        Self {
            sid,
            n_mode,
            c,
            mask,
            mode: mode.into(),
            runtime,
            n_cols,
        }
    }
}

impl<M> CalibProps<M> for Calib<M>
where
    M: Modality + Display,
    Calib<M>: Display,
{
    /// Returns the segment ID
    fn sid(&self) -> u8 {
        self.sid
    }
    /// Returns the pseudo-inverse of the calibration matrix
    ///
    /// The pseudo-inverse is computed using the SVD decomposition of the matrix
    /// and the condition number of the matrix is also returned within [CalibPinv].
    /// Returns a reference to the calibration matrix
    /// Return the number of rows
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, algebra::CalibProps, CalibrationMode};
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
    /// let pinv = calib.pseudoinverse().unwrap();
    /// assert_eq!(pinv.cond(),1f64);
    /// ```
    fn pseudoinverse(&self) -> Option<CalibPinv<M>> {
        if self.is_empty() {
            return None;
        }
        let svd = self.mat_ref().svd().expect("failed to compute SVD");
        let s = svd.S().column_vector();
        let n_s = s.nrows();
        let u = svd.U().subcols(0, n_s);
        let v = svd.V();
        let i_s: Vec<_> = s.iter().map(|x| x.recip()).collect();
        let i_s_diag = MatRef::from_column_major_slice(&i_s, n_s, 1)
            .col(0)
            .as_diagonal();
        let mat = v * i_s_diag * u.transpose();
        let cond = s[0] / s[s.nrows() - 1];
        Some(CalibPinv {
            mat,
            cond,
            mode: self.mode.clone(),
        })
    }
    /// Returns the trucated pseudo-inverse of the calibration matrix
    ///
    /// The inverse of the last `n` eigen values are set to zero
    fn truncated_pseudoinverse(&self, n: usize) -> Option<CalibPinv<M>> {
        if self.is_empty() {
            return None;
        }
        let svd = self.mat_ref().svd().expect("failed to compute SVD");
        let s = svd.S().column_vector();
        let n_s = s.nrows();
        let u = svd.U().subcols(0, n_s);
        let v = svd.V();
        let i_s: Vec<_> = s
            .iter()
            .take(n_s - n)
            .map(|x| x.recip())
            .chain(vec![0.; n])
            .collect();
        let i_s_diag = MatRef::from_column_major_slice(&i_s, n_s, 1)
            .col(0)
            .as_diagonal();
        let mat = v * i_s_diag * u.transpose();
        let cond = s[0] / s[n_s - n - 1];
        Some(CalibPinv {
            mat,
            cond,
            mode: self.mode.clone(),
        })
    }
    /// Returns the number of non-zero elements in the inputs mask
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, algebra::CalibProps, CalibrationMode};
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
    fn match_areas(&mut self, other: &mut Calib<M>) {
        <Self as MatchAreas<M>>::match_areas(self, other);
        /* assert_eq!(
            self.mask.len(),
            other.mask.len(),
            "failed to match areas these 2 `Calib:`\n > {self}\n > {other}"
        );
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
        other.mask = mask; */
    }
    fn mask_as_slice(&self) -> &[bool] {
        &self.mask
    }
    fn mask_as_mut_slice(&mut self) -> &mut [bool] {
        &mut self.mask
    }
    /// Applies the mask to the input data
    ///
    /// The mask is applied element-wise to the input data, returning a new
    /// vector with only the elements for which the mask is `true`.
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, algebra::CalibProps, CalibrationMode};
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
        // assert_eq!(data.len(), self.mask_slice().iter().filter(|&&x| x).count());
        data.iter()
            .cycle()
            .zip(self.mask_as_slice().iter())
            .filter_map(|(x, b)| if *b { Some(*x) } else { None })
            .collect()
    }
    /// Return the number of columns
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, algebra::CalibProps, CalibrationMode};
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
    fn n_cols(&self) -> usize {
        if let Some(n_cols) = self.n_cols {
            return n_cols;
        }
        self.mode.n_cols()
    }
    /// Return the number of rows
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, algebra::CalibProps, CalibrationMode};
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
    fn n_rows(&self) -> usize {
        let n_cols = self.n_cols();
        if n_cols > 0 { self.c.len() / n_cols } else { 0 }
    }
    /// Returns a reference to the calibration matrix
    /// Return the number of rows
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, algebra::CalibProps, CalibrationMode};
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
    fn mat_ref(&self) -> MatRef<'_, f64> {
        MatRef::from_column_major_slice(&self.c, self.n_rows(), self.n_cols())
    }
    /// Return the number of modes
    ///
    /// The number of modes corresponds to the number of degree of freedoms
    /// associated with the probed property, e.g. calibrating Rx and Ry
    /// of M1 RBMS gives `n_mode=6` and `n_cols=2`
    /// ```
    /// # use gmt_dos_clients_crseo::calibration::{Calib, algebra::CalibProps, CalibrationMode};
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
    fn n_mode(&self) -> usize {
        self.n_mode
    }
    #[inline]
    fn mode(&self) -> M {
        self.mode.clone()
    }
    /// Normalize the calibration matrix by its Froebenius norm
    fn normalize(&mut self) -> f64 {
        let mat = self.mat_ref();
        let norm = mat.norm_l2();
        self.c = mat
            .col_iter()
            .flat_map(|c| c.iter().map(|&x| x / norm).collect::<Vec<_>>())
            .collect();
        norm
    }
    /// Return the calibration matrix Froebenius norm
    fn norm_l2(&mut self) -> f64 {
        self.mat_ref().norm_l2()
    }
    /// Return a slice of [Calib] column wise
    fn as_slice(&self) -> &[f64] {
        self.c.as_slice()
    }

    fn mode_as_mut(&mut self) -> &mut M {
        &mut self.mode
    }

    fn as_mut_slice(&mut self) -> &mut [f64] {
        self.c.as_mut_slice()
    }

    fn as_mut(&mut self) -> &mut Vec<f64> {
        &mut self.c
    }

    fn empty(sid: u8, n_mode: usize, mode: M) -> Self {
        Self {
            sid,
            n_mode,
            mode,
            c: Default::default(),
            mask: Default::default(),
            runtime: Default::default(),
            n_cols: Default::default(),
        }
    }

    fn filter(&mut self, filter: &[bool]) {
        assert_eq!(self.mask.len(), filter.len());
        self.c = self
            .c
            .chunks(self.c.len() / self.n_cols())
            .flat_map(|c| {
                c.iter()
                    .zip(filter.iter().zip(&self.mask).filter(|&(_, &m)| m))
                    .filter_map(|(c, (f, _))| f.then_some(*c))
                    .collect::<Vec<_>>()
            })
            .collect();
        self.mask.iter_mut().zip(filter).for_each(|(m, f)| {
            *m = *m && *f;
        });
    }

    fn shape(&self) -> (usize, usize) {
        (self.n_rows(), self.n_cols())
    }

    fn smode(&self) -> (u8, M) {
        (self.sid(), self.mode())
    }

    fn is_empty(&self) -> bool {
        self.as_slice().is_empty()
    }

    fn guide_stars_differentiation(&self, n_guide_star: usize) -> Calib<M>
    where
        Self: Sized,
    {
        assert!(
            n_guide_star > 1,
            "only 1 guide star, cannot differentiate Calib wrt. guide stars"
        );
        // let mask = calib.mask_as_slice();
        // dbg!(self.mask.len());
        // getting the # of non zeros elements in the mask
        let nz = self
            .mask
            .iter()
            .filter(|&&x| x)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        log::debug!("non zeros: {nz}");
        // getting the # of non zeros elements in the mask per guide star
        let nzs: Vec<_> = self
            .mask
            .chunks(self.mask.len() / n_guide_star)
            .map(|mask| {
                mask.iter()
                    .filter(|&&x| x)
                    .enumerate()
                    .map(|(i, _)| i)
                    .last()
                    .unwrap()
                    + 1
            })
            .collect();
        log::debug!("GS non zeros: {nzs:?}={}", nzs.iter().sum::<usize>());

        let mat = self.mat_ref();
        let mats = nzs.iter().scan(0usize, |s, nz| {
            let smat = mat.subrows(*s, *nz);
            *s += *nz;
            Some(smat)
        });
        log::debug!(
            "mats: {:?}",
            mats.clone().map(|mat| mat.shape()).collect::<Vec<_>>()
        );
        let diff_mask = self.mask.chunks(self.mask.len() / n_guide_star).fold(
            vec![true; self.mask.len() / n_guide_star],
            |mut prod, mask| {
                prod.iter_mut().zip(mask).for_each(|(p, m)| *p = *p && *m);
                prod
            },
        );
        dbg!(diff_mask.len());
        // getting the # of non zeros elements in the collapsed mask
        let dnz = diff_mask
            .iter()
            .filter(|&&x| x)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        log::debug!("diff mask non zeros: {dnz}");
        // duplicating the collapsed mask for each guide star
        // and filtering each mask according to the centroids mask
        let fmask: Vec<_> = self
            .mask
            .chunks(self.mask.len() / n_guide_star)
            .map(|mask| {
                mask.iter()
                    .zip(&diff_mask)
                    .filter(|(m, _)| **m)
                    .map(|(_, d)| *d)
                    .collect::<Vec<_>>()
            })
            .collect();
        // dbg!(fmask.iter().map(|mask| mask.len()).collect::<Vec<_>>());
        // getting the # of non zeros elements in the filtered collapsed duplicated mask
        let fnz: Vec<_> = fmask
            .iter()
            .map(|mask| {
                mask.iter()
                    .filter(|&&x| x)
                    .enumerate()
                    .map(|(i, _)| i)
                    .last()
                    .unwrap()
                    + 1
            })
            .collect();
        log::debug!("filtered diff mask non zeros: {fnz:?}");
        let n = fnz[0];
        // filtering the rows of the sub-matrices according to the former mask
        let fmats = mats.zip(&fmask).map(|(mat, mask)| {
            let mat = mat
                .row_iter()
                .zip(mask)
                .filter(|&(_, &mask)| mask)
                .flat_map(|(mat, _)| mat.iter().copied().collect::<Vec<_>>())
                .collect::<Vec<_>>();
            MatRef::from_row_major_slice(mat.as_slice(), n, 2).to_owned()
        });
        log::debug!(
            "fmats: {:?}",
            fmats.clone().map(|mat| mat.shape()).collect::<Vec<_>>()
        );
        // differentiating the sub-matrices
        let mut w = fmats.cycle().peekable();
        let diff_mats: Vec<_> = (0..n_guide_star)
            .map(|_| w.next().unwrap() - w.peek().unwrap())
            .collect();
        println!(
            "diff. mats: {:?}",
            diff_mats.iter().map(|mat| mat.shape()).collect::<Vec<_>>()
        );
        // concatenating the sub-matrices vertically
        let row_wise_mat: Vec<_> = diff_mats
            .iter()
            .flat_map(|mat| {
                mat.row_iter()
                    .flat_map(|row| row.iter().copied().collect::<Vec<_>>())
                    .collect::<Vec<_>>()
            })
            .collect();
        let nrows = n * n_guide_star;
        let ncols = row_wise_mat.len() / nrows;
        // rasterizing the matrix column wise
        let mat = MatRef::from_row_major_slice(&row_wise_mat, nrows, ncols);
        let c: Vec<_> = mat
            .col_iter()
            .flat_map(|col| col.iter().copied().collect::<Vec<_>>())
            .collect();
        // building the calibration and reconstructor from the differentiated sub-matrices
        CalibBuilder::<M> {
            sid: self.sid,
            n_mode: self.n_mode,
            c,
            mask: iter::repeat(diff_mask)
                .take(n_guide_star)
                .flatten()
                .collect(),
            mode: self.mode.clone(),
            n_cols: None,
        }
        .build()
    }
}

pub trait MatchAreas<M, Mp = M>
where
    M: Modality + Display,
    Mp: Modality + Display,
{
    /// Computes the intersection of the mask with the mask on another [Calib]
    ///
    /// Both matrices are filtered according to the mask resulting from the
    /// intersection of their masks.
    fn match_areas(this: &mut Calib<M>, other: &mut Calib<Mp>) {
        assert_eq!(
            this.mask.len(),
            other.mask.len(),
            "failed to match areas these 2 `Calib:`\n > {this}\n > {other}"
        );
        let area_a = this.area();
        let area_b = other.area();
        let mask: Vec<_> = this
            .mask
            .iter()
            .zip(other.mask.iter())
            .map(|(&a, &b)| a && b)
            .collect();

        let c_to_area: Vec<_> = this
            .c
            .chunks(area_a)
            .flat_map(|c| {
                this.mask
                    .iter()
                    .zip(&mask)
                    .filter(|&(&ma, _)| ma)
                    .zip(c)
                    .filter(|&((_, &mb), _)| mb)
                    .map(|(_, c)| *c)
            })
            .collect();
        this.c = c_to_area;
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

        this.mask = mask.clone();
        other.mask = mask;
    }
}
impl<M: Modality + Display, Mp: Modality + Display> MatchAreas<M, Mp> for Calib<M> {}

impl<M> Calib<M>
where
    M: Modality + Default,
    Calib<M>: CalibProps<M> + Display,
{
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
    pub fn builder() -> CalibBuilder<M> {
        CalibBuilder::default()
    }
}

impl Block for Calib<MirrorMode>
where
    Calib<MirrorMode>: CalibProps<MirrorMode> + Display,
{
    type Output = Calib<MixedMirrorMode>;
    fn block(array: &[&[&Self]]) -> Self::Output
    where
        Self: Sized,
    {
        let mut rows_n_cols: Vec<_> = array
            .iter()
            .map(|&row| row.iter().map(|r| r.n_cols()).sum::<usize>())
            .collect();
        rows_n_cols.dedup();
        let n_cols = if rows_n_cols.len() > 1 {
            panic!(
                "All rows must have the same number of columns: {:?}",
                rows_n_cols
            );
        } else {
            rows_n_cols.pop().unwrap()
        };

        let n_rows = array
            .iter()
            .map(|row| {
                let mut row_n_rows: Vec<_> = row.iter().map(|r| r.n_rows()).collect();
                row_n_rows.dedup();
                if row_n_rows.len() > 1 {
                    panic!(
                        "All calibrations in the same row must have the same number of rows: {:?}",
                        row_n_rows
                    );
                } else {
                    row_n_rows.pop().unwrap()
                }
            })
            .sum::<usize>();

        let mut mat = Mat::<f64>::zeros(n_rows, n_cols);
        let mut ni: usize = 0;
        let mut mj: usize;
        let mut mask = vec![];
        let mut mode = vec![];
        for row in array.iter() {
            mj = 0;

            let mut row_mask: Vec<bool> = row[0].mask_as_slice().to_vec();
            for calib in row.iter() {
                // row_mask.extend(calib.mask_slice());

                let elem = calib.mat_ref();
                let mut dst = mat
                    .as_mut()
                    .submatrix_mut(ni, mj, elem.nrows(), elem.ncols());
                dst.copy_from(elem);
                mj += elem.ncols();
                row_mask
                    .iter_mut()
                    .zip(calib.mask_as_slice().into_iter())
                    .for_each(|(m, r)| {
                        *m &= r;
                    });
                mode.push(calib.mode());
            }
            // mask.iter_mut()
            //     .zip(row_mask.into_iter())
            //     .for_each(|(m, r)| {
            //         *m &= r;
            //     });
            mask.extend(row_mask);
            ni += row[0].mat_ref().nrows();
        }
        Calib {
            sid: 0,
            n_mode: n_cols,
            c: mat
                .col_iter()
                .flat_map(|x| x.iter().cloned().collect::<Vec<_>>())
                .collect(),
            mask,
            mode: MixedMirrorMode::from(mode),
            runtime: Default::default(),
            n_cols: Some(n_cols),
        }
    }
}

impl Block for Calib<MixedMirrorMode>
where
    Calib<MixedMirrorMode>: CalibProps<MixedMirrorMode> + Display,
{
    type Output = Calib<MixedMirrorMode>;
    fn block(array: &[&[&Self]]) -> Self::Output
    where
        Self: Sized,
    {
        let mut rows_n_cols: Vec<_> = array
            .iter()
            .map(|&row| row.iter().map(|r| r.n_cols()).sum::<usize>())
            .collect();
        rows_n_cols.dedup();
        let n_cols = if rows_n_cols.len() > 1 {
            panic!(
                "All rows must have the same number of columns: {:?}",
                rows_n_cols
            );
        } else {
            rows_n_cols.pop().unwrap()
        };

        let n_rows = array
            .iter()
            .map(|row| {
                let mut row_n_rows: Vec<_> = row.iter().map(|r| r.n_rows()).collect();
                row_n_rows.dedup();
                if row_n_rows.len() > 1 {
                    panic!(
                        "All calibrations in the same row must have the same number of rows: {:?}",
                        row_n_rows
                    );
                } else {
                    row_n_rows.pop().unwrap()
                }
            })
            .sum::<usize>();

        let mut mat = Mat::<f64>::zeros(n_rows, n_cols);
        let mut ni: usize = 0;
        let mut mj: usize;
        let mut mask = vec![];
        let mut mode = MixedMirrorMode::default();
        for row in array.iter() {
            mj = 0;

            let mut row_mask: Vec<bool> = row[0].mask_as_slice().to_vec();
            for calib in row.iter() {
                // row_mask.extend(calib.mask_slice());

                let elem = calib.mat_ref();
                let mut dst = mat
                    .as_mut()
                    .submatrix_mut(ni, mj, elem.nrows(), elem.ncols());
                dst.copy_from(elem);
                mj += elem.ncols();
                row_mask
                    .iter_mut()
                    .zip(calib.mask_as_slice().into_iter())
                    .for_each(|(m, r)| {
                        *m &= r;
                    });
                mode = calib.mode();
            }
            // mask.iter_mut()
            //     .zip(row_mask.into_iter())
            //     .for_each(|(m, r)| {
            //         *m &= r;
            //     });
            mask.extend(row_mask);
            ni += row[0].mat_ref().nrows();
        }
        Calib {
            sid: 0,
            n_mode: n_cols,
            c: mat
                .col_iter()
                .flat_map(|x| x.iter().cloned().collect::<Vec<_>>())
                .collect(),
            mask,
            mode,
            runtime: Default::default(),
            n_cols: Some(n_cols),
        }
    }
}

impl<M: Modality + Display> Display for Calib<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.sid > 0 {
            write!(
                f,
                "Calib({}) S{} ({}, {}) in {:.0?}; non-zeros: {}/{}",
                self.mode,
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
                "Calib({}) ({}, {}) in {:.0?}; non-zeros: {}/{}",
                self.mode,
                self.n_rows(),
                self.n_cols(),
                self.runtime,
                self.area(),
                self.mask.len()
            )
        }
    }
}

impl Mul<&[f64]> for &Calib {
    type Output = Mat<f64>;
    fn mul(self, rhs: &[f64]) -> Self::Output {
        self.mat_ref() * MatRef::from_row_major_slice(rhs, rhs.len(), 1)
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

#[cfg(test)]
mod tests {
    use crate::calibration::algebra::Merge;

    use super::*;

    #[test]
    fn block_ab() {
        // Create some sample Calib instances
        let calib1 = Calib::builder()
            .c(vec![1.0, 2.0, 3.0, 4.0, 9.0, 0.0])
            .n_mode(2)
            .n_cols(3)
            .mask(vec![true, true])
            .mode(MirrorMode::default())
            .build();
        println!("{calib1}");

        let calib2 = Calib::builder()
            .c(vec![5.0, 6.0, 7.0, 8.0])
            .n_mode(2)
            .n_cols(2)
            .mask(vec![true, false])
            .mode(MirrorMode::default())
            .build();
        println!("{calib2}");

        let block_calib = Calib::block(&[&[&calib1, &calib2]]);
        println!("{block_calib}");
    }

    #[test]
    fn block() {
        // Create some sample Calib instances
        let calib1 = Calib::builder()
            .c(vec![1.0, 2.0, 3.0, 4.0, 9.0, 0.0])
            .n_mode(2)
            .n_cols(2)
            .mask(vec![true, true])
            .mode(MirrorMode::default())
            .build();
        println!("{calib1}");

        let calib2 = Calib::builder()
            .c(vec![5.0, 6.0, 7.0, 8.0])
            .n_mode(2)
            .n_cols(2)
            .mask(vec![true, false])
            .mode(MirrorMode::default())
            .build();
        println!("{calib2}");

        let calib3 = Calib::builder()
            .c(vec![9.0, 10.0, 11.0, 12.0, 13.0, 14.0])
            .n_mode(2)
            .n_cols(3)
            .mask(vec![true, true, true])
            .mode(MirrorMode::default())
            .build();
        println!("{calib3}");

        let block_calib = Calib::block(&[&[&calib1], &[&calib2]]);
        println!("{block_calib}");
        println!("{:?}", block_calib.mat_ref());
        println!("{:?}", block_calib.mask_as_slice());

        let block_calib = Calib::block(&[&[&calib2, &calib3]]);
        println!("{block_calib}");
        println!("{:?}", block_calib.mat_ref());
        println!("{:?}", block_calib.mask_as_slice());
    }

    #[test]
    fn merge() {
        let mut calib1: Calib = Calib::builder()
            .c(vec![1.; 8])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::t_xy(1e-6))
            .build();
        println!("{calib1}");
        let calib2: Calib = Calib::builder()
            .c(vec![2.; 8])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::r_xy(1e-6))
            .build();
        println!("{calib2}");

        calib1.merge(calib2);
        println!("{calib1}");
        println!("{:?}", calib1.mat_ref());
    }

    #[test]
    fn merge_overwrite() {
        let mut calib1: Calib = Calib::builder()
            .c(vec![1.; 8])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::t_xy(1e-6))
            .build();
        println!("{calib1}");
        let calib2: Calib = Calib::builder()
            .c(vec![2.; 8])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::t_xy(1e-6))
            .build();
        println!("{calib2}");

        calib1.merge(calib2);
        println!("{calib1}");
        println!("{:?}", calib1.mat_ref());
    }

    #[test]
    fn merge_mixed() {
        let mut calib1: Calib = Calib::builder()
            .c(vec![1.; 8])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::t_xy(1e-6))
            .build();
        println!("{calib1}");
        let calib2: Calib = Calib::builder()
            .c(vec![2.; 4])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::t_z(1e-6))
            .build();
        println!("{calib2}");

        calib1.merge(calib2);
        println!("{calib1}");
        println!("{:?}", calib1.mat_ref());

        let mut calib3: Calib = Calib::builder()
            .c(vec![3.; 8])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::r_xy(1e-6))
            .build();
        println!("{calib3}");
        let calib4: Calib = Calib::builder()
            .c(vec![4.; 4])
            .n_mode(6)
            .mask(vec![true, true])
            .mode(CalibrationMode::t_z(1e-6))
            .build();
        println!("{calib4}");

        calib3.merge(calib4);
        println!("{calib3}");
        println!("{:?}", calib3.mat_ref());

        calib1.merge(calib3);
        println!("{calib1}");
        println!("{:?}", calib1.mat_ref());
    }
}
