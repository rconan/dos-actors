use faer::{Mat, MatRef};
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use serde::{Deserialize, Serialize};
use std::{
    fmt::{Display, Formatter},
    marker::PhantomData,
    ops::{Div, Mul, SubAssign},
    sync::Arc,
};

use crate::calibration::mode::{MirrorMode, MixedMirrorMode, Modality};

use super::{Block, Calib, CalibPinv, CalibProps, CalibrationMode, ClosedLoopCalib, Merge};

/// Reconstructor from calibration matrices
///
/// A reconstructor is a collection of segment wise calibration matrices.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Reconstructor<M = CalibrationMode, C = Calib<M>>
where
    M: Modality,
    C: CalibProps<M>,
{
    calib: Vec<C>,
    pinv: Vec<Option<CalibPinv<f64, M>>>,
    data: Arc<Vec<f64>>,
    estimate: Arc<Vec<f64>>,
    mode: PhantomData<M>,
}

impl<M> From<Calib<M>> for Reconstructor<M, Calib<M>>
where
    M: Modality + Default,
    Calib<M>: CalibProps<M> + Default,
{
    fn from(calib: Calib<M>) -> Self {
        Self::new(vec![calib])
    }
}

impl<M, C> Reconstructor<M, C>
where
    M: Modality + Default,
    C: CalibProps<M> + Default,
{
    /// Creates a new reconstructor
    pub fn new(calib: Vec<C>) -> Self {
        Self {
            pinv: vec![None; calib.len()],
            calib,
            ..Default::default()
        }
    }
    pub fn calib_slice(&self) -> &[C] {
        &self.calib
    }
    /// Computes the pseudo-inverse of the calibration matrices
    pub fn pseudoinverse(&mut self) -> &mut Self {
        self.pinv = self.calib.iter().map(|c| Some(c.pseudoinverse())).collect();
        self
    }
    /// Returns the trucated pseudo-inverse of the calibration matrices
    ///
    /// The inverse of the last `n` eigen values are set to zero
    pub fn truncated_pseudoinverse(&mut self, n: Vec<usize>) -> &mut Self {
        self.pinv = self
            .calib
            .iter()
            .zip(n.into_iter())
            .map(|(c, n)| Some(c.truncated_pseudoinverse(n)))
            .collect();
        self
    }
    /// Returns the total number of non-zero inputs
    pub fn area(&self) -> usize {
        self.calib.iter().map(|c| c.area()).sum()
    }
    /// Computes the intersection of the calibration matrices of two reconstructors
    ///
    /// The calibration matrices are filtered according to the mask resulting from the intersection of their masks.
    pub fn match_areas(&mut self, other: &mut Self) {
        self.calib
            .iter_mut()
            .zip(&mut other.calib)
            .for_each(|(c, oc)| c.match_areas(oc));
    }
    /// Solves `AX=B` for each pair of calibration matrices in two reconstructors
    ///
    /// [Self] is A and `B` is another reconstructor
    pub fn least_square_solve(&mut self, b: &Reconstructor<M, C>) -> Vec<Mat<f64>> {
        self.pinv()
            .zip(&b.calib)
            .map(|(pinv, calib)| pinv * calib)
            .collect()
    }
    // pub fn iter(&self) -> impl Iterator<Item = MatRef<'_, f64>> {
    //     self.calib.iter().map(|c| c.mat_ref())
    // }
    /// Returns an iterator over the calibration matrices
    pub fn calib(&self) -> impl Iterator<Item = &C> {
        self.calib.iter()
    }
    /// Returns an iterator over the pseudo-inverse of the calibration matrices
    pub fn pinv(&mut self) -> impl Iterator<Item = &mut CalibPinv<f64, M>> {
        self.pinv
            .iter_mut()
            .zip(&self.calib)
            .map(|(p, c)| p.get_or_insert_with(|| c.pseudoinverse()))
            .map(|p| p)
    }
    /// Returns an iterator over the calibration matrices and their pseudo-inverse
    pub fn calib_pinv(&mut self) -> impl Iterator<Item = (&C, &CalibPinv<f64, M>)> {
        self.pinv
            .iter_mut()
            .zip(&self.calib)
            .map(|(p, c)| (c, p.get_or_insert_with(|| c.pseudoinverse())))
            .map(|(c, p)| (c, &*p))
    }
    /// Returns the calibration matrices cross-talk vector
    pub fn cross_talks(&self) -> Vec<usize> {
        let n = self.calib[0].mask_as_slice().len();
        (0..n)
            .map(|i| {
                self.calib
                    .iter()
                    .fold(0usize, |m, c| m + if c.mask_as_slice()[i] { 1 } else { 0 })
            })
            .collect()
    }
    /// Returns the number of calibration matrices cross-talks
    pub fn n_cross_talks(&self) -> usize {
        self.cross_talks().iter().filter(|&&c| c > 1).count()
    }
}

impl<M: Modality + Default, C: CalibProps<M> + Default + Display> Display for Reconstructor<M, C> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "RECONSTRUCTOR (non-zeros={}): ", self.area())?;
        for (c, ic) in self.calib.iter().zip(&self.pinv) {
            if let Some(ic) = ic {
                writeln!(f, " * {c} ; cond: {:6.3E}", ic.cond)?
            } else {
                writeln!(f, " * {c}")?
            }
        }
        Ok(())
    }
}

pub trait Collapse {
    /// Collapses the calibration matrices into a single matrix
    ///
    /// The matrices are concatenated column wise.
    fn collapse(self) -> Reconstructor<MirrorMode, Calib<MirrorMode>>;
}

impl Collapse for Reconstructor {
    fn collapse(self) -> Reconstructor<MirrorMode, Calib<MirrorMode>> {
        // let Calib {
        //     sid,
        //     n_mode,
        //     c,
        //     mask,
        //     mode,
        //     runtime,
        //     n_cols,
        // } = self;
        let calib = Calib {
            sid: 0,
            n_mode: self.calib[0].n_mode,
            mask: self.calib[0].mask.clone(),
            mode: self
                .calib
                .iter()
                .fold(MirrorMode::default(), |m, calib| m.update(calib.smode())),
            runtime: self.calib.iter().map(|calib| calib.runtime).sum(),
            n_cols: Some(self.calib.iter().map(|calib| calib.n_cols()).sum()),
            c: self.calib.into_iter().flat_map(|calib| calib.c).collect(),
        };
        Reconstructor::new(vec![calib])
    }
}

impl Collapse for Reconstructor<CalibrationMode, ClosedLoopCalib> {
    fn collapse(self) -> Reconstructor<MirrorMode, Calib<MirrorMode>> {
        let Calib {
            n_mode,
            mask,
            runtime,
            ..
        } = self.calib[0].m1_closed_loop_to_sensor.clone();
        let calib = Calib {
            sid: 0,
            n_mode,
            mask,
            n_cols: Some(self.calib.iter().map(|calib| calib.n_cols()).sum()),
            mode: self
                .calib
                .iter()
                .fold(MirrorMode::default(), |m, calib| m.update(calib.smode())),
            runtime,
            c: self
                .calib
                .into_iter()
                .flat_map(|c| c.m1_closed_loop_to_sensor.c)
                .collect(),
        };
        Reconstructor::new(vec![calib])
    }
}

impl<M: Modality + Default, C: CalibProps<M> + Default + Display> Reconstructor<M, C> {
    /// Normalize the calibration matrices by their Froebenius norms
    pub fn normalize(&mut self) -> Vec<f64> {
        self.calib.iter_mut().map(|c| c.normalize()).collect()
    }
}

impl<C: CalibProps<CalibrationMode>> Reconstructor<CalibrationMode, C> {
    /// Returns the # of calibration matrix
    pub fn len(&self) -> usize {
        self.calib.len()
    }
    /// Collapses the calibration matrices into a single block-diagonal matrix
    ///
    /// The matrices are concatenated along the main diagonal.
    pub fn diagonal(&self) -> Reconstructor<MirrorMode, Calib<MirrorMode>> {
        let n_rows: usize = self.calib.iter().map(|c| c.n_rows()).sum();
        let n_cols: usize = self.calib.iter().map(|c| c.n_cols()).sum();

        let mut block_diag_mat = Mat::<f64>::zeros(n_rows, n_cols);

        let mut ni = 0;
        let mut mi = 0;
        let mut n_mode = 0;
        let mut mask = vec![];
        let mut modes: [Option<CalibrationMode>; 7] = [None, None, None, None, None, None, None];
        for (calib, mode) in self.calib.iter().zip(modes.iter_mut()) {
            let mat = calib.mat_ref();
            let mut dst = block_diag_mat
                .as_mut()
                .submatrix_mut(ni, mi, mat.nrows(), mat.ncols());
            dst.copy_from(mat);

            n_mode += calib.n_mode();
            mask.extend(calib.mask_as_slice());
            *mode = Some(calib.mode());

            ni += mat.nrows();
            mi += mat.ncols();
        }
        let calib = Calib {
            sid: 0,
            n_mode,
            mask,
            mode: MirrorMode::new(modes),
            runtime: Default::default(),
            n_cols: Some(n_cols),
            c: block_diag_mat
                .col_iter()
                .flat_map(|x| x.iter().cloned().collect::<Vec<_>>())
                .collect(),
        };
        Reconstructor::new(vec![calib])
    }
}
impl<C: CalibProps<CalibrationMode> + Merge> Reconstructor<CalibrationMode, C> {
    /// Merge two [Reconstructor]s
    ///
    /// `other` overwrite `self`, when the mode of `other` is not [None]
    pub fn merge<T: CalibProps<CalibrationMode>>(
        &mut self,
        other: Reconstructor<CalibrationMode, T>,
    ) -> &mut Self {
        self.calib.iter_mut().zip(other.calib).for_each(|(c, oc)| {
            c.merge(oc);
        });
        self
    }
}

impl<C> Block for Reconstructor<MirrorMode, C>
where
    C: CalibProps<MirrorMode> + Block<Output = Calib<MixedMirrorMode>> + Default,
{
    type Output = Reconstructor<MixedMirrorMode, Calib<MixedMirrorMode>>;
    fn block(array: &[&[&Self]]) -> Self::Output
    where
        Self: Sized,
    {
        let calib_array: Vec<Vec<&C>> = array
            .iter()
            .map(|row| {
                row.iter()
                    .flat_map(|r| r.calib_slice().iter().collect::<Vec<_>>())
                    .collect::<Vec<_>>()
            })
            .collect();
        let calib_array: Vec<&[&C]> = calib_array.iter().map(|c| c.as_slice()).collect();
        let calib = <C as Block>::block(&calib_array);
        Reconstructor::new(vec![calib])
    }
}

impl<C> Block for Reconstructor<MixedMirrorMode, C>
where
    C: CalibProps<MixedMirrorMode> + Block<Output = Calib<MixedMirrorMode>> + Default,
{
    type Output = Reconstructor<MixedMirrorMode, Calib<MixedMirrorMode>>;
    fn block(array: &[&[&Self]]) -> Self::Output
    where
        Self: Sized,
    {
        let calib_array: Vec<Vec<&C>> = array
            .iter()
            .map(|row| {
                row.iter()
                    .flat_map(|r| r.calib_slice().iter().collect::<Vec<_>>())
                    .collect::<Vec<_>>()
            })
            .collect();
        let calib_array: Vec<&[&C]> = calib_array.iter().map(|c| c.as_slice()).collect();
        let calib = <C as Block>::block(&calib_array);
        Reconstructor::new(vec![calib])
    }
}

impl<M, C> Update for Reconstructor<M, C>
where
    M: Modality + Default + Send + Sync,
    C: CalibProps<M> + Default + Send + Sync,
{
    fn update(&mut self) {
        let data = Arc::clone(&self.data);
        self.estimate = Arc::new(
            self.calib_pinv()
                .flat_map(|(c, ic)| ic * c.mask(&data))
                .collect(),
        );
    }
}

impl<M, C, U> Read<U> for Reconstructor<M, C>
where
    M: Modality + Default + Send + Sync,
    C: CalibProps<M> + Default + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

impl<M, C, U> Write<U> for Reconstructor<M, C>
where
    M: Modality + Default + Send + Sync,
    C: CalibProps<M> + Default + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.estimate.clone().into())
    }
}

impl Mul<Vec<Mat<f64>>> for &Reconstructor {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: Vec<Mat<f64>>) -> Self::Output {
        self.calib.iter().zip(rhs).map(|(c, m)| c * m).collect()
    }
}

impl Mul<MatRef<'_, f64>> for &Reconstructor {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
        self.calib.iter().map(|c| c * rhs).collect()
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

impl SubAssign<Vec<Mat<f64>>> for &mut Reconstructor {
    fn sub_assign(&mut self, rhs: Vec<Mat<f64>>) {
        self.calib
            .iter_mut()
            .zip(rhs.into_iter())
            .for_each(|(mut c, r)| c -= r);
        self.pinv = vec![None; self.calib.len()];
    }
}
