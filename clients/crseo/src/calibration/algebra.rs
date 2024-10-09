use faer::MatRef;

use super::{mode::Modality, CalibrationMode};

pub mod calib;
mod closed_loop_calib;
pub mod pinv;
mod reconstructor;

pub use calib::{Calib, CalibBuilder};
pub use closed_loop_calib::ClosedLoopCalib;
pub use pinv::CalibPinv;
pub use reconstructor::{Collapse, Reconstructor};

/// Calibration matrix properties
pub trait CalibProps<M = CalibrationMode>
where
    M: Modality,
{
    fn sid(&self) -> u8;
    fn pseudoinverse(&self) -> CalibPinv<f64, M>;
    fn truncated_pseudoinverse(&self, n: usize) -> CalibPinv<f64, M>;
    fn area(&self) -> usize;
    fn match_areas(&mut self, other: &mut Self);
    fn mask_as_slice(&self) -> &[bool];
    fn mask_as_mut_slice(&mut self) -> &mut [bool];
    fn mask(&self, data: &[f64]) -> Vec<f64>;
    fn n_cols(&self) -> usize;
    fn n_rows(&self) -> usize;
    fn mat_ref(&self) -> MatRef<'_, f64>;
    fn n_mode(&self) -> usize;
    fn mode(&self) -> M;
    fn mode_as_mut(&mut self) -> &mut M;
    fn smode(&self) -> (u8, M) {
        (self.sid(), self.mode())
    }
    fn normalize(&mut self) -> f64;
    fn norm_l2(&mut self) -> f64;
    fn as_slice(&self) -> &[f64];
    fn as_mut_slice(&mut self) -> &mut [f64];
    fn as_mut(&mut self) -> &mut Vec<f64>;
}
pub trait Block {
    type Output;
    /// Block matrix
    ///
    /// Creates a block matrix from a nested array such as
    /// `[[A,B];[C,D]]` becomes
    /// ```ignore
    /// | A B |
    /// | C D |
    /// ```
    fn block(array: &[&[&Self]]) -> Self::Output
    where
        Self: Sized;
}

pub type ClosedLoopReconstructor = Reconstructor<CalibrationMode, ClosedLoopCalib<CalibrationMode>>;

pub trait Merge {
    /// Merge two [Calib]s
    ///
    /// `other` overwrite `self`, when the mode of `other` is not [None]
    fn merge<C: CalibProps<CalibrationMode>>(&mut self, other: C) -> &mut Self;
}

impl<C: CalibProps<CalibrationMode>> Merge for C {
    fn merge<T: CalibProps<CalibrationMode>>(&mut self, other: T) -> &mut Self {
        let n = self.n_rows();
        assert_eq!(n, other.n_rows());
        assert_eq!(self.mask_as_slice().len(), other.mask_as_slice().len());
        self.mask_as_mut_slice()
            .iter_mut()
            .zip(other.mask_as_slice())
            .for_each(|(a, b)| *a &= *b);
        let mut c: Vec<_> = self.as_slice().chunks(n).map(|x| x.to_vec()).collect();
        let mode = self.mode_as_mut();
        mode.merge(
            other.mode(),
            &mut c,
            other.as_slice().to_vec().chunks(n).map(|x| x.to_vec()),
        );
        *(self.as_mut()) = c.into_iter().flatten().collect::<Vec<_>>();
        self
    }
}
