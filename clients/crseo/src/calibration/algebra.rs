use faer::MatRef;

use super::CalibrationMode;

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
    fn area(&self) -> usize;
    fn match_areas(&mut self, other: &mut Self);
    fn mask_slice(&self) -> &[bool];
    fn mask(&self, data: &[f64]) -> Vec<f64>;
    fn n_cols(&self) -> usize;
    fn n_rows(&self) -> usize;
    fn mat_ref(&self) -> MatRef<'_, f64>;
    fn n_mode(&self) -> usize;
    fn mode(&self) -> M;
    fn smode(&self) -> (u8, M) {
        (self.sid(), self.mode())
    }
    fn normalize(&mut self) -> f64;
    fn norm_l2(&mut self) -> f64;
}

pub trait Block {
    /// Block matrix
    ///
    /// Creates a block matrix from a nested array such as
    /// `[[A,B];[C,D]]` becomes
    /// ```
    /// | A B |
    /// | C D |
    /// ```
    fn block(array: &[&[&Self]]) -> Self
    where
        Self: Sized;
}

pub trait Modality: std::fmt::Debug + Clone {
    fn n_cols(&self) -> usize;
    fn fill(&self, iter: impl Iterator<Item = f64>) -> Vec<f64>;
}
