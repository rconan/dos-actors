mod bilinear;
#[cfg(feature = "cuda")]
mod cuda_solver;
mod exponential;
mod exponential_matrix;

pub use bilinear::Bilinear;
#[cfg(feature = "cuda")]
pub use cuda_solver::{CuStateSpace, ModeStateSpace};
pub use exponential::Exponential;
pub use exponential_matrix::ExponentialMatrix;

pub trait Solver: Send + Sync {
    fn from_second_order(
        _tau: f64,
        _omega: f64,
        _zeta: f64,
        _continuous_bb: Vec<f64>,
        _continuous_cc: Vec<f64>,
    ) -> Self
    where
        Self: Sized + Default,
    {
        Default::default()
    }
    fn solve<'a>(&'a mut self, u: &'a [f64]) -> &'a [f64] {
        u
    }
    fn get_b(&self) -> &[f64];
    fn get_c(&self) -> &[f64];
    fn n_input(&self) -> usize;
    fn n_output(&self) -> usize;
}
