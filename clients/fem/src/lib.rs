//! This module is used to build the state space model of the telescope structure
//!
//! A state space model is represented by the structure [DiscreteModalSolver] that is created using the builder [`DiscreteStateSpace`].
//! The transformation of the FEM continuous 2nd order differential equation
//! into a discrete state space model is performed by the [Exponential] structure
//! (for the details of the transformation see the `exponential` module ).
//!
//! # Example
//! The following example loads a FEM model and converts it into a state space model
//! setting the sampling rate and the damping coefficients and truncating the eigen frequencies.
//! A single input and a single output are selected.
//! ```no_run
//! use gmt_fem::FEM;
//! use gmt_dos_clients_fem::{DiscreteStateSpace, DiscreteModalSolver,
//!               fem_io::{actors_inputs::OSSM1Lcl6F, actors_outputs::OSSM1Lcl}};
//! #[cfg(not(feature = "cuda"))]
//! use gmt_dos_clients_fem::solvers::Exponential;
//! #[cfg(feature = "cuda")]
//! use gmt_dos_clients_fem::solvers::{CuStateSpace, Exponential};
//!
//! # fn main() -> anyhow::Result<()> {
//!     let sampling_rate = 1e3; // Hz
//!     let fem = FEM::from_env()?;
//!   #[cfg(not(feature = "cuda"))]
//!     let mut fem_ss: DiscreteModalSolver<Exponential> = DiscreteStateSpace::from(fem)
//!         .sampling(sampling_rate)
//!         .proportional_damping(2. / 100.)
//!         .ins::<OSSM1Lcl6F>()
//!         .outs::<OSSM1Lcl>()
//!         .build()?;
//!   #[cfg(feature = "cuda")]
//!     let mut fem_ss: DiscreteModalSolver<CuStateSpace> = DiscreteStateSpace::<Exponential>::from(fem)
//!         .sampling(sampling_rate)
//!         .proportional_damping(2. / 100.)
//!         .ins::<OSSM1Lcl6F>()
//!         .outs::<OSSM1Lcl>()
//!         .build()?;
//! # Ok::<(), anyhow::Error>(())
//! # }
//! ```

use interface::UniqueIdentifier;
use std::ops::Range;

mod discrete_state_space;
pub mod solvers;
pub use discrete_state_space::{DiscreteStateSpace, StateSpaceError};
mod discrete_modal_solver;
pub use discrete_modal_solver::DiscreteModalSolver;
pub mod actors_interface;
#[cfg(feature = "serde")]
mod impl_serde;
mod model;
pub use model::{fem_io, Model, Switch};

use solvers::Solver;

/* #[cfg(feature = "serde")]
pub trait Solver: serde::Serialize + for<'a> serde::Deserialize<'a> {
    fn from_second_order(
        tau: f64,
        omega: f64,
        zeta: f64,
        continuous_bb: Vec<f64>,
        continuous_cc: Vec<f64>,
    ) -> Self;
    fn solve(&mut self, u: &[f64]) -> &[f64];
} */

pub trait Get<U: UniqueIdentifier> {
    fn get(&self) -> Option<Vec<f64>>;
}
impl<T, U> Get<U> for DiscreteModalSolver<T>
where
    // Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
    T: Solver + Default,
    U: 'static + UniqueIdentifier,
{
    fn get(&self) -> Option<Vec<f64>> {
        self.outs
            .iter()
            .find(|&x| x.as_any().is::<fem_io::SplitFem<U>>())
            .map(|io| self.y[io.range()].to_vec())
    }
}
pub trait Set<U: UniqueIdentifier> {
    fn set(&mut self, u: &[f64]);
    fn set_slice(&mut self, _u: &[f64], _range: Range<usize>) {
        unimplemented!()
    }
}
impl<T, U> Set<U> for DiscreteModalSolver<T>
where
    // Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
    T: Solver + Default,
    U: 'static + UniqueIdentifier,
{
    fn set(&mut self, u: &[f64]) {
        if let Some(io) = self
            .ins
            .iter()
            .find(|&x| x.as_any().is::<fem_io::SplitFem<U>>())
        {
            self.u[io.range()].copy_from_slice(u);
        }
    }
    fn set_slice(&mut self, u: &[f64], range: Range<usize>) {
        if let Some(io) = self
            .ins
            .iter()
            .find(|&x| x.as_any().is::<fem_io::SplitFem<U>>())
        {
            self.u[io.range()][range].copy_from_slice(u);
        }
    }
}

#[cfg(feature = "serde")]
impl<S> interface::filing::Codec for DiscreteModalSolver<S> where
    S: Solver + Default + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>
{
}
