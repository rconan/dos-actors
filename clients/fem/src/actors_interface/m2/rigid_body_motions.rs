//! M2 rigid body motions

use super::prelude::*;
use gmt_dos_clients_io::gmt_m2::M2RigidBodyMotions;

impl<S> Size<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        42
    }
}
#[cfg(all(fem, topend = "FSM"))]
impl<S> Write<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2Lcl6D>>::get(self).map(|data| Data::new(data))
    }
}
#[cfg(all(fem, topend = "ASM"))]
impl<S> Write<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2RB6D>>::get(self).map(|data| Data::new(data))
    }
}
#[cfg(all(fem, m2_rbm = "MCM2Lcl"))]
impl<S> Write<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2Lcl>>::get(self).map(|data| Data::new(data))
    }
}
