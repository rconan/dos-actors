//! M1 segment rigid body motions

use super::prelude::*;
use gmt_dos_clients_io::gmt_m1::segment::{M1S, RBM};

impl<const ID: u8, S> interface::Size<RBM<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        6
    }
}

impl<const ID: u8, S> Write<RBM<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<RBM<ID>>> {
        let a: usize = (ID * 6).into();
        <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self)
            .as_ref()
            .map(|data| Data::new((data[a - 6..a]).to_vec()))
    }
}

impl<S, const ID: u8, const DOF: u8> Write<M1S<ID, DOF>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1S<ID, DOF>>> {
        let a: usize = (ID * 6 - 6 + DOF).into();
        <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self)
            .as_ref()
            .map(|data| vec![data[a]].into())
    }
}
