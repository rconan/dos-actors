use super::*;

impl<const ID: u8, S> Write<HardpointsForces<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<HardpointsForces<ID>>> {
        let a: usize = (ID * 6).into();
        <DiscreteModalSolver<S> as Get<fem_io::OSSHardpointForce>>::get(self)
            .as_ref()
            .map(|data| Data::new((data[a - 6..a]).to_vec()))
    }
}

impl<const ID: u8, S> Size<HardpointsForces<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        6
    }
}

impl<const ID: u8, S> Read<HardpointsMotion<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<HardpointsMotion<ID>>) {
        use crate::{Set, actors_interface::prelude::fem_io};

        let a: usize = (ID * 6).into();
        <DiscreteModalSolver<S> as Set<fem_io::OSSHardpointExtension>>::set_slice(
            self,
            &data,
            a - 6..a,
        );
    }
}

impl<const ID: u8, S> Size<HardpointsMotion<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        6
    }
}
