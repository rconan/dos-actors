//! M2 positioner

use super::prelude::*;
use gmt_dos_clients_io::gmt_m2::{M2PositionerForces, M2PositionerNodes};

/// forces
impl<S> Read<M2PositionerForces> for DiscreteModalSolver<S>
where
    S: Solver + Default,
{
    fn read(&mut self, data: Data<M2PositionerForces>) {
        <DiscreteModalSolver<S> as Set<fem_io::MCM2SmHexF>>::set(self, &data)
    }
}
///  nodes
impl<S> Write<M2PositionerNodes> for DiscreteModalSolver<S>
where
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2PositionerNodes>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2SmHexD>>::get(self).map(|data| Data::new(data))
    }
}
