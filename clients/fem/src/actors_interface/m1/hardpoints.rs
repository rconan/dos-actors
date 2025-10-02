//! M1 segment hardpoints

use super::prelude::*;
use gmt_dos_clients_io::gmt_m1::segment::{HardpointsForces, HardpointsMotion};

impl<const ID: u8, S> Read<HardpointsForces<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<HardpointsForces<ID>>) {
        let a: usize = (ID * 6).into();
        <DiscreteModalSolver<S> as Set<fem_io::OSSHarpointDeltaF>>::set_slice(
            self,
            &data,
            a - 6..a,
        );
    }
}

#[cfg(m1_hp_force_extension)]
impl<const ID: u8, S> Write<HardpointsForces<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<HardpointsForces<ID>>> {
        let a: usize = (ID * 12).into();
        <DiscreteModalSolver<S> as Get<fem_io::OSSHardpointForce>>::get(self)
            .as_ref()
            .map(|data| Data::new((data[a - 12..a]).to_vec()))
    }
}

impl<const ID: u8, S> Write<HardpointsMotion<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<HardpointsMotion<ID>>> {
        let a: usize = (ID * 12).into();
        <DiscreteModalSolver<S> as Get<fem_io::OSSHardpointD>>::get(self)
            .as_ref()
            .map(|data| Data::new((data[a - 12..a]).to_vec()))
    }
}

#[cfg(m1_hp_force_extension)]
impl<const ID: u8, S> Read<HardpointsMotion<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<HardpointsMotion<ID>>) {
        let a: usize = (ID * 6).into();
        <DiscreteModalSolver<S> as Set<fem_io::OSSHardpointExtension>>::set_slice(
            self,
            &data,
            a - 6..a,
        );
    }
}
