//! CFD

use super::prelude::*;
use gmt_dos_clients_io::cfd_wind_loads::{CFDM1WindLoads, CFDMountWindLoads};

/// mount
impl<S> Read<CFDMountWindLoads> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    #[cfg(cfd2021)]
    fn read(&mut self, data: Data<CFDMountWindLoads>) {
        <DiscreteModalSolver<S> as Set<fem_io::CFD2021106F>>::set(self, &data)
    }
    #[cfg(cfd2025)]
    fn read(&mut self, data: Data<CFDMountWindLoads>) {
        <DiscreteModalSolver<S> as Set<fem_io::CFD2025046F>>::set(self, &data)
    }
}
/// M1
impl<S> Read<CFDM1WindLoads> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<CFDM1WindLoads>) {
        <DiscreteModalSolver<S> as Set<fem_io::OSSM1Lcl6F>>::set(self, &data)
    }
}

// #[cfg(any(feature = "asm", feature = "fsm"))]
use gmt_dos_clients_io::cfd_wind_loads::CFDM2WindLoads;
/// M2
// #[cfg(feature = "asm")]
#[cfg(all(fem, topend = "ASM"))]
impl<S> Read<CFDM2WindLoads> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<CFDM2WindLoads>) {
        <DiscreteModalSolver<S> as Set<fem_io::MCM2Lcl6F>>::set(self, &data)
    }
}
#[cfg(all(fem, topend = "FSM"))]
impl<S> Read<CFDM2WindLoads> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<CFDM2WindLoads>) {
        <DiscreteModalSolver<S> as Set<fem_io::MCM2Lcl6F>>::set(self, &data)
    }
}
