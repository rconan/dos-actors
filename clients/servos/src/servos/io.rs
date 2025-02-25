use gmt_dos_actors::{
    prelude::Actor,
    system::{SystemInput, SystemOutput},
};
use gmt_dos_clients_fem::{solvers::ExponentialMatrix, DiscreteModalSolver};

use gmt_dos_clients_m2_ctrl::AsmsPositioners;
use gmt_dos_clients_mount::Mount;

use super::GmtServoMechanisms;

// FEM inputs
impl<const M1_RATE: usize, const M2_RATE: usize>
    SystemInput<DiscreteModalSolver<ExponentialMatrix>, 1, 1>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    fn input(&mut self) -> &mut Actor<DiscreteModalSolver<ExponentialMatrix>, 1, 1> {
        &mut self.fem
    }
}
// FEM outputs
impl<const M1_RATE: usize, const M2_RATE: usize>
    SystemOutput<DiscreteModalSolver<ExponentialMatrix>, 1, 1>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    fn output(&mut self) -> &mut Actor<DiscreteModalSolver<ExponentialMatrix>, 1, 1> {
        &mut self.fem
    }
}

// Mount inputs
impl<const M1_RATE: usize, const M2_RATE: usize> SystemInput<Mount, 1, 1>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    fn input(&mut self) -> &mut Actor<Mount, 1, 1> {
        &mut self.mount
    }
}

// AsmsPositioners inputs
impl<const M1_RATE: usize, const M2_RATE: usize> SystemInput<AsmsPositioners, 1, 1>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    fn input(&mut self) -> &mut Actor<AsmsPositioners, 1, 1> {
        &mut self.m2_positioners
    }
}

// M1 inputs
impl<const M1_RATE: usize, const M2_RATE: usize>
    SystemInput<gmt_dos_systems_m1::assembly::DispatchIn, 1, 1>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    fn input(&mut self) -> &mut Actor<gmt_dos_systems_m1::assembly::DispatchIn, 1, 1> {
        self.m1.input()
    }
}

// M2 inputs
impl<const M1_RATE: usize, const M2_RATE: usize> SystemInput<gmt_dos_systems_m2::DispatchIn, 1, 1>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    fn input(&mut self) -> &mut Actor<gmt_dos_systems_m2::DispatchIn, 1, 1> {
        self.m2.input()
    }
}
