/*!
# M1 assembly control system

A [gmt_dos-actors] client for the GMT M1 assembly control system.

There are 2 implementations of the M1 control system depending on the FEM input and output.

Latest FEM have a new hardpoints input `OSSHardpointExtension` and a new hardpoints output `OSSHardpointForce` that together emulates the behavior of the hardpoints loadcells.
For older FEM, the loadcells is modeled using the [LoadCells](gmt_dos_clients_m1_ctrl::LoadCells) `struct`.

The hardpoints and actuators controllers are also different according to the FEM model.
For older FEM, the controllers are modeled following the design implementation whereas
the latest model use the as-build implementation.

The switch between the controller models is done automatically based on the `m1_hp_force_extension` compiler flag which is enabled when both `OSSHardpointExtension` and `OSSHardpointForce` are available in the FEM inputs and outputs.

## Examples

### With a FEM dating from 2025 onward

```no_run
// Dependencies:
//  * tokio
//  * gmt_dos_actors
//  * gmt_dos_clients
//  * gmt_dos_clients_io
//  * gmt_dos_clients_fem
//  * gmt-fem
//  * gmt_dos_clients_m1_ctrl
// Environment variables:
//  * FEM_REPO

#[cfg(m1_hp_force_extension)]
# tokio_test::block_on(async {
use std::env;
use std::path::Path;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_fem::{
    DiscreteModalSolver, DiscreteStateSpace,
    solvers::{CuStateSpace, ExponentialMatrix},
};
use gmt_dos_clients_io::gmt_m1::assembly::{
    M1ActuatorAppliedForces, M1HardpointsForces, M1HardpointsMotion,
};
use gmt_dos_systems_m1::M1;
use gmt_fem::FEM;
use interface::Tick;

let sim_sampling_frequency = 1000;
let sim_duration = 10_usize; // second
let mut fem = FEM::from_env()?;

let m1 = M1::<10>::builder(&mut fem).build()?;

let plant: DiscreteModalSolver<CuStateSpace> =
    DiscreteStateSpace::<ExponentialMatrix>::from(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        // .including_mount()
        .including_m1(None)?
        // .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;

let timer: Timer = Timer::new(sim_duration * sim_sampling_frequency);

actorscript!(
    1: timer[Tick] -> plant
    1: {m1}[M1ActuatorAppliedForces]
        -> plant[M1HardpointsForces]!
            -> {m1}
    1: {m1}[M1HardpointsMotion] -> plant
);

# anyhow::Result::<()>::Ok(())
# });
```

### With a FEM older than 2025

```no_run
// Dependencies:
//  * tokio
//  * gmt_dos_actors
//  * gmt_dos_clients
//  * gmt_dos_clients_io
//  * gmt_dos_clients_fem
//  * gmt-fem
//  * gmt_dos_clients_m1_ctrl
// Environment variables:
//  * FEM_REPO

#[cfg(not(m1_hp_force_extension))]
# tokio_test::block_on(async {
use std::env;
use std::path::Path;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_fem::{
    DiscreteModalSolver, DiscreteStateSpace,
    solvers::{CuStateSpace, ExponentialMatrix},
};
use gmt_dos_clients_io::gmt_m1::assembly::{
    M1ActuatorAppliedForces, M1HardpointsForces, M1HardpointsMotion,
};
use gmt_dos_systems_m1::M1;
use gmt_fem::FEM;
use interface::Tick;

let sim_sampling_frequency = 1000;
let sim_duration = 10_usize; // second
let mut fem = FEM::from_env()?;

let m1 = M1::<10>::builder(&mut fem).build()?;

let plant: DiscreteModalSolver<CuStateSpace> =
    DiscreteStateSpace::<ExponentialMatrix>::from(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        // .including_mount()
        .including_m1(None)?
        // .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;

let timer: Timer = Timer::new(sim_duration * sim_sampling_frequency);

actorscript!(
    1: timer[Tick] -> plant
    1: {m1}[M1HardpointsForces]
        -> plant[M1HardpointsMotion]!
            -> {m1}[M1ActuatorAppliedForces]
                -> plant
);

# anyhow::Result::<()>::Ok(())
# });
```

[gmt_dos-actors]: https://docs.rs/gmt_dos-actors
*/

#[cfg(fem)]
mod fem;
mod singular_modes;

#[cfg(fem)]
pub use fem::*;

pub use singular_modes::{NullSpace, SegmentSingularModes, SingularModes};
