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
use std::ops::Deref;

#[cfg(fem)]
pub use fem::*;

/// M1 segment singular modes (aka bending modes)
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub struct SegmentSingularModes {
    /// segmennt meshes vertex coordinates `[x,y,z]`
    mode_nodes: Vec<Vec<f64>>,
    /// segment actuator location `[x,y,z]`
    actuator_nodes: Vec<Vec<f64>>,
    /// segment left singular modes
    raw_modes: Vec<f64>,
    /// segment left singular modes restricted to the rigid body motions null space
    modes: Vec<f64>,
    /// modes to forces matrix transform
    mode_2_force: Vec<f64>,
    // /// segment right singular modes
    // right_modes: Vec<f64>,
    // /// segment singular values
    // singular_values: Vec<f64>,
    /// modes shape `[n_points,n_actuators]`
    shape: (usize, usize),
}

impl SegmentSingularModes {
    pub fn new(
        mode_nodes: Vec<Vec<f64>>,
        actuator_nodes: Vec<Vec<f64>>,
        raw_modes: Vec<f64>,
        modes: Vec<f64>,
        mode_2_force: Vec<f64>,
        // singular_values: Vec<f64>,
        shape: (usize, usize),
    ) -> Self {
        Self {
            mode_nodes,
            actuator_nodes,
            raw_modes,
            modes,
            mode_2_force,
            // right_modes,
            // singular_values,
            shape,
        }
    }
    #[cfg(feature = "faer")]
    pub fn mat_ref(&self) -> faer::mat::MatRef<'_, f64> {
        let (ns, na) = self.shape;
        faer::mat::MatRef::from_column_major_slice(&self.raw_modes, ns, na)
    }
    #[cfg(feature = "faer")]
    pub fn mode2force_mat_ref(&self) -> faer::mat::MatRef<'_, f64> {
        let (_, na) = self.shape;
        let ns = self.mode_2_force.len() / na;
        faer::mat::MatRef::from_column_major_slice(&self.mode_2_force, na, ns)
    }
    pub fn mode2force(&self) -> nalgebra::DMatrix<f64> {
        // let n = self.singular_values.len();
        // let singular_values_inverse =
        //     nalgebra::DMatrix::from_diagonal(&nalgebra::DVector::from_iterator(
        //         self.singular_values.len(),
        //         self.singular_values.iter().map(|x| x.recip()),
        //     ));
        // nalgebra::DMatrix::from_column_slice(self.right_modes.len() / n, n, &self.right_modes)
        //     * singular_values_inverse
        let (_, na) = self.shape;
        let ns = self.mode_2_force.len() / na;
        nalgebra::DMatrix::from_column_slice(na, ns, &self.mode_2_force)
    }
    pub fn raw_modes_into_mat(&self) -> nalgebra::DMatrix<f64> {
        let (ns, na) = self.shape;
        nalgebra::DMatrix::from_column_slice(ns, na, &self.raw_modes)
    }
    pub fn modes_into_mat(&self, n_mode: Option<usize>) -> nalgebra::DMatrix<f64> {
        let (ns, ..) = self.shape;
        if let Some(n) = n_mode {
            nalgebra::DMatrix::from_column_slice(ns, n, &self.modes[..ns * n])
        } else {
            nalgebra::DMatrix::from_column_slice(ns, self.modes.len() / ns, &self.modes)
        }
    }
    pub fn shape(&self) -> (usize, usize) {
        self.shape
    }
    pub fn raw_modes_iter(&self) -> impl Iterator<Item = &f64> {
        self.raw_modes.iter()
    }
}

/// M1 singular modes (aka bending modes)
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub struct SingularModes(Vec<SegmentSingularModes>);
impl Deref for SingularModes {
    type Target = [SegmentSingularModes];

    fn deref(&self) -> &Self::Target {
        self.0.as_slice()
    }
}
impl SingularModes {
    pub fn push(&mut self, segment: SegmentSingularModes) {
        self.0.push(segment);
    }
    pub fn modes_into_mat(&self, n_mode: Option<usize>) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter()
            .map(|segment| segment.modes_into_mat(n_mode))
            .collect()
    }
    pub fn raw_modes_into_mat(&self) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter()
            .map(|segment| segment.raw_modes_into_mat())
            .collect()
    }
    pub fn mode2force(&self) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter().map(|segment| segment.mode2force()).collect()
    }
}
