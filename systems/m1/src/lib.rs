/*!
# M1 control system

A [gmt_dos-actors] client for the GMT M1 control system.

## Examples

### Single segment

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

# tokio_test::block_on(async {
use gmt_dos_actors::actorscript;
use interface::Size;
use gmt_dos_clients::{logging::Logging, signals::{Signal, Signals}};
use gmt_dos_clients_fem::{fem_io::actors_inputs::*, fem_io::actors_outputs::*};
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::gmt_m1::segment::{
    ActuatorAppliedForces, ActuatorCommandForces, BarycentricForce, HardpointsForces,
    HardpointsMotion, RBM,
};
use gmt_dos_clients_m1_ctrl::{Actuators, Calibration, Hardpoints, LoadCells};
use gmt_fem::FEM;

const S1: u8 = 1;

let sim_sampling_frequency = 1000;
let sim_duration = 10_usize; // second
let n_step = sim_sampling_frequency * sim_duration;
let mut whole_fem = FEM::from_env()?;
let m1_calibration = Calibration::new(&mut whole_fem);
let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(whole_fem)
    .sampling(sim_sampling_frequency as f64)
    .proportional_damping(2. / 100.)
    .including_m1(Some(vec![1]))?
    .outs::<OSSM1Lcl>()
    .use_static_gain_compensation()
    .build()?;

let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
let hp_setpoint = (0..6).fold(Signals::new(6, n_step), |signals, i| {
    signals.channel(
        i,
        Signal::Sigmoid {
            amplitude: rbm_fun(i) * 1e-6,
            sampling_frequency_hz: sim_sampling_frequency as f64,
        },
    )
});
// Hardpoints
let hardpoints = Hardpoints::<S1>::from(&m1_calibration);
// Loadcells
let loadcell = LoadCells::<S1>::from(&m1_calibration);
// Actuators
let actuators = Actuators::<S1>::new();
let actuators_setpoint = Signals::new(
    Size::<ActuatorCommandForces<S1>>::len(&Actuators::<S1>::new()),
    n_step,
);
actorscript! {
    #[model(state=completed)]
    #[labels(hp_setpoint="RBM",actuators_setpoint="Actuators")]
    1: hp_setpoint[RBM<S1>]
        -> hardpoints[HardpointsForces<S1>]
            -> loadcell
    1: hardpoints[HardpointsForces<S1>]
        -> plant[RBM<S1>]$
    1: actuators[ActuatorAppliedForces<S1>]
        -> plant[HardpointsMotion<S1>]!
            -> loadcell
    10: actuators_setpoint[ActuatorCommandForces<S1>] -> actuators
    10: loadcell[BarycentricForce<S1>]! -> actuators
};

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
    pub fn mat_ref(&self) -> faer::mat::MatRef<f64> {
        let (ns, na) = self.shape;
        faer::mat::MatRef::from_column_major_slice(&self.raw_modes, ns, na)
    }
    #[cfg(feature = "faer")]
    pub fn mode2force_mat_ref(&self) -> faer::mat::MatRef<f64> {
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
    pub fn modes_into_mat(&self) -> nalgebra::DMatrix<f64> {
        let (ns, ..) = self.shape;
        nalgebra::DMatrix::from_column_slice(ns, self.modes.len() / ns, &self.modes)
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
    pub fn modes_into_mat(&self) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter()
            .map(|segment| segment.modes_into_mat())
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
