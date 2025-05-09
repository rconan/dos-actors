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
let hardpoints = Hardpoints::new(
    m1_calibration.stiffness,
    m1_calibration.rbm_2_hp[S1 as usize - 1],
);
// Loadcells
let loadcell = LoadCells::new(
    m1_calibration.stiffness,
    m1_calibration.lc_2_cg[S1 as usize - 1],
);
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

mod actuators;
mod hardpoints;

pub use actuators::Actuators;
pub use hardpoints::{Hardpoints, LoadCells};

#[cfg(fem)]
mod calibration;
#[cfg(fem)]
pub use calibration::Calibration;

// #[cfg(fem)]
// mod fem;
// #[cfg(fem)]
// pub use fem::*;

// #[cfg(fem)]
// pub enum M1<const ACTUATOR_RATE: usize> {}
// #[cfg(fem)]
// impl<const ACTUATOR_RATE: usize> M1<ACTUATOR_RATE> {
//     pub fn new(
//         calibration: &Calibration,
//     ) -> anyhow::Result<gmt_dos_actors::system::Sys<assembly::M1<ACTUATOR_RATE>>> {
//         Ok(
//             gmt_dos_actors::system::Sys::new(assembly::M1::<ACTUATOR_RATE>::new(calibration)?)
//                 .build()?,
//         )
//     }
// }
