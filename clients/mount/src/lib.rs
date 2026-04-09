/*!
# GMT mount control model

A [gmt_dos-actors] client for the GMT mount control system.

## Example

Commanding the elevation axis of the mount to a 1arcsec offset.

```no_run
// Dependencies:
//  * tokio
//  * gmt_dos_actors
//  * gmt_dos_clients
//  * gmt_dos_clients_io
//  * gmt_dos_clients_arrow
//  * gmt_dos_clients_fem
//  * gmt-fem
//  * gmt_dos_clients_mount
//  * gmt-lom
//  * skyangle
// Environment variables:
//  * FEM_REPO
//  * LOM

# tokio_test::block_on(async {
use gmt_dos_actors::actorscript;
use gmt_dos_clients::signals::{Signal, Signals};
use gmt_dos_clients_fem::{fem_io::actors_outputs::*, DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    mount::{MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use gmt_lom::{OpticalMetrics, LOM};
use skyangle::Conversion;
use serde::{Deserialize, Serialize};
use gmt_mount_ctrl_controller::MountController;
use gmt_mount_ctrl_driver::MountDriver;

let sim_sampling_frequency = 1000; // Hz
let sim_duration = 20_usize; // second
let n_step = sim_sampling_frequency * sim_duration;
// FEM MODEL
let state_space = {
    let fem = FEM::from_env()?;
    println!("{fem}");
    DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .use_static_gain_compensation()
        .build()?
};
println!("{state_space}");
// SET POINT
let setpoint = Signals::new(3, n_step).channel(1, Signal::Constant(1f64.from_arcsec()));
// MOUNT CONTROL
let mount = Mount::new();
actorscript! {
    #[model(state = completed)]
    1: setpoint[MountSetPoint]
        -> mount[MountTorques]
            -> state_space[MountEncoders]!
                -> mount
    1: state_space[M1RigidBodyMotions]$
    1: state_space[M2RigidBodyMotions]$
}
// Linear optical sensitivities to derive segment tip and tilt
let lom = LOM::builder()
    .rigid_body_motions_record(
        (*model_logging_1.lock().await).record()?,
        Some("M1RigidBodyMotions"),
        Some("M2RigidBodyMotions"),
    )?
    .build()?;
let segment_tiptilt = lom.segment_tiptilt();
let stt = segment_tiptilt.items().last().unwrap();
println!("Segment TT: {:.3?}mas", stt.to_mas());
# anyhow::Result::<()>::Ok(())
# });
```

[gmt_dos-actors]: https://docs.rs/gmt_dos-actors
*/

use gmt_mount_ctrl_controller::MountController;
use gmt_mount_ctrl_driver::MountDriver;
use interface::filing::Codec;
use serde::{Deserialize, Serialize};

/// Discrete sampling frequency [Hz] of the mount controller
pub fn sampling_frequency() -> usize {
    match env!("MOUNT_MODEL") {
        "MOUNT_PDR_8kHz" | "MOUNT_FDR_8kHz" => 8000,
        "MOUNT_FDR_1kHz" | "MOUNT_FDR_1kHz-az17Hz" => 1000,
        val => panic!("Unknown mount model: {val}"),
    }
}

// #[cfg(fem)]
// mod builder;
// #[cfg(fem)]
// pub use builder::Builder;

mod actors_interface;

/// GMT mount control model
///
// A [gmt_dos-actors] client for the GMT mount control system.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Mount {
    drive: MountDriver,
    control: MountController,
}
impl Mount {
    /// Returns the mount controller
    pub fn new() -> Self {
        Self {
            drive: MountDriver::new(),
            control: MountController::new(),
        }
    }
}

impl Codec for Mount {}
