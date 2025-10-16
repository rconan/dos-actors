/*!
# M1 RBM command integrated tests

Test that each command applied to the M1 segment hardpoints, given in rigid body displacement, results in the actual rigid body displacement of the segment.

Test all the rigid bodies motion at once for a given segment, and there is one test per segment.

The tests fail if any rigid body displacement is more than 10% of the input command.

## Usage
```shell
cargo test -r --test main -- --no-capture
```

*/

use gmt_dos_actors::actorscript;
use gmt_dos_clients::signals::Signals;
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::outputs::OSSM1Lcl,
    gmt_m1::segment::{
        ActuatorAppliedForces, ActuatorCommandForces, BarycentricForce, HardpointsForces,
        HardpointsMotion, RBM,
    },
    mount::{MountEncoders, MountTorques},
};
use gmt_dos_clients_m1_ctrl::{Actuators, Calibration, Hardpoints, LoadCells};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use interface::Size;
use std::env;

async fn m1_segment_control_system<const SX: u8>() -> anyhow::Result<Vec<f64>> {
    println!("M1 S{SX}");

    unsafe { env::set_var("FLOWCHART", "") };

    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 10_usize; // second
    let mut whole_fem = FEM::from_env()?;
    let n_step = sim_sampling_frequency * sim_duration;

    let m1_calibration = Calibration::new(&mut whole_fem);
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(whole_fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .including_m1(Some(vec![SX]))?
        .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;

    let mount = Mount::new();

    let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let hp_setpoint = (0..6).fold(Signals::new(6, n_step), |signals, i| {
        signals.channel(
            i,
            rbm_fun(i) * 1e-6, // Signal::Sigmoid {
                               // Signal::Sigmoid {
                               //     amplitude: rbm_fun(i) * 1e-6,
                               //     sampling_frequency_hz: sim_sampling_frequency as f64,
                               // },
        )
    });
    // Hardpoints
    let hardpoints = Hardpoints::<SX>::from(&m1_calibration);
    // Loadcells
    let loadcell = LoadCells::<SX>::from(&m1_calibration);
    // Actuators
    let actuators = Actuators::<SX>::new();
    let actuators_setpoint =
        Signals::new(Size::<ActuatorCommandForces<SX>>::len(&actuators), n_step);

    #[cfg(m1_hp_force_extension)]
    actorscript! {
        #[labels(
            plant = "Plant",
            hp_setpoint = "RBM",
            actuators_setpoint = "Actuators\nSetpoint")]
        1: mount[MountTorques] -> plant[MountEncoders]! -> mount
        1: actuators[ActuatorAppliedForces<SX>]
            -> plant[HardpointsForces<SX>]!
                -> loadcell
        1: hp_setpoint[RBM<SX>]
            -> hardpoints[HardpointsMotion<SX>] -> plant
        1: plant[RBM<SX>]!$
        10: actuators_setpoint[ActuatorCommandForces<SX>] -> actuators
        10: loadcell[BarycentricForce<SX>] -> actuators
    }
    #[cfg(not(m1_hp_force_extension))]
    actorscript! {
        #[labels(
            plant = "Plant",
            hp_setpoint = "RBM",
            actuators_setpoint = "Actuators\nSetpoint")]
        1: mount[MountTorques] -> plant[MountEncoders]! -> mount
        1: hp_setpoint[RBM<SX>]
            -> hardpoints[HardpointsForces<SX>]
                -> loadcell
        1: hardpoints[HardpointsForces<SX>]
            -> plant[RBM<SX>]!$
        1: actuators[ActuatorAppliedForces<SX>]
            -> plant[HardpointsMotion<SX>]!
                -> loadcell
        10: actuators_setpoint[ActuatorCommandForces<SX>] -> actuators
        10: loadcell[BarycentricForce<SX>]! -> actuators
    }

    let data: Vec<f64> = model_logging_1
        .into_inner()
        .unwrap()
        .into_iter(format!("RBM<{SX}>"))?
        .last()
        .unwrap();
    let relative_errors: Vec<_> = data
        .iter()
        .enumerate()
        .map(|(i, x)| (x * 1e6, rbm_fun(i)))
        .map(|(x, x0)| (x - x0).abs() * 1e2 / x0.abs())
        .inspect(|&e| assert!(e < 10f64))
        .collect();
    Ok(relative_errors)
}

#[tokio::test(flavor = "multi_thread")]
async fn main() -> anyhow::Result<()> {
    let mut relative_errors = vec![];
    relative_errors.push(m1_segment_control_system::<1>().await?);
    relative_errors.push(m1_segment_control_system::<2>().await?);
    relative_errors.push(m1_segment_control_system::<3>().await?);
    relative_errors.push(m1_segment_control_system::<4>().await?);
    relative_errors.push(m1_segment_control_system::<5>().await?);
    relative_errors.push(m1_segment_control_system::<6>().await?);
    relative_errors.push(m1_segment_control_system::<7>().await?);
    println!("    [   Tx      Ty      Tz      Rx      Ry      Rz ]");
    for (i, seg_err) in relative_errors.into_iter().enumerate() {
        println!("S{:}: {seg_err:6.2?}", i + 1);
    }
    Ok(())
}
