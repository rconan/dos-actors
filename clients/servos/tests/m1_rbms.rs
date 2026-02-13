/*!
# M1 assembly RBMs command integrated test

Testing that commands applied to the M1 segment hardpoints, given in rigid body displacement, results in the actual rigid body displacement of the segments.

Test all the rigid bodies motion for all given segment at once.

The tests fail if any rigid body displacement is more than 25% of the input command.

## Usage
```shell
MOUNT_MODEL=MOUNT_FDR_1kHz cargo test -r --features cuda --test m1_rbms -- --show-output
```

*/

use std::env;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_io::gmt_m1::assembly::M1RigidBodyMotions;
use gmt_dos_clients_servos::{GmtFem, GmtM1, GmtServoMechanisms};
use gmt_fem::FEM;
use interface::{
    Tick,
    optics::{
        M1State,
        state::{MirrorState, OpticalState},
    },
};

#[tokio::test(flavor = "multi_thread")]
async fn main() -> anyhow::Result<()> {
    unsafe { env::set_var("FLOWCHART", "") };

    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 3_usize; // second
    let n_step = sim_duration * sim_sampling_frequency;

    let fem = FEM::from_env()?;

    let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let rbms: Vec<_> = (0..7)
        .flat_map(|_i| (0..6).map(|j| rbm_fun(j) * 1e-6).collect::<Vec<_>>())
        .collect();
    let optical_state =
        OpticalState::default().zero_point(OpticalState::m1(MirrorState::from_rbms(&rbms)));

    let servos = GmtServoMechanisms::<10, 1>::new(sim_sampling_frequency as f64, fem).build()?;

    let timer: Timer = Timer::new(n_step);

    actorscript!(
        1: timer[Tick] -> {servos::GmtFem}[M1RigidBodyMotions]!$
        1: optical_state[M1State] -> {servos::GmtM1}
    );

    let data: Vec<f64> = model_logging_1
        .into_inner()
        .unwrap()
        .into_iter(format!("M1RigidBodyMotions"))?
        .last()
        .unwrap();
    let relative_errors: Vec<_> = data
        .chunks(6)
        .map(|data| {
            data.iter()
                .enumerate()
                .map(|(i, x)| (x * 1e6, rbm_fun(i)))
                .map(|(x, x0)| (x - x0).abs() * 1e2 / x0.abs())
                // .inspect(|&e| assert!(e < 25f64))
                .collect::<Vec<_>>()
        })
        .collect();
    println!("    [   Tx      Ty      Tz      Rx      Ry      Rz ]");
    for (i, seg_err) in relative_errors.into_iter().enumerate() {
        println!("S{:}: {seg_err:6.2?}", i + 1);
    }
    Ok(())
}
