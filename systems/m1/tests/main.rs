/*!
# M1 assembly RBM command integrated test

Testing that commands applied to the M1 segment hardpoints, given in rigid body displacement, results in the actual rigid body displacement of the segments.

Test all the rigid bodies motion for all given segment at once.

The tests fail if any rigid body displacement is more than 25% of the input command.

## Usage
```shell
cargo test -r --features="gmt_dos-clients_fem/cuda" --test main -- --show-output
```

*/

use std::env;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::signals::Signals;
use gmt_dos_clients_fem::{
    DiscreteModalSolver, DiscreteStateSpace,
    fem_io::actors_outputs::OSSM1Lcl,
    solvers::{CuStateSpace, ExponentialMatrix},
};
use gmt_dos_clients_io::{
    gmt_m1::assembly::{
        M1ActuatorAppliedForces, M1HardpointsForces, M1HardpointsMotion, M1RigidBodyMotions,
    },
    mount::{MountEncoders, MountTorques},
};
use gmt_dos_clients_mount::Mount;
use gmt_dos_systems_m1::M1;
use gmt_fem::FEM;

#[tokio::test(flavor = "multi_thread")]
async fn main() -> anyhow::Result<()> {
    unsafe { env::set_var("FLOWCHART", "") };

    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 10_usize; // second
    let n_step = sim_duration * sim_sampling_frequency;

    let mut fem = FEM::from_env()?;

    let mount = Mount::new();

    let m1 = M1::<10>::builder(&mut fem).build()?;

    let plant: DiscreteModalSolver<CuStateSpace> =
        DiscreteStateSpace::<ExponentialMatrix>::from(fem)
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            .including_mount()
            .including_m1(None)?
            .outs::<OSSM1Lcl>()
            .use_static_gain_compensation()
            .build()?;

    let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let hp_setpoint = (0..7).fold(Signals::new(42, n_step), |signals, k| {
        (0..6).fold(signals, |signals, i| {
            signals.channel(i + 6 * k, rbm_fun(i) * 1e-6)
        })
    });

    #[cfg(m1_hp_force_extension)]
    actorscript!(
        1: mount[MountTorques] -> plant[MountEncoders]! -> mount
        1: hp_setpoint[M1RigidBodyMotions]
            -> {m1}[M1HardpointsMotion]
                -> plant[M1RigidBodyMotions]!$
        1: {m1}[M1ActuatorAppliedForces]
            -> plant[M1HardpointsForces]! -> {m1}
    );

    #[cfg(not(m1_hp_force_extension))]
    actorscript!(
        1: mount[MountTorques] -> plant[MountEncoders]! -> mount
        1: hp_setpoint[M1RigidBodyMotions]
            -> {m1}[M1HardpointsForces]
                -> plant[M1HardpointsMotion]!
                    -> {m1}[M1ActuatorAppliedForces]
                        -> plant[M1RigidBodyMotions]!$
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
                .inspect(|&e| assert!(e < 25f64))
                .collect::<Vec<_>>()
        })
        .collect();
    println!("    [   Tx      Ty      Tz      Rx      Ry      Rz ]");
    for (i, seg_err) in relative_errors.into_iter().enumerate() {
        println!("S{:}: {seg_err:6.2?}", i + 1);
    }
    Ok(())
}

// #[derive(Debug, Default, Clone)]
// pub struct ToVec {
//     data: Arc<Vec<Arc<Vec<f64>>>>,
// }
// impl Update for ToVec {}
// impl<U: UniqueIdentifier<DataType = Vec<Arc<Vec<f64>>>>> Read<U> for ToVec {
//     fn read(&mut self, data: Data<U>) {
//         self.data = data.into_arc();
//     }
// }
// impl<U: UniqueIdentifier<DataType = Vec<Arc<Vec<f64>>>>> Write<Flatten<U>> for ToVec {
//     fn write(&mut self) -> Option<Data<Flatten<U>>> {
//         let y: Vec<_> = self
//             .data
//             .iter()
//             .flat_map(|x| x.as_slice().to_vec())
//             .collect();
//         Some(y.into())
//     }
// }

// #[derive(UID)]
// pub struct Flatten<U: UniqueIdentifier<DataType = Vec<Arc<Vec<f64>>>>>(PhantomData<U>);
