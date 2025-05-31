use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{Logging, Signal, Signals};
use gmt_dos_clients_fem::{
    DiscreteModalSolver, ExponentialMatrix, fem_io::actors_outputs::OSSM1Lcl,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m1::assembly,
    mount::{MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_m1_ctrl::{Calibration, assembly::M1};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 10;

/*
export MOUNT_MODEL=MOUNT_FDR_1kHz
export FEM_REPO=~/mnt/20230530_1756_zen_30_M1_202110_FSM_202305_Mount_202305_noStairs/
cargo test --release  --package gmt_dos-clients_m1-ctrl --test mount-m1b_dsl -- main --exact --nocapture
 */

#[tokio::test]
async fn main() -> anyhow::Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let sim_sampling_frequency = 1000;
    let m1_freq = 100; // Hz
    assert!(m1_freq == sim_sampling_frequency / ACTUATOR_RATE);
    let sim_duration = 3_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = FEM::from_env()?;
    // println!("{fem}");
    let m1_calibration = Calibration::new(&mut fem);

    let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
    let fem_dss = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .truncate_hankel_singular_values(1e-7)
        .hankel_frequency_lower_bound(50.)
        .including_mount()
        .including_m1(Some(sids.clone()))?
        .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;
    // println!("{fem_dss}");

    let plant = fem_dss;
    // .image("../icons/fem.png");

    let rbm_fun =
        |i: usize, sid: u8| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64 + sid as f64 / 10_f64;
    let rbm = (1..=7).fold(Signals::new(6 * 7, n_step), |signals_sid, sid| {
        (0..6).fold(signals_sid, |signals, i| {
            signals.channel(
                i + 6 * (sid - 1) as usize,
                Signal::Sigmoid {
                    amplitude: rbm_fun(i, sid) * 1e-6,
                    sampling_frequency_hz: sim_sampling_frequency as f64,
                },
            )
        })
    });

    let calibration = &m1_calibration;

    let actuators = Signals::new(6 * 335 + 306, n_step);

    let m1 = Sys::new(M1::<ACTUATOR_RATE>::new(calibration)?).build()?;

    // MOUNT CONTROL
    let mount_setpoint = Signals::new(3, n_step);
    let mount = Mount::new();

    let plant_logging: Logging<f64> = Logging::<f64>::new(1);

    actorscript! {
        1: mount_setpoint[MountSetPoint] -> mount[MountTorques] -> plant[MountEncoders]! -> mount

        1: rbm[assembly::M1RigidBodyMotions]
            -> {m1}[assembly::M1HardpointsForces]
                -> plant[assembly::M1HardpointsMotion]! -> {m1}
        1: actuators[assembly::M1ActuatorCommandForces]
                -> {m1}[assembly::M1ActuatorAppliedForces] -> plant

        1: plant[M1RigidBodyMotions].. -> plant_logging
    }

    let rbm_err = (*plant_logging.lock().await)
        .chunks()
        .last()
        .unwrap()
        .chunks(6)
        .map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
        .enumerate()
        .inspect(|(i, x)| println!("{:2}: {:+.1?}", i, x))
        .map(|(i, x)| {
            x.iter()
                .enumerate()
                .map(|(j, x)| x - rbm_fun(j, i as u8 + 1))
                .map(|x| x * x)
                .sum::<f64>()
                / 6f64
        })
        .map(|x| x.sqrt())
        .sum::<f64>()
        / 7f64;
    assert!(dbg!(rbm_err) < 5e-2);

    Ok(())
}
