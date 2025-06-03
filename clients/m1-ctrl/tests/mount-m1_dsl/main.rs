use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Logging, Signal, Signals};
use gmt_dos_clients_fem::{
    DiscreteModalSolver, ExponentialMatrix, fem_io::actors_outputs::OSSM1Lcl,
};
use gmt_dos_clients_io::{
    gmt_m1::{
        M1RigidBodyMotions,
        segment::{
            ActuatorAppliedForces, ActuatorCommandForces, HardpointsForces, HardpointsMotion, RBM,
        },
    },
    mount::{MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_m1_ctrl::{Calibration, subsystems::M1Assembly};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 10;

/*
export FEM_REPO=/home/ubuntu/mnt/20230530_1756_zen_30_M1_202110_FSM_202305_Mount_202305_noStairs/
cargo test --release  --package gmt_dos-clients_m1-ctrl --test mount-m1_dsl -- main --exact --nocapture
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

    let sids = vec![1, 2, 3, 4, 5, 6, 7];
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
    let rbm_signal = |sid: u8| -> Signals {
        (0..6).fold(Signals::new(6, n_step), |signals, i| {
            signals.channel(
                i,
                Signal::Sigmoid {
                    amplitude: rbm_fun(i, sid) * 1e-6,
                    sampling_frequency_hz: sim_sampling_frequency as f64,
                },
            )
        })
    };

    let calibration = &m1_calibration;

    let [rbm1, rbm2, rbm3, rbm4, rbm5, rbm6, rbm7] = [
        rbm_signal(1),
        rbm_signal(2),
        rbm_signal(3),
        rbm_signal(4),
        rbm_signal(5),
        rbm_signal(6),
        rbm_signal(7),
    ];

    let actuators1 = Signals::new(335, n_step);
    let [actuators2, actuators3, actuators4, actuators5, actuators6] = [
        actuators1.clone(),
        actuators1.clone(),
        actuators1.clone(),
        actuators1.clone(),
        actuators1.clone(),
    ];
    let actuators7 = Signals::new(306, n_step);

    let (mut s1, mut s2, mut s3, mut s4, mut s5, mut s6, mut s7) =
        M1Assembly::<ACTUATOR_RATE>::new(calibration)?;

    // MOUNT CONTROL
    let mount_setpoint = Signals::new(3, n_step);
    let mount = Mount::new();

    let plant_logging = Logging::<f64>::new(1);

    actorscript! {
        1: mount_setpoint[MountSetPoint] -> mount[MountTorques] -> plant[MountEncoders]! -> mount

        1: rbm1[RBM<1>] -> {s1}[HardpointsForces<1>] -> plant[HardpointsMotion<1>]! -> {s1}
        1: actuators1[ActuatorCommandForces<1>] -> {s1}[ActuatorAppliedForces<1>] -> plant

        1: rbm2[RBM<2>] -> {s2}[HardpointsForces<2>] -> plant[HardpointsMotion<2>]! -> {s2}
        1: actuators2[ActuatorCommandForces<2>] -> {s2}[ActuatorAppliedForces<2>] -> plant

        1: rbm3[RBM<3>] -> {s3}[HardpointsForces<3>] -> plant[HardpointsMotion<3>]! -> {s3}
        1: actuators3[ActuatorCommandForces<3>] -> {s3}[ActuatorAppliedForces<3>] -> plant

        1: rbm4[RBM<4>] -> {s4}[HardpointsForces<4>] -> plant[HardpointsMotion<4>]! -> {s4}
        1: actuators4[ActuatorCommandForces<4>] -> {s4}[ActuatorAppliedForces<4>] -> plant

        1: rbm5[RBM<5>] -> {s5}[HardpointsForces<5>] -> plant[HardpointsMotion<5>]! -> {s5}
        1: actuators5[ActuatorCommandForces<5>] -> {s5}[ActuatorAppliedForces<5>] -> plant

        1: rbm6[RBM<6>] -> {s6}[HardpointsForces<6>] -> plant[HardpointsMotion<6>]! -> {s6}
        1: actuators6[ActuatorCommandForces<6>] -> {s6}[ActuatorAppliedForces<6>] -> plant

        1: rbm7[RBM<7>] -> {s7}[HardpointsForces<7>] -> plant[HardpointsMotion<7>]! -> {s7}
        1: actuators7[ActuatorCommandForces<7>] -> {s7}[ActuatorAppliedForces<7>] -> plant

        1: plant[M1RigidBodyMotions].. -> &plant_logging
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
