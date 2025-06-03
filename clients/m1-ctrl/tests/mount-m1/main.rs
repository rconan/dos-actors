use gmt_dos_actors::prelude::*;
use gmt_dos_clients::{Logging, Signal, Signals};
use gmt_dos_clients_fem::{
    DiscreteModalSolver, ExponentialMatrix, fem_io::actors_outputs::OSSM1Lcl,
};
use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
use gmt_dos_clients_m1_ctrl::{Calibration, Segment};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use std::env;

const ACTUATOR_RATE: usize = 10;

/*
export FEM_REPO=/fsx/20230530_1756_zen_30_M1_202110_FSM_202305_Mount_202305_noStairs/
cargo test --release  --package gmt_dos-clients_m1-ctrl --test mount-m1 -- main --exact --nocapture
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
    println!("{fem}");
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
    println!("{fem_dss}");

    let mut plant: Actor<_> = Actor::new(fem_dss.into_arcx()).name(format!(
        "GMT
    Finite Element Model
    {}",
        env::var("FEM_REPO").unwrap()
    ));
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
    let mut m1: Model<model::Unknown> = Default::default();
    let mut setpoints: Model<model::Unknown> = Default::default();
    for sid in sids {
        match sid {
            i if i == 1 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, 1> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += Segment::<1, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
            }
            i if i == 2 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, 1> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += Segment::<2, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
            }
            i if i == 3 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, 1> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += Segment::<3, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
            }
            i if i == 4 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, 1> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += Segment::<4, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
            }
            i if i == 5 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, 1> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += Segment::<5, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
            }
            i if i == 6 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, 1> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += Segment::<6, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
            }
            i if i == 7 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, 1> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += Segment::<7, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
            }
            _ => unimplemented!("Segments ID must be in the range [1,7]"),
        }
    }

    // MOUNT CONTROL
    let mut mount_setpoint: Initiator<_> = (
        Signals::new(3, n_step),
        "Mount
    Setpoint",
    )
        .into();
    let mount: Actor<_> = Mount::builder(&mut mount_setpoint).build(&mut plant)?;
    setpoints += mount_setpoint;

    let plant_logging = Logging::<f64>::new(1).into_arcx();
    let mut plant_logger: Terminator<_> = Actor::new(plant_logging.clone());

    plant
        .add_output()
        .bootstrap()
        .unbounded()
        .build::<M1RigidBodyMotions>()
        .into_input(&mut plant_logger)?;

    (model!(plant, plant_logger) + mount + m1 + setpoints)
        .name("mount-m1")
        .flowchart()
        .check()?
        .run()
        .await?;

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
