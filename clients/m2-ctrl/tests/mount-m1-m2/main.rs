use gmt_dos_actors::prelude::*;
use gmt_dos_clients::{Logging, Signal, Signals};
use gmt_dos_clients_fem::fem_io::actors_outputs::OSSM1Lcl;
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::asm::segment::VoiceCoilsMotion};
use gmt_dos_clients_m1_ctrl::{Calibration as M1Calibration, Segment as M1Segment};
use gmt_dos_clients_m2_ctrl::{Calibration as AsmsCalibration, Segment as AsmsSegment};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use nalgebra::DVector;
use nanorand::{Rng, WyRand};
use std::{env, path::Path};

/*
export FEM_REPO=/home/ubuntu/projects/dos-actors/grsim/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/
cargo test --release  --package gmt_dos-clients_m2-ctrl --features serde --test mount-m1-m2 -- main --exact --nocapture
 */

const ACTUATOR_RATE: usize = 80;

#[tokio::test]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let mut rng = WyRand::new();

    let sim_sampling_frequency = 8000;
    let m1_freq = 100; // Hz
    assert!(m1_freq == sim_sampling_frequency / ACTUATOR_RATE);
    let sim_duration = 3_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = FEM::from_env()?;
    println!("{fem}");
    let m1_calibration = M1Calibration::new(&mut fem);

    let n_mode = env::var("N_KL_MODE").map_or_else(|_| 66, |x| x.parse::<usize>().unwrap());
    let n_actuator = 675;

    let sids = vec![1, 2, 3, 4, 5, 6, 7];
    let calibration_file_name =
        Path::new(".").join(format!("asms_zonal_kl{n_mode}qr_calibration.bin"));
    let mut asms_calibration = if let Ok(data) = AsmsCalibration::try_from(&calibration_file_name) {
        data
    } else {
        let asms_calibration = AsmsCalibration::builder(
            n_mode,
            n_actuator,
            (
                "KLmodesQR.mat".to_string(),
                (1..=7).map(|i| format!("KL_{i}")).collect::<Vec<String>>(),
            ),
            &mut fem,
        )
        .stiffness("Zonal")
        .build()?;
        asms_calibration.save(&calibration_file_name)?;
        asms_calibration
    };
    asms_calibration.transpose_modes();

    let fem_dss = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        // .truncate_hankel_singular_values(1e-5)
        // .hankel_frequency_lower_bound(50.)
        .including_mount()
        .including_m1(Some(sids.clone()))?
        .including_asms(Some(sids.clone()), None, None)?
        .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;
    println!("{fem_dss}");

    let mut plant: Actor<_> = Actor::new(fem_dss.into_arcx())
        /*         .name(format!(
            "GMT
        Finite Element Model
        {}",
            env::var("FEM_REPO").unwrap()
        )) */
        .name("")
        .image("fem.png");

    let plant_logging = Logging::<f64>::new(sids.len() + 1).into_arcx();
    let mut plant_logger: Terminator<_> = Actor::new(plant_logging.clone());

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

    let mut aas = vec![];
    let mut asms_coefs = |sid: u8| {
        let mut c = 0;
        let mut n = 0;
        let mut a = vec![];
        for i in 0..n_mode {
            if c < (n + 1) {
                c += 1;
            } else {
                n += 1;
                c = 1;
            }
            a.push(1e-6 * (rng.generate::<f64>() * 2f64 - 1f64) / ((n + 1) as f64));
            // asm_coefs = asm_coefs.channel(i, gmt_dos_clients::Signal::Constant(a));
        }
        let m = asms_calibration.modes(Some(vec![sid]))[0];
        let mt = asms_calibration.modes_t(Some(vec![sid])).unwrap()[0];
        let u = DVector::from_column_slice(&a);
        let forces = m * u;
        let a_u = mt * &forces;
        aas.extend_from_slice(a_u.as_slice());
        forces
            .into_iter()
            .enumerate()
            .fold(Signals::new(n_actuator, n_step), |s, (i, f)| {
                s.channel(i, Signal::Constant(*f))
            })
    };

    let mut m1: Model<model::Unknown> = Default::default();
    let mut m2: Model<model::Unknown> = Default::default();
    let mut setpoints: Model<model::Unknown> = Default::default();
    for &sid in &sids {
        match sid {
            i if i == 1 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += M1Segment::<1, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
                let mut asm_setpoint: Initiator<_> = (
                    asms_coefs(i),
                    format!(
                        "ASM #{i}
      Set-Point"
                    ),
                )
                    .into();
                let client = asm_setpoint.client();
                (&mut *client.lock().await).progress();
                m2 += AsmsSegment::<1>::builder(
                    n_actuator,
                    asms_calibration.stiffness(i),
                    &mut asm_setpoint,
                )
                .build(&mut plant)?;
                setpoints += asm_setpoint;
                plant
                    .add_output()
                    .bootstrap()
                    .build::<VoiceCoilsMotion<1>>()
                    .into_input(&mut plant_logger)?;
            }
            i if i == 2 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += M1Segment::<2, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
                let mut asm_setpoint: Initiator<_> = (
                    asms_coefs(i),
                    format!(
                        "ASM #{i}
      Set-Point"
                    ),
                )
                    .into();
                m2 += AsmsSegment::<2>::builder(
                    n_actuator,
                    asms_calibration.stiffness(i),
                    &mut asm_setpoint,
                )
                .build(&mut plant)?;
                setpoints += asm_setpoint;
                plant
                    .add_output()
                    .bootstrap()
                    .build::<VoiceCoilsMotion<2>>()
                    .into_input(&mut plant_logger)?;
            }
            i if i == 3 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += M1Segment::<3, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
                let mut asm_setpoint: Initiator<_> = (
                    asms_coefs(i),
                    format!(
                        "ASM #{i}
      Set-Point"
                    ),
                )
                    .into();
                m2 += AsmsSegment::<3>::builder(
                    n_actuator,
                    asms_calibration.stiffness(i),
                    &mut asm_setpoint,
                )
                .build(&mut plant)?;
                setpoints += asm_setpoint;
                plant
                    .add_output()
                    .bootstrap()
                    .build::<VoiceCoilsMotion<3>>()
                    .into_input(&mut plant_logger)?;
            }
            i if i == 4 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += M1Segment::<4, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
                let mut asm_setpoint: Initiator<_> = (
                    asms_coefs(i),
                    format!(
                        "ASM #{i}
      Set-Point"
                    ),
                )
                    .into();
                m2 += AsmsSegment::<4>::builder(
                    n_actuator,
                    asms_calibration.stiffness(i),
                    &mut asm_setpoint,
                )
                .build(&mut plant)?;
                setpoints += asm_setpoint;
                plant
                    .add_output()
                    .bootstrap()
                    .build::<VoiceCoilsMotion<4>>()
                    .into_input(&mut plant_logger)?;
            }
            i if i == 5 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += M1Segment::<5, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
                let mut asm_setpoint: Initiator<_> = (
                    asms_coefs(i),
                    format!(
                        "ASM #{i}
      Set-Point"
                    ),
                )
                    .into();
                m2 += AsmsSegment::<5>::builder(
                    n_actuator,
                    asms_calibration.stiffness(i),
                    &mut asm_setpoint,
                )
                .build(&mut plant)?;
                setpoints += asm_setpoint;
                plant
                    .add_output()
                    .bootstrap()
                    .build::<VoiceCoilsMotion<5>>()
                    .into_input(&mut plant_logger)?;
            }
            i if i == 6 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += M1Segment::<6, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
                let mut asm_setpoint: Initiator<_> = (
                    asms_coefs(i),
                    format!(
                        "ASM #{i}
      Set-Point"
                    ),
                )
                    .into();
                m2 += AsmsSegment::<6>::builder(
                    n_actuator,
                    asms_calibration.stiffness(i),
                    &mut asm_setpoint,
                )
                .build(&mut plant)?;
                setpoints += asm_setpoint;
                plant
                    .add_output()
                    .bootstrap()
                    .build::<VoiceCoilsMotion<6>>()
                    .into_input(&mut plant_logger)?;
            }
            i if i == 7 => {
                let mut rbm_setpoint: Initiator<_> = rbm_signal(i).into();
                let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> =
                    Signals::new(if i == 7 { 306 } else { 335 }, n_step).into();
                m1 += M1Segment::<7, ACTUATOR_RATE>::builder(
                    m1_calibration.clone(),
                    &mut rbm_setpoint,
                    &mut actuators_setpoint,
                )
                .build(&mut plant)?;
                setpoints += rbm_setpoint + actuators_setpoint;
                let mut asm_setpoint: Initiator<_> = (
                    asms_coefs(i),
                    format!(
                        "ASM #{i}
      Set-Point"
                    ),
                )
                    .into();
                m2 += AsmsSegment::<7>::builder(
                    n_actuator,
                    asms_calibration.stiffness(i),
                    &mut asm_setpoint,
                )
                .build(&mut plant)?;
                setpoints += asm_setpoint;
                plant
                    .add_output()
                    .bootstrap()
                    .build::<VoiceCoilsMotion<7>>()
                    .into_input(&mut plant_logger)?;
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

    plant
        .add_output()
        .bootstrap()
        .build::<M1RigidBodyMotions>()
        .into_input(&mut plant_logger)?;

    (model!(plant, plant_logger) + mount + m1 + m2 + setpoints)
        .name("mount-m1-m2")
        .flowchart()
        .check()?
        .run()
        .await?;
    println!("{}", plant_logging.lock().await);

    (*plant_logging.lock().await).to_mat_file("mount-m1-m2.mat")?;

    let n = sids.len();
    let n_total_mode = n_actuator * n;

    let rbm_err = (*plant_logging.lock().await).chunks().last().unwrap()[n_total_mode..]
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

    let m = asms_calibration.modes_t(Some(sids.clone())).unwrap();

    (*plant_logging.lock().await)
        .chunks()
        .enumerate()
        .skip(n_step - 11)
        .map(|(i, u)| {
            let x: Vec<_> = u[..n_total_mode]
                .chunks(n_actuator)
                .zip(&m)
                .flat_map(|(u, m)| {
                    let c = m * DVector::<f64>::from_column_slice(u);
                    c.as_slice().to_vec()
                })
                .collect();
            (
                i,
                x.iter()
                    .step_by(n_mode)
                    .take(n)
                    .map(|x| x * 1e6)
                    .collect::<Vec<f64>>(),
                x.iter()
                    .skip(n_mode - 1)
                    .step_by(n_mode)
                    .take(n)
                    .map(|x| x * 1e6)
                    .collect::<Vec<f64>>(),
            )
        })
        .for_each(|(i, x, y)| println!("{:4}: {:+.6?}---{:+.6?}", i, x, y));

    let aas_err = (*plant_logging.lock().await)
        .chunks()
        .last()
        .unwrap()
        .chunks(n_actuator)
        .zip(&m)
        .zip(aas.chunks(n_mode))
        .map(|((u, m), x0)| {
            let x = m * DVector::<f64>::from_column_slice(u);
            x.as_slice()
                .iter()
                .zip(x0)
                .map(|(x, x0)| (x - x0) * 1e6)
                .map(|x| x * x)
                .sum::<f64>()
                / n_mode as f64
        })
        .map(|x| x.sqrt())
        .sum::<f64>()
        / 7f64;

    assert!(dbg!(rbm_err) < 5e-2 && dbg!(aas_err) < 1e-4);

    Ok(())
}
