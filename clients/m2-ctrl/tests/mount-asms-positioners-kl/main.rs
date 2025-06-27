use std::{env, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Signal, Signals};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::MCM2SmHexF,
        outputs::{MCM2Lcl6D, MCM2RB6D, MCM2SmHexD, OSSM1Lcl},
    },
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{
        M2PositionerForces, M2PositionerNodes, M2RigidBodyMotions,
        asm::{
            M2ASMAsmCommand, M2ASMFluidDampingForces, M2ASMReferenceBodyNodes,
            M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
        },
    },
    mount::{MountEncoders, MountSetPoint, MountTorques},
    optics::SegmentPiston,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_m2_ctrl::{ASMS, positioner::AsmsPositioners};
use gmt_dos_clients_mount::Mount;
use interface::UID;
use matio_rs::MatFile;
use nalgebra as na;

#[derive(UID)]
pub enum ASMSCmd {}

/*
export FEM_REPO=/home/rconan/mnt/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/
cargo test --release  --package gmt_dos-clients_m2-ctrl --features serde --test mount-asms-positioners-kl -- main --exact --nocapture
 */

#[tokio::test(flavor = "multi_thread")]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 8000;
    let sim_duration = 1_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = gmt_fem::FEM::from_env().unwrap();

    let positioners = AsmsPositioners::from_fem(&mut fem)?;
    let n_mode = 6;
    let asms = ASMS::<1>::from_fem(&mut fem, Some(vec![n_mode; 7]))?;

    let mat = MatFile::load(
        Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
            .join("tests")
            .join("mount-asms-positioners-kl")
            .join("m2_rbm_2_asm_kl.mat"),
    )?;
    let rbm_2_mode: na::DMatrix<f64> = mat.var("r2kl")?;
    dbg!(rbm_2_mode.shape());

    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set");
    let fem_path = Path::new(&fem_var);
    let path = fem_path.join("KLmodesGS36p90.mat");
    let mut kl_modes: Vec<na::DMatrix<f64>> = vec![];
    for i in 1..=7 {
        let mat: na::DMatrix<f64> = MatFile::load(&path)?.var(format!("KL_{i}"))?;
        let (nact, nkl) = mat.shape();
        assert!(n_mode <= nkl);
        kl_modes.push(mat.remove_columns(n_mode, nkl - n_mode));
    }

    let kl_modes_t: Vec<_> = kl_modes.iter().map(|mat| mat.transpose()).collect();
    let sids = vec![1, 2, 3, 4, 5, 6, 7];
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(8e3)
        .proportional_damping(2. / 100.)
        .including_mount()
        .including_asms(
            Some(sids.clone()),
            Some(kl_modes.iter().map(|x| x.as_view()).collect()),
            Some(kl_modes_t.iter().map(|x| x.as_view()).collect()),
        )?
        .ins::<MCM2SmHexF>()
        .outs::<MCM2SmHexD>()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .use_static_gain_compensation()
        .outs::<MCM2RB6D>()
        .build()?;

    // MOUNT CONTROL
    let mount_setpoint = Signals::new(3, n_step);
    let mount = Mount::new();

    /*     let rbm_fun =
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
    }); */

    let rbm = Signals::new(6 * 7, n_step); //.channel(36 + 2, Signal::Constant(1e-6));

    // let asm_cmd: Signals<_> = Signals::new(n_mode * 7, n_step).channel(1, Signal::Constant(-7e-6));
    let asm_cmd: Signals<_> = rbm_2_mode
        .column(2)
        .iter()
        .take(6)
        .enumerate()
        .fold(Signals::new(n_mode * 7, n_step), |signals, (i, c)| {
            signals.channel(i, Signal::Constant(c * 1e-6))
        });

    let lom = LinearOpticalModel::new()?;

    actorscript! {
        1: mount_setpoint[MountSetPoint] -> mount[MountTorques] -> plant[MountEncoders]! -> mount
        1: rbm[M2RigidBodyMotions] -> positioners[M2PositionerForces] -> plant[M2PositionerNodes]! -> positioners

        1: asm_cmd[M2ASMAsmCommand] -> {asms}[M2ASMVoiceCoilsForces]-> plant
        1: {asms}[M2ASMFluidDampingForces] -> plant[M2ASMVoiceCoilsMotion]! -> {asms}

        1: lom
        1: plant[M1RigidBodyMotions] -> lom
        1: plant[M2RigidBodyMotions] -> lom
    }

    println!("RBM (REF-BODY)");
    let mut p = plant.lock().await;
    <DiscreteModalSolver<ExponentialMatrix> as interface::Write<M2ASMReferenceBodyNodes>>::write(
        &mut p,
    )
    .map(|rb| {
        rb.chunks(6)
            .map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("{:2}: {:+.1?}", i, x));
    });

    println!("RBM (SHELL)");
    <DiscreteModalSolver<ExponentialMatrix> as interface::Write<M2RigidBodyMotions>>::write(&mut p)
        .map(|rb| {
            rb.chunks(6)
                .map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
                .enumerate()
                .for_each(|(i, x)| println!("{:2}: {:+.1?}", i, x));
        });

    println!("KL COEFS");
    <DiscreteModalSolver<ExponentialMatrix> as interface::Write<M2ASMVoiceCoilsMotion>>::write(
        &mut p,
    )
    .map(|rb| {
        rb.iter()
            .map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("{:2}: {:+.1?}", i, x));
    });

    println!("PISTON");
    <LinearOpticalModel as interface::Write<SegmentPiston<-6>>>::write(&mut *lom.lock().await)
        .map(|p| println!("{:+.3?}", p.as_arc()));

    Ok(())
}
