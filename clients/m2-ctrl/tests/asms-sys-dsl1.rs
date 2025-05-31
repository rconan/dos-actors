use gmt_dos_actors::actorscript;
use gmt_dos_actors::system::Sys;
use gmt_dos_clients::Signals;
use gmt_dos_clients_io::gmt_m2::asm::{
    M2ASMAsmCommand, M2ASMFluidDampingForces, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
    segment::VoiceCoilsMotion,
};
use gmt_dos_clients_m2_ctrl::assembly::ASMS;

use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix, Model, Switch};
use interface::{Data, Read, UID, Update, Write};
use matio_rs::MatFile;
use nalgebra as na;
use std::sync::Arc;
use std::{path::Path, time::Instant};

/*
export FEM_REPO=...
cargo test --release  --package gmt_dos-clients_m2-ctrl --features serde --test asms-sys-dsl1 -- --exact --nocapture
*/
#[tokio::test]
async fn main() -> anyhow::Result<()> {
    let now = Instant::now();
    let mut fem = gmt_fem::FEM::from_env().unwrap();
    println!("FEM loaded in {}ms", now.elapsed().as_millis());

    let path = Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
        .join("examples")
        .join("asm-nodes")
        .join("KLmodesGS36p.mat");
    let now = Instant::now();
    let mut vc_f2d = vec![];
    let mut kl_modes: Vec<na::DMatrix<f64>> = vec![];
    let mut asms_kl_coefs = vec![];
    let mut asms_nact = vec![];
    // let mut cmd = vec![];
    let n_mode = 6;
    for i in 1..=7 {
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);

        vc_f2d.push(
            fem.switch_inputs_by_name(vec![format!("MC_M2_S{i}_VC_delta_F")], Switch::On)
                .and_then(|fem| {
                    fem.switch_outputs_by_name(vec![format!("MC_M2_S{i}_VC_delta_D")], Switch::On)
                })
                .map(|fem| {
                    fem.reduced_static_gain()
                        .unwrap_or_else(|| fem.static_gain())
                })?,
        );
        // println!("{:?}", vc_f2d.shape());
        let mat: na::DMatrix<f64> = MatFile::load(&path)?.var(format!("KL_{i}"))?;
        let (nact, nkl) = mat.shape();
        assert!(n_mode <= nkl);
        let mut kl_coefs = vec![0.; n_mode];
        kl_coefs[1] = 1e-8;
        // cmd.push(
        //     { &mat * na::DVector::from_column_slice(&kl_coefs) }
        //         .as_slice()
        //         .to_vec(),
        // );
        asms_kl_coefs.push(kl_coefs);
        asms_nact.push(n_mode);
        kl_modes.push(mat.remove_columns(n_mode, nkl - n_mode));
    }
    fem.switch_inputs(Switch::On, None)
        .switch_outputs(Switch::On, None);
    println!("stiffness from FEM in {}ms", now.elapsed().as_millis());

    let kl_modes_t = kl_modes.iter().map(|x| x.transpose()).collect::<Vec<_>>();

    let now = Instant::now();
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(8e3)
        .proportional_damping(2. / 100.)
        .including_asms(
            Some(vec![1, 2, 3, 4, 5, 6, 7]),
            Some(kl_modes.iter().map(|x| x.as_view()).collect()),
            Some(kl_modes_t.iter().map(|x| x.as_view()).collect()),
        )?
        .use_static_gain_compensation()
        .build()?;
    println!("plant build up in {}ms", now.elapsed().as_millis());
    println!("{plant}");

    let ks: Vec<_> = vc_f2d.iter().map(|x| Some(x.as_slice().to_vec())).collect();
    let mut asms = Sys::new(ASMS::new(asms_nact, ks)).build()?;

    let cmd: Vec<_> = asms_kl_coefs.clone().into_iter().flatten().collect();
    let signal = Signals::from((cmd.as_slice(), 8000)); //Signals::new(675 * 7, 800);

    actorscript!(
        1: signal[M2ASMAsmCommand] -> {asms}[M2ASMVoiceCoilsForces] -> plant
        1: {asms}[M2ASMFluidDampingForces] -> plant[M2ASMVoiceCoilsMotion]! -> {asms}
    );

    let mut p = plant.lock().await;
    let data = vec![
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<1>>>::write(
            &mut p,
        )
        .unwrap()
        .into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<2>>>::write(
            &mut p,
        )
        .unwrap()
        .into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<3>>>::write(
            &mut p,
        )
        .unwrap()
        .into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<4>>>::write(
            &mut p,
        )
        .unwrap()
        .into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<5>>>::write(
            &mut p,
        )
        .unwrap()
        .into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<6>>>::write(
            &mut p,
        )
        .unwrap()
        .into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<7>>>::write(
            &mut p,
        )
        .unwrap()
        .into_arc(),
    ];

    for i in 0..7 {
        /*         let err = (asms_kl_coefs[i]
        .iter()
        .zip({ kl_modes[i].transpose() * na::DVector::from_column_slice(&data[0]) }.as_slice())
        .filter(|(c, _)| c.abs() > 0.)
        .map(|(&c, &p)| (1. - p / c).powi(2))
        .sum::<f64>()
        / 500 as f64)
        .sqrt(); */
        let err = (asms_kl_coefs[i]
            .iter()
            .zip(data[i].iter())
            .filter(|(c, _)| c.abs() > 0.)
            .map(|(&c, &p)| (1. - p / c).powi(2))
            .sum::<f64>()
            / 500 as f64)
            .sqrt();
        dbg!(err);
    }

    Ok(())
}
