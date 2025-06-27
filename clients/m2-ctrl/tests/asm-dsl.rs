use gmt_dos_actors::actorscript;
use gmt_dos_clients::Signals;
use gmt_dos_clients_io::gmt_m2::asm::segment::{
    AsmCommand, FluidDampingForces, VoiceCoilsForces, VoiceCoilsMotion,
};
use gmt_dos_clients_m2_ctrl::AsmSegmentInnerController;

use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix, Model, Switch};
use matio_rs::MatFile;
use nalgebra as na;
use std::{path::Path, time::Instant};

/*
export FEM_REPO=...
cargo test --release  --package gmt_dos-clients_m2-ctrl --features serde --test asm-dsl -- --exact --nocapture
*/
#[tokio::test(flavor = "multi_thread")]
async fn main() -> anyhow::Result<()> {
    const SID: u8 = 1;

    let path = Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
        .join("examples")
        .join("asm-nodes")
        .join("KLmodesGS36p.mat");
    let kl_modes: na::DMatrix<f64> = MatFile::load(path)?.var(format!("KL_{SID}"))?;

    let now = Instant::now();
    let mut fem = gmt_fem::FEM::from_env().unwrap();
    println!("FEM loaded in {}ms", now.elapsed().as_millis());

    let now = Instant::now();
    fem.switch_inputs(Switch::Off, None)
        .switch_outputs(Switch::Off, None);

    let vc_f2d = fem
        .switch_inputs_by_name(vec![format!("MC_M2_S{SID}_VC_delta_F")], Switch::On)
        .and_then(|fem| {
            fem.switch_outputs_by_name(vec![format!("MC_M2_S{SID}_VC_delta_D")], Switch::On)
        })
        .map(|fem| {
            fem.reduced_static_gain()
                .unwrap_or_else(|| fem.static_gain())
        })?;
    println!("{:?}", vc_f2d.shape());

    fem.switch_inputs(Switch::On, None)
        .switch_outputs(Switch::On, None);
    println!("stiffness from FEM in {}ms", now.elapsed().as_millis());

    let now = Instant::now();
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(8e3)
        .proportional_damping(2. / 100.)
        .including_asms(Some(vec![SID]), None, None)?
        .use_static_gain_compensation()
        .build()?;
    println!("plant build up in {}ms", now.elapsed().as_millis());

    let (na, n_mode) = kl_modes.shape();
    let asm = AsmSegmentInnerController::<SID>::new(na, Some(vc_f2d.as_slice().to_vec()));

    let mut kl_coefs = vec![0.; n_mode];
    kl_coefs[6] = 1e-8;
    let cmd = { &kl_modes * na::DVector::from_column_slice(&kl_coefs) }
        .as_slice()
        .to_vec();
    let signal = Signals::from((cmd, 800));

    actorscript!(
        1: signal[AsmCommand<SID>] -> asm
        1: asm[VoiceCoilsForces<SID>] -> &plant
        1: asm[FluidDampingForces<SID>] -> &plant
        1: &plant[VoiceCoilsMotion<SID>]! -> asm
    );

    let mut p = plant.lock().await;
    let data = <DiscreteModalSolver<ExponentialMatrix> as interface::Write<
        VoiceCoilsMotion<SID>,
    >>::write(&mut p)
    .unwrap();

    let err = (kl_coefs
        .iter()
        .zip({ kl_modes.transpose() * na::DVector::from_column_slice(&data) }.as_slice())
        .filter(|(c, _)| c.abs() > 0.)
        .map(|(&c, &p)| (1. - p / c).powi(2))
        .sum::<f64>()
        / n_mode as f64)
        .sqrt();
    dbg!(err);

    Ok(())
}
