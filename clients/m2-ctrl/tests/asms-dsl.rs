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
    let mut cmd = vec![];
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
        let (nact, n_mode) = mat.shape();
        let mut kl_coefs = vec![0.; n_mode];
        kl_coefs[6] = 1e-8;
        cmd.push(
            { &mat * na::DVector::from_column_slice(&kl_coefs) }
                .as_slice()
                .to_vec(),
        );
        asms_kl_coefs.push(kl_coefs);
        asms_nact.push(nact);
        kl_modes.push(mat);
    }
    fem.switch_inputs(Switch::On, None)
        .switch_outputs(Switch::On, None);
    println!("stiffness from FEM in {}ms", now.elapsed().as_millis());

    let now = Instant::now();
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(8e3)
        .proportional_damping(2. / 100.)
        .including_asms(Some(vec![1, 2, 3, 4, 5, 6, 7]), None, None)?
        .use_static_gain_compensation()
        .build()?;
    println!("plant build up in {}ms", now.elapsed().as_millis());

    let (asm1, asm2, asm3, asm4, asm5, asm6, asm7) = (
        AsmSegmentInnerController::<1>::new(asms_nact[0], Some(vc_f2d[0].as_slice().to_vec())),
        AsmSegmentInnerController::<2>::new(asms_nact[1], Some(vc_f2d[1].as_slice().to_vec())),
        AsmSegmentInnerController::<3>::new(asms_nact[2], Some(vc_f2d[2].as_slice().to_vec())),
        AsmSegmentInnerController::<4>::new(asms_nact[3], Some(vc_f2d[3].as_slice().to_vec())),
        AsmSegmentInnerController::<5>::new(asms_nact[4], Some(vc_f2d[4].as_slice().to_vec())),
        AsmSegmentInnerController::<6>::new(asms_nact[5], Some(vc_f2d[5].as_slice().to_vec())),
        AsmSegmentInnerController::<7>::new(asms_nact[6], Some(vc_f2d[6].as_slice().to_vec())),
    );

    let (s1, s2, s3, s4, s5, s6, s7) = (
        Signals::from((cmd[0].as_slice(), 800)),
        Signals::from((cmd[1].as_slice(), 800)),
        Signals::from((cmd[2].as_slice(), 800)),
        Signals::from((cmd[3].as_slice(), 800)),
        Signals::from((cmd[4].as_slice(), 800)),
        Signals::from((cmd[5].as_slice(), 800)),
        Signals::from((cmd[6].as_slice(), 800)),
    );

    actorscript!(
        1: s1[AsmCommand<1>] -> asm1[VoiceCoilsForces<1>] -> &plant
        1: asm1[FluidDampingForces<1>] -> &plant[VoiceCoilsMotion<1>]! -> asm1

        1: s2[AsmCommand<2>] -> asm2[VoiceCoilsForces<2>] -> &plant
        1: asm2[FluidDampingForces<2>] -> &plant[VoiceCoilsMotion<2>]! -> asm2

        1: s3[AsmCommand<3>] -> asm3[VoiceCoilsForces<3>] -> &plant
        1: asm3[FluidDampingForces<3>] -> &plant[VoiceCoilsMotion<3>]! -> asm3

        1: s4[AsmCommand<4>] -> asm4[VoiceCoilsForces<4>] -> &plant
        1: asm4[FluidDampingForces<4>] -> &plant[VoiceCoilsMotion<4>]! -> asm4

        1: s5[AsmCommand<5>] -> asm5[VoiceCoilsForces<5>] -> &plant
        1: asm5[FluidDampingForces<5>] -> &plant[VoiceCoilsMotion<5>]! -> asm5

        1: s6[AsmCommand<6>] -> asm6[VoiceCoilsForces<6>] -> &plant
        1: asm6[FluidDampingForces<6>] -> &plant[VoiceCoilsMotion<6>]! -> asm6

        1: s7[AsmCommand<7>] -> asm7[VoiceCoilsForces<7>] -> &plant
        1: asm7[FluidDampingForces<7>] -> &plant[VoiceCoilsMotion<7>]! -> asm7

    );

    let mut p = plant.lock().await;
    let data =
        vec![
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<1>>>::write(
            &mut p,
        )
        .unwrap().into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<2>>>::write(
            &mut p,
        )
        .unwrap().into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<3>>>::write(
            &mut p,
        )
        .unwrap().into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<4>>>::write(
            &mut p,
        )
        .unwrap().into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<5>>>::write(
            &mut p,
        )
        .unwrap().into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<6>>>::write(
            &mut p,
        )
        .unwrap().into_arc(),
        <DiscreteModalSolver<ExponentialMatrix> as interface::Write<VoiceCoilsMotion<7>>>::write(
            &mut p,
        )
        .unwrap().into_arc()
    ];

    for i in 0..7 {
        let err = (asms_kl_coefs[i]
            .iter()
            .zip({ kl_modes[i].transpose() * na::DVector::from_column_slice(&data[0]) }.as_slice())
            .filter(|(c, _)| c.abs() > 0.)
            .map(|(&c, &p)| (1. - p / c).powi(2))
            .sum::<f64>()
            / 500 as f64)
            .sqrt();
        dbg!(err);
    }

    Ok(())
}
