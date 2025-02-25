use std::{env, path::Path};

use demos::*;
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    signals::{OneSignal, Signal, Signals},
    smooth::{Smooth, Weight},
};
use gmt_dos_clients_fem::{solvers::ExponentialMatrix, DiscreteModalSolver};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_fem::{
        inputs::{MCM2Lcl6F, MCM2SmHexF, OSSM1Lcl6F, CFD2021106F},
        outputs::{MCM2Lcl6D, MCM2SmHexD, OSSM1Lcl, MCM2RB6D},
    },
    gmt_m1::{assembly, M1RigidBodyMotions},
    gmt_m2::{
        asm::{
            M2ASMAsmCommand, M2ASMFluidDampingForces, M2ASMReferenceBodyNodes,
            M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
        },
        M2PositionerForces, M2PositionerNodes, M2RigidBodyMotions,
    },
    mount::{MountEncoders, MountSetPoint, MountTorques},
    optics::WfeRms,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_m1_ctrl::Calibration;
use gmt_dos_clients_m2_ctrl::AsmsPositioners;
use gmt_dos_clients_mount::Mount;
use gmt_dos_clients_windloads::CfdLoads;
use gmt_dos_systems_m1::M1;
use gmt_dos_systems_m2::ASMS;
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80;

// #[derive(UID)]
// #[alias(name = WfeRms<-6>, client = LinearOpticalModel, traits = Write)]
// enum M2WfeRms {}

/*
MOUNT_MODEL=MOUNT_PDR_8kHz FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release --bin windloaded-mount-m1-asms
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bin")
            .join("windloaded-mount-m1-asms"),
    );

    let sim_sampling_frequency = 8000;
    let sim_duration = 5_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = Option::<FEM>::None;
    // println!("{fem}");

    let m1_calibration = Calibration::new(fem.get_or_insert(FEM::from_env()?));

    let positioners = AsmsPositioners::new(fem.get_or_insert(FEM::from_env()?))?;
    let asms = ASMS::<1>::new(fem.get_or_insert(FEM::from_env()?))?.build()?;

    let cfd_loads = CfdLoads::foh(".", sim_sampling_frequency)
        .duration(sim_duration as f64)
        .mount(fem.get_or_insert(FEM::from_env()?), 0, None)
        .m1_segments()
        .m2_segments()
        .build()?;

    // FEM MODEL
    let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
    let state_space =
        DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem.unwrap_or(FEM::from_env()?))
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            //.max_eigen_frequency(75f64)
            .including_mount()
            .including_m1(Some(sids.clone()))?
            .including_asms(Some(sids.clone()), None, None)?
            .ins::<CFD2021106F>()
            .ins::<OSSM1Lcl6F>()
            .ins::<MCM2Lcl6F>()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .ins::<MCM2SmHexF>()
            .outs::<MCM2SmHexD>()
            .outs::<MCM2RB6D>()
            .use_static_gain_compensation()
            .build()?
            .with_cuda_solver();
    println!("{state_space}");

    // SET POINT
    let setpoint = Signals::new(3, n_step); //.channel(1, Signal::Constant(1f64.from_arcsec()));

    // FEM
    let fem = state_space;
    // MOUNT CONTROL
    let mount = Mount::new();
    // LOM
    let lom = LinearOpticalModel::new()?;
    let m1_lom = LinearOpticalModel::new()?;
    let asm_shell_lom = LinearOpticalModel::new()?;
    let asm_rb_lom = LinearOpticalModel::new()?;

    let sigmoid = OneSignal::try_from(Signals::new(1, n_step).channel(
        0,
        Signal::Sigmoid {
            amplitude: 1f64,
            sampling_frequency_hz: sim_sampling_frequency as f64,
        },
    ))?;

    let m1_smoother = Smooth::new();
    let m2_smoother = Smooth::new();
    let mount_smoother = Smooth::new();

    let actuators = Signals::new(6 * 335 + 306, n_step);
    let m1_rbm = Signals::new(6 * 7, n_step);
    let m1 = M1::<ACTUATOR_RATE>::new(&m1_calibration)?;

    let m2_rbm = Signals::new(6 * 7, n_step);

    let asm_cmd = Signals::new(675 * 7, n_step);

    actorscript! {
    #[labels(fem = "GMT FEM", mount = "Mount\nControl", lom="Linear Optical\nModel")]
    1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount

    1: cfd_loads[CFDM1WindLoads] -> m1_smoother
    1: sigmoid[Weight] -> m1_smoother[CFDM1WindLoads] -> fem

    1: cfd_loads[CFDM2WindLoads] -> m2_smoother
    1: sigmoid[Weight] -> m2_smoother[CFDM2WindLoads] -> fem

    1: cfd_loads[CFDMountWindLoads] -> mount_smoother
    1: sigmoid[Weight] -> mount_smoother[CFDMountWindLoads] -> fem

    1: m1_rbm[assembly::M1RigidBodyMotions]
        -> {m1}[assembly::M1HardpointsForces]
            -> fem[assembly::M1HardpointsMotion]! -> {m1}
    1: actuators[assembly::M1ActuatorCommandForces]
            -> {m1}[assembly::M1ActuatorAppliedForces] -> fem

    1: m2_rbm[M2RigidBodyMotions]
        -> positioners[M2PositionerForces]
            -> fem[M2PositionerNodes]! -> positioners

    1: asm_cmd[M2ASMAsmCommand] -> {asms}[M2ASMVoiceCoilsForces]-> fem
    1: {asms}[M2ASMFluidDampingForces] -> fem[M2ASMVoiceCoilsMotion]! -> {asms}

    8: lom[WfeRms<-6>]~
    1: fem[M1RigidBodyMotions] -> lom
    1: fem[M2RigidBodyMotions] -> lom

    8: m1_lom[M1RbmWfeRms]~
    1: fem[M1RigidBodyMotions] -> m1_lom

    8: asm_shell_lom[AsmShellWfeRms]~
    1: fem[M2RigidBodyMotions] -> asm_shell_lom

    8: asm_rb_lom[AsmRefBodyWfeRms]~
    1: fem[M2ASMReferenceBodyNodes] -> asm_rb_lom
    }

    Ok(())
}
