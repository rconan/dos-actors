use std::{env, path::Path};

use demos::*;
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::{assembly, M1RigidBodyMotions},
    gmt_m2::{
        asm::{M2ASMAsmCommand, M2ASMReferenceBodyNodes},
        M2RigidBodyMotions,
    },
    mount::MountSetPoint,
    optics::WfeRms,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_servos::{
    asms_servo::ReferenceBody, AsmsServo, GmtFem, GmtServoMechanisms, WindLoads,
};
use gmt_dos_clients_windloads::{
    system::{Mount, SigmoidCfdLoads, M1, M2},
    CfdLoads,
};
use gmt_fem::FEM;
use interface::filing::Filing;

const ACTUATOR_RATE: usize = 80;

/*
MOUNT_MODEL=MOUNT_PDR_8kHz FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release --bin windloaded-servos
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bin")
            .join("windloaded-servos"),
    );

    let sim_sampling_frequency = 8000;
    let sim_duration = 5_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let (cfd_loads, gmt_servos) = {
        let mut fem = Option::<FEM>::None;
        // The CFD wind loads must be called next afer the FEM as it is modifying
        // the FEM CFDMountWindLoads inputs
        let cfd_loads = Sys::<SigmoidCfdLoads>::from_data_repo_or_else("windloads.bin", || {
            CfdLoads::foh(".", sim_sampling_frequency)
                .duration(sim_duration as f64)
                .windloads(
                    fem.get_or_insert(FEM::from_env().unwrap()),
                    Default::default(),
                )
        })?;

        let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else(
            "servos.bin",
            || {
                GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                    sim_sampling_frequency as f64,
                    fem.unwrap(),
                )
                .wind_loads(WindLoads::new())
                .asms_servo(AsmsServo::new().reference_body(ReferenceBody::new()))
            },
        )?;

        (cfd_loads, gmt_servos)
    };

    // SET POINT
    // let setpoint = Signals::new(3, n_step); //.channel(1, Signal::Constant(1f64.from_arcsec()));

    // LOM
    let lom = LinearOpticalModel::new()?;
    let m1_lom = LinearOpticalModel::new()?;
    let asm_shell_lom = LinearOpticalModel::new()?;
    let asm_rb_lom = LinearOpticalModel::new()?;

    // let sigmoid = OneSignal::try_from(Signals::new(1, n_step).channel(
    //     0,
    //     Signal::Sigmoid {
    //         amplitude: 1f64,
    //         sampling_frequency_hz: sim_sampling_frequency as f64,
    //     },
    // ))?;

    // let m1_smoother = Smooth::new();
    // let m2_smoother = Smooth::new();
    // let mount_smoother = Smooth::new();

    // let actuators = Signals::new(6 * 335 + 306, n_step);
    // let m1_rbm = Signals::new(6 * 7, n_step);

    // let m2_rbm: Signals<_> = Signals::new(6 * 7, n_step);
    // let asm_cmd: Signals<_> = Signals::new(675 * 7, n_step);

    actorscript! {
    // 1: setpoint[MountSetPoint] -> {gmt_servos::GmtMount}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

    // 1: m1_rbm[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
    // 1: actuators[assembly::M1ActuatorCommandForces] -> {gmt_servos::GmtM1}

    // 1: m2_rbm[M2RigidBodyMotions]-> {gmt_servos::GmtM2Hex}
    // 1: asm_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}

    8: lom[WfeRms<-6>]~
    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> lom
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> lom

    8: m1_lom[M1RbmWfeRms]~
    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> m1_lom

    8: asm_shell_lom[AsmShellWfeRms]~
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> asm_shell_lom

    8: asm_rb_lom[AsmRefBodyWfeRms]~
    1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes] -> asm_rb_lom
    }

    Ok(())
}
