use std::{env, path::Path};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
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
use interface::{filing::Filing, Tick};

const ACTUATOR_RATE: usize = 80;

const PRELOADING_N_SAMPLE: usize = 2000 * 8;
const N_SAMPLE: usize = 1000 * 8;

/*
MOUNT_MODEL=MOUNT_PDR_8kHz FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release --bin windloaded-servos-preloading --features "gmt_dos-clients_arrow"
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bin")
            .join("windloaded-servos-preloading"),
    );

    let sim_sampling_frequency = 8000;
    let sim_duration = 5_usize; // second

    let (cfd_loads, gmt_servos) = {
        let mut fem = Option::<FEM>::None;
        // The CFD wind loads must be called next afer the FEM as it is modifying the FEM CFDMountWindLoads inputs
        let cfd_loads = Sys::<SigmoidCfdLoads>::from_data_repo_or("windloads.bin", {
            CfdLoads::foh(".", sim_sampling_frequency)
                .duration(sim_duration as f64)
                .windloads(fem.get_or_insert(FEM::from_env()?), Default::default())
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

    // LOM
    let lom = LinearOpticalModel::new()?;

    let metronome: Timer = Timer::new(PRELOADING_N_SAMPLE + N_SAMPLE);

    actorscript! {
    1: metronome[Tick] -> {gmt_servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

    8: lom[WfeRms]${1}
    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> lom
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> lom

    }

    let ref_wfe_rms: Vec<f64> = model_logging_8
        .lock()
        .await
        .iter("WfeRms")?
        .flatten()
        .collect();

    /*
    PRELOADING
     */

    let (cfd_loads, gmt_servos) = {
        let mut fem = Option::<FEM>::None;
        // The CFD wind loads must be called next afer the FEM as it is modifying
        // the FEM CFDMountWindLoads inputs
        let cfd_loads = Sys::<SigmoidCfdLoads>::from_data_repo_or("windloads.bin", {
            CfdLoads::foh(".", sim_sampling_frequency)
                .duration(sim_duration as f64)
                .windloads(fem.get_or_insert(FEM::from_env()?), Default::default())
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

    let metronome: Timer = Timer::new(PRELOADING_N_SAMPLE);

    actorscript! {
    1: metronome[Tick] -> {gmt_servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}
    }
    gmt_servos.to_data_repo("preloaded_servos.bin")?;
    cfd_loads.to_data_repo("preloaded_windloads.bin")?;

    /*
    PRELOADED MODELS
     */

    let cfd_loads = Sys::<SigmoidCfdLoads>::from_data_repo("preloaded_windloads.bin")?;
    let gmt_servos =
        Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo("preloaded_servos.bin")?;

    // LOM
    let lom = LinearOpticalModel::new()?;

    let metronome: Timer = Timer::new(N_SAMPLE);

    actorscript! {
    1: metronome[Tick] -> {gmt_servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

    8: lom[WfeRms]${1}
    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> lom
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> lom

    }

    let wfe_rms: Vec<f64> = model_logging_8
        .lock()
        .await
        .iter("WfeRms")?
        .flatten()
        .collect();
    let n = wfe_rms.len() as f64;

    let preload_err = (wfe_rms
        .into_iter()
        .rev()
        .zip(ref_wfe_rms.into_iter().rev())
        .map(|(w, w0)| w.powi(2) - w0.powi(2))
        .map(|x| x.powi(2))
        .sum::<f64>()
        / n)
        .sqrt();
    dbg!(preload_err);

    Ok(())
}
