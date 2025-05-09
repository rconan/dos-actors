use std::{env, path::Path};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    optics::{SegmentPiston, SegmentTipTilt, TipTilt, WfeRms},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_servos::{GmtFem, GmtServoMechanisms, WindLoads};
use gmt_dos_clients_windloads::{
    system::{Mount, SigmoidCfdLoads, M1, M2},
    CfdLoads,
};
use gmt_fem::FEM;
use interface::units::Arcsec;

const ACTUATOR_RATE: usize = 80;

/*
MOUNT_MODEL=MOUNT_FDR_1kHz FEM_REPO=path/to/20250506_1715_zen_30_M1_202110_FSM_202305_Mount_202305_pier_202411_M1_actDamping/ cargo r --r--bin windloaded-servos-fsm
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

    let sim_sampling_frequency = 1000;
    let sim_duration = 30_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    /* let (cfd_loads, gmt_servos) = {
        let mut fem = Option::<FEM>::None;
        // The CFD wind loads must be called next afer the FEM as it is modifying
        // the FEM CFDMountWindLoads inputs
        let cfd_loads = Sys::<SigmoidCfdLoads>::from_data_repo_or_else("windloads.bin", || {
            CfdLoads::foh(".", sim_sampling_frequency)
                .duration(sim_duration as f64)
                .windloads(
                    fem.get_or_insert_with(|| FEM::from_env().unwrap()),
                    0,
                    Default::default(),
                )
            // .mount(fem.get_or_insert_with(|| FEM::from_env().unwrap()), 0, None)
            // .m1_segments()
            // .m2_segments()
        })?;

        let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else(
            "servos.bin",
            || {
                GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                    sim_sampling_frequency as f64,
                    fem.unwrap(),
                )
                .wind_loads(WindLoads::new())
                // .asms_servo(AsmsServo::new().reference_body(ReferenceBody::new()))
            },
        )?;

        (cfd_loads, gmt_servos)
    }; */
    let mut fem = FEM::from_env()?;
    let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(
        CfdLoads::foh(
            // "/home/rconan/projects/dos-actors-broken/demos",
            ".",
            sim_sampling_frequency,
        )
        .duration(sim_duration as f64)
        .windloads(&mut fem, Default::default()),
    )?;

    let gmt_servos =
        GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
            .wind_loads(WindLoads::new())
            .build()?;

    // LOM
    let lom = LinearOpticalModel::new()?;

    actorscript! {
    1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

    1: {gmt_servos::GmtFem}[M1RigidBodyMotions] -> lom
    1: {gmt_servos::GmtFem}[M2RigidBodyMotions] -> lom
    1: lom[WfeRms<-6>]~
    1: lom[Arcsec<SegmentTipTilt>]~
    1: lom[Arcsec<TipTilt>]~
    1: lom[SegmentPiston<-6>]~
    }

    Ok(())
}
