/*!
# M2 ASMS Edge Sensors Feed-Forward and Shell Off-load

Model of the GMT servo-mechanisms with M2 edge sensors feed-forward to the the
voice coil actuators and off-load from the voice coil actuators to the reference bodies.

*/
use anyhow::Result;
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_io::cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads};
use gmt_dos_clients_servos::{
    AsmsServo, EdgeSensors, GmtFem, GmtServoMechanisms, WindLoads, asms_servo,
};
use gmt_dos_clients_windloads::{
    CfdLoads,
    system::{M1, M2, Mount, SigmoidCfdLoads},
};
use gmt_fem::FEM;
use interface::{Tick, filing::Filing};
use matio_rs::MatFile;
use nalgebra as na;
use std::{env, path::Path};

const ACTUATOR_RATE: usize = 80; // 100Hz
const PRELOADING_N_SAMPLE: usize = 8000 * 3;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let data_repo = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap()).join("data");
    env::set_var("DATA_REPO", &data_repo);

    let sim_sampling_frequency = 8000;

    // EDGE SENSORS
    //  * M1 EDGE SENSORS NODES
    let es_nodes_2_data: na::DMatrix<f64> =
        MatFile::load(data_repo.join("M1_edge_sensor_conversion.mat"))?.var("A1")?;
    //  * EDGE SENSORS TO RIGID-BODY MOTIONS TRANSFORM (M1 & M2)
    let es_2_m1_rbm = {
        let mat = MatFile::load(data_repo.join("m12_r_es.mat"))?;
        let m1_es_recon: na::DMatrix<f64> = mat.var("m1_r_es")?;
        m1_es_recon.insert_rows(36, 6, 0f64) * es_nodes_2_data
    };

    // GMT Servo-Mechanisms
    let (cfd_loads, gmt_servos) = {
        let mut fem = Option::<FEM>::None;
        // The CFD wind loads must be called next afer the FEM as it is modifying the FEM CFDMountWindLoads inputs
        let cfd_loads = Sys::<SigmoidCfdLoads>::from_data_repo_or("preloaded_windloads.bin", {
            CfdLoads::foh(data_repo.to_str().unwrap(), sim_sampling_frequency)
                .duration(10f64)
                .mount(fem.get_or_insert_with(|| FEM::from_env().unwrap()), 0, None)
                .m1_segments()
                .m2_segments()
        })?;

        let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else(
            "preloaded_servos.bin",
            || {
                GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                    sim_sampling_frequency as f64,
                    fem.unwrap(),
                )
                .wind_loads(WindLoads::new())
                .asms_servo(AsmsServo::new().reference_body(asms_servo::ReferenceBody::new()))
                .edge_sensors(EdgeSensors::both().m1_with(es_2_m1_rbm))
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

    Ok(())
}
