use anyhow::{Ok, Result};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    integrator::Integrator,
    low_pass_filter::LowPassFilter,
    operator::{Left, Operator, Right},
    signals::{Signal, Signals},
    timer::Timer,
};
use gmt_dos_clients_io::{
    gmt_m1::{M1EdgeSensors, M1RigidBodyMotions, assembly},
    gmt_m2::M2RigidBodyMotions,
    optics::{SegmentPiston, SegmentTipTilt},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_scope::server::{Monitor, Scope};
use gmt_dos_clients_servos::{
    AsmsServo, EdgeSensors, GmtFem, GmtM1, GmtServoMechanisms, asms_servo,
};
use gmt_fem::FEM;
use interface::{Tick, UID, filing::Filing, units::NM};
use matio_rs::MatFile;
use nalgebra as na;
use std::{env, path::Path};

const ACTUATOR_RATE: usize = 80; // 100Hz

#[derive(UID)]
pub enum RBMCmd {}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let data_repo = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("src")
        .join("bin")
        .join("m1");
    env::set_var("DATA_REPO", &data_repo);
    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set");
    let fem_path = Path::new(&fem_var);

    let sim_sampling_frequency = 8000;
    let m1_freq = 100; // Hz
    assert!(m1_freq == sim_sampling_frequency / ACTUATOR_RATE);
    let sim_duration = 4_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    // EDGE SENSORS
    //  * M1 EDGE SENSORS NODES
    let es_nodes_2_data: na::DMatrix<f64> =
        MatFile::load(fem_path.join("M1_edge_sensor_conversion.mat"))?.var("A1")?;
    //  * EDGE SENSORS TO RIGID-BODY MOTIONS TRANSFORM (M1 & M2)
    let (es_2_m1_rbm, _es_2_m2_rbm) = {
        let mat = MatFile::load(fem_path.join("m12_e_rs").join("m12_r_es.mat"))?;
        let m1_es_recon: na::DMatrix<f64> = mat.var("m1_r_es")?;
        let m2_es_recon: na::DMatrix<f64> = mat.var("m2_r_es")?;
        (
            m1_es_recon.insert_rows(36, 6, 0f64) * es_nodes_2_data,
            m2_es_recon.insert_rows(36, 6, 0f64),
        )
    };
    dbg!(es_2_m1_rbm.shape());

    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else(
        "preloaded_servos.bin",
        || {
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                sim_sampling_frequency as f64,
                FEM::from_env().unwrap(),
            )
            .asms_servo(AsmsServo::new().reference_body(asms_servo::ReferenceBody::new()))
            .edge_sensors(EdgeSensors::both().m1_with(es_2_m1_rbm))
        },
    )?;

    let rbm = Signals::new(6 * 7, n_step).channel(
        6 * 6 + 2,
        Signal::Sigmoid {
            amplitude: 1e-6,
            sampling_frequency_hz: sim_sampling_frequency as f64,
        },
    );

    // EDGE SENSORS INTEGRAL CONTROLLERS:
    //  * M1
    let m1_es_int = Integrator::<M1EdgeSensors>::new(42).gain(1e-4);
    let m1_add = Operator::new("+");

    // LINEAR OPTICAL MODELS
    let lom = LinearOpticalModel::new()?;

    // SCOPES
    let mut monitor = Monitor::new(); //  * segment piston
    let sp_scope = Scope::<SegmentPiston<-9>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;

    let metronome: Timer = Timer::new(sim_sampling_frequency * sim_duration.min(3));

    /*     actorscript! {
        #[model(name=warmup)]
        1: metronome[Tick] -> {gmt_servos::GmtFem}
        1: rbm[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions]! -> lom
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions]! -> lom[SegmentPiston<-9>].. -> sp_scope
    }

    gmt_servos.to_data_repo("preloaded_servos.bin")?; */

    actorscript! {
        1: rbm[Right<RBMCmd>] -> m1_add[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions]! -> lom
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions]! -> lom[SegmentPiston<-9>].. -> sp_scope
        1: {gmt_servos::GmtFem}[M1EdgeSensors]! -> m1_es_int
        1: m1_es_int[Left<M1EdgeSensors>]! -> m1_add
    }

    monitor.await?;

    Ok(())
}
