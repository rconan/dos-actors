/*!
# M2 ASMS Edge Sensors Feed-Forward and Shell Off-load

Model of the GMT servo-mechanisms with M2 edge sensors feed-forward to the the
voice coil actuators and off-load from the voice coil actuators to the reference bodies.

*/
use anyhow::Result;
use edge_sensors::{
    segment_piston::{M1Lom, M2Lom, M2RBLom, M2SegmentActuatorAverage, Scopes},
    AsmsToHexOffload, EdgeSensorsFeedForward, HexToRbm, M1EdgeSensorsAsRbms, M1EdgeSensorsToRbm,
    M2EdgeSensorsToRbm, RbmToShell, N_ACTUATOR,
};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    low_pass_filter::LowPassFilter,
    operator::{Left, Operator, Right},
    Integrator, Signal, Signals, Timer,
};
use gmt_dos_clients_io::gmt_m1::assembly;
use gmt_dos_clients_io::{
    gmt_fem::outputs::MCM2SmHexD,
    gmt_m1::{M1EdgeSensors, M1RigidBodyMotions},
    gmt_m2::{
        asm::{M2ASMAsmCommand, M2ASMReferenceBodyNodes, M2ASMVoiceCoilsMotion},
        M2EdgeSensors, M2RigidBodyMotions,
    },
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_scope::server::Monitor;
use gmt_dos_clients_servos::{
    asms_servo, AsmsServo, EdgeSensors, GmtFem, GmtM1, GmtM2, GmtM2Hex, GmtServoMechanisms,
    WindLoads,
};
use gmt_fem::FEM;
use interface::{filing::Filing, Tick};
use io::RBMCmd;
use matio_rs::MatFile;
use nalgebra as na;
use std::{env, path::Path};

const ACTUATOR_RATE: usize = 80; // 100Hz
const ASM_LPF_GAIN: f64 = 0.05;
const ASM_OFFLOAD_GAIN: f64 = 1. / 2000f64;
const M1_RBM_LPF_GAIN: f64 = 0.001;
#[tokio::main]
async fn main() -> Result<()> {
    env_logger::builder().init(); //.format_timestamp(None).init();

    let data_repo = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap()).join("data");
    env::set_var("DATA_REPO", &data_repo);
    if env::var("EDGE_SENSORS_DATA").is_err() {
        env::set_var("EDGE_SENSORS_DATA", &data_repo);
    };

    let sim_sampling_frequency = 8000;
    let m1_freq = 100; // Hz
    assert!(m1_freq == sim_sampling_frequency / ACTUATOR_RATE);
    let sim_duration = 3_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = Option::<FEM>::None;

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
    dbg!(es_2_m1_rbm.shape());

    // GMT Servo-Mechanisms
    let gmt_servos =
        Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else("servos.bin", || {
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(
                sim_sampling_frequency as f64,
                fem.unwrap_or_else(|| FEM::from_env().unwrap()),
            )
            .wind_loads(WindLoads::new())
            .asms_servo(AsmsServo::new().reference_body(asms_servo::ReferenceBody::new()))
            .edge_sensors(EdgeSensors::both().m1_with(es_2_m1_rbm))
        })?;

    let asms_cmd = Signals::from((vec![0f64; N_ACTUATOR * 7], n_step));

    // SCOPES
    let mut monitor = Monitor::new();
    let scopes = Sys::new(Scopes::new(sim_sampling_frequency as f64, &mut monitor)?).build()?;

    let m1_rbm = Signals::new(6 * 7, n_step).channel(2, Signal::Constant(1e-6));
    // .channel(36 + 2, Signal::Constant(1e-6));
    let m1_rbm_lpf = LowPassFilter::new(6 * 7, M1_RBM_LPF_GAIN);

    // Integrated Model
    let metronome: Timer = Timer::new(sim_sampling_frequency);
    actorscript! {
        #[model(name = stage1)]
        #[labels(m1_rbm = "M1 RBM\nCommand", m1_rbm_lpf = "LPF (1e-3)")]
        1: metronome[Tick] -> {gmt_servos::GmtFem}
        1: m1_rbm[RBMCmd] -> m1_rbm_lpf[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> {scopes::LinearOpticalModel}
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> {scopes::LinearOpticalModel}
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> {scopes::M1Lom}
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> {scopes::M2Lom}
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes].. -> {scopes::M2RBLom}
        1: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion].. -> {scopes::M2SegmentActuatorAverage}
    }

    // Integrated Model
    // Voice coils displacements to rigid body motions
    let asms_to_pos = Sys::new(AsmsToHexOffload::new(ASM_OFFLOAD_GAIN)?).build()?;
    // Rigid body motions to facesheet displacements
    let edge_sensors_feedfwd = Sys::new(EdgeSensorsFeedForward::new(ASM_LPF_GAIN)?).build()?;
    type Operatorf64 = Operator<f64>;
    type LowPassFilterf64 = LowPassFilter<f64>;
    let metronome: Timer = Timer::new(sim_sampling_frequency);
    actorscript! {
        #[model(name = stage2)]
        #[labels(asms_cmd = "ASMS Actuators\nCommand",
        m1_rbm = "M1 RBM\nCommand", m1_rbm_lpf = "LPF (1e-3)")]
        1: metronome[Tick] -> {gmt_servos::GmtFem}

        1: m1_rbm[RBMCmd] -> m1_rbm_lpf[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}

        1: {gmt_servos::GmtFem}[M1EdgeSensorsAsRbms]! -> {edge_sensors_feedfwd::RbmToShell}

        // send the ASMS command (actuator displacement) to the ASMS controller
        1: asms_cmd[Left<M2ASMAsmCommand>] -> {edge_sensors_feedfwd::Operatorf64}
        1: {edge_sensors_feedfwd::LowPassFilterf64}[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        // read the voice coil displacement from the FEM
        1: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion]!
            // transfrom them to rigid body motions (RBMS)
            -> {asms_to_pos}[M2RigidBodyMotions] -> {gmt_servos::GmtM2Hex}

        // send the edge sensors data to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[M2EdgeSensors]! -> {edge_sensors_feedfwd::M2EdgeSensorsToRbm}
        // send the reference body RBMS to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[MCM2SmHexD]! -> {edge_sensors_feedfwd::HexToRbm}

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> {scopes::LinearOpticalModel}
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> {scopes::LinearOpticalModel}
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> {scopes::M1Lom}
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> {scopes::M2Lom}
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes].. -> {scopes::M2RBLom}
        1: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion].. -> {scopes::M2SegmentActuatorAverage}
    }

    // M1 EDGE SENSORS INTEGRAL CONTROLLER:
    let m1_es_to_rbm = Sys::new(M1EdgeSensorsToRbm::new()).build()?;

    // Integrated Model
    type M1EdgeSensorsIntegrator = Integrator<M1EdgeSensors>;
    actorscript! {
        #[model(name = stage3)]
        #[labels(asms_cmd = "ASMS Actuators\nCommand",
        m1_rbm = "M1 RBM\nCommand", m1_rbm_lpf = "LPF (1e-3)")]

        1: m1_rbm[RBMCmd] -> m1_rbm_lpf[Right<RBMCmd>]
            -> {m1_es_to_rbm::Operatorf64}[assembly::M1RigidBodyMotions]
                -> {gmt_servos::GmtM1}
        1: {gmt_servos::GmtFem}[M1EdgeSensors]! -> {m1_es_to_rbm::M1EdgeSensorsIntegrator}

        1: {gmt_servos::GmtFem}[M1EdgeSensorsAsRbms]! -> {edge_sensors_feedfwd::RbmToShell}

        // send the ASMS command (actuator displacement) to the ASMS controller
        1: asms_cmd[Left<M2ASMAsmCommand>] -> {edge_sensors_feedfwd::Operatorf64}
        1: {edge_sensors_feedfwd::LowPassFilterf64}[M2ASMAsmCommand] -> {gmt_servos::GmtM2}

        // read the voice coil displacement from the FEM
        1: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion]!
            // transfrom them to rigid body motions (RBMS)
            -> {asms_to_pos}[M2RigidBodyMotions] -> {gmt_servos::GmtM2Hex}

        // send the edge sensors data to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[M2EdgeSensors]! -> {edge_sensors_feedfwd::M2EdgeSensorsToRbm}
        // send the reference body RBMS to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[MCM2SmHexD]! -> {edge_sensors_feedfwd::HexToRbm}

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> {scopes::LinearOpticalModel}
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> {scopes::LinearOpticalModel}
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> {scopes::M1Lom}
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> {scopes::M2Lom}
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes].. -> {scopes::M2RBLom}
        1: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion].. -> {scopes::M2SegmentActuatorAverage}
    }

    monitor.await?;

    Ok(())
}
