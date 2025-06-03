/*!
# M2 ASMS Edge Sensors Feed-Forward and Shell Off-load

Model of the GMT servo-mechanisms with M2 edge sensors feed-forward to the the
voice coil actuators and off-load from the voice coil actuators to the reference bodies.

*/
use anyhow::Result;
use edge_sensors::{
    AsmsToHexOffload, EdgeSensorsFeedForward, HexToRbm, M1EdgeSensorsAsRbms, M1EdgeSensorsToRbm,
    M2EdgeSensorsToRbm, N_ACTUATOR, N_SCOPE, RbmToShell,
    segment_piston::{M1Lom, M2Lom, M2RBLom, M2SegmentActuatorAverage, Scopes},
};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    integrator::Integrator,
    low_pass_filter::LowPassFilter,
    operator::{Left, Operator, Right},
    signals::{Signal, Signals},
    timer::Timer,
};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_fem::outputs::MCM2SmHexD,
    gmt_m1::{M1EdgeSensors, M1RigidBodyMotions, assembly},
    gmt_m2::{
        M2EdgeSensors, M2RigidBodyMotions,
        asm::{M2ASMAsmCommand, M2ASMReferenceBodyNodes, M2ASMVoiceCoilsMotion},
    },
    optics::{MaskedWavefront, SegmentD21PistonRSS, TipTilt, Wavefront},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_scope::server::{GmtShot, Monitor, Scope, Shot};
use gmt_dos_clients_servos::{
    AsmsServo, EdgeSensors, GmtFem, GmtM1, GmtM2, GmtM2Hex, GmtServoMechanisms, WindLoads,
    asms_servo,
};
use gmt_dos_clients_windloads::{
    CfdLoads,
    system::{M1, M2, Mount, SigmoidCfdLoads},
};
use gmt_fem::FEM;
use interface::{
    Tick,
    filing::Filing,
    units::{Mas, NM},
};
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
    let (cfd_loads, gmt_servos) = {
        let mut fem = Option::<FEM>::None;
        // The CFD wind loads must be called next afer the FEM as it is modifying the FEM CFDMountWindLoads inputs
        let cfd_loads = Sys::<SigmoidCfdLoads>::from_data_repo("preloaded_windloads.bin")?;

        let gmt_servos =
            Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo("preloaded_servos.bin")?;

        (cfd_loads, gmt_servos)
    };

    let asms_cmd = Signals::from((vec![0f64; N_ACTUATOR * 7], n_step));

    // SCOPES
    let mut monitor = Monitor::new();
    let scopes = Sys::new(Scopes::new(sim_sampling_frequency as f64, &mut monitor)?).build()?;
    let d21_scope = Scope::<SegmentD21PistonRSS<-9>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64 / N_SCOPE as f64)
        .build()?;
    let tiptilt_scope = Scope::<Mas<TipTilt>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64 / N_SCOPE as f64)
        .build()?;
    let wft_scope = Shot::<NM<Wavefront>>::builder(&mut monitor, [512; 2])
        .sampling_frequency(10f64)
        .build()?;

    let lom = LinearOpticalModel::new()?;

    let m1_rbm = Signals::new(6 * 7, n_step).channel(2, Signal::Constant(1e-6));
    // .channel(36 + 2, Signal::Constant(1e-6));
    let m1_rbm_lpf = LowPassFilter::new(6 * 7, M1_RBM_LPF_GAIN);

    /*     // Integrated Model
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
    } */

    // Voice coils displacements to rigid body motions
    let asms_to_pos = Sys::new(AsmsToHexOffload::new(ASM_OFFLOAD_GAIN)?).build()?;
    // Rigid body motions to facesheet displacements
    let edge_sensors_feedfwd = Sys::new(EdgeSensorsFeedForward::new(ASM_LPF_GAIN)?).build()?;
    type Operatorf64 = Operator<f64>;
    type LowPassFilterf64 = LowPassFilter<f64>;

    // Integrated Model
    // let metronome: Timer = Timer::new(sim_sampling_frequency);
    actorscript! {
        #[model(name = windloading)]
        #[labels(asms_cmd = "ASMS Actuators\nCommand",
        m1_rbm = "M1 RBM\nCommand", m1_rbm_lpf = "LPF (1e-3)")]
        // 1: metronome[Tick] -> {gmt_servos::GmtFem}

        1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}

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

        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> lom
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> lom
        8: lom[Mas<TipTilt>] -> tiptilt_scope
        800: lom[NM<Wavefront>] -> wft_scope
    }

    /*     // M1 EDGE SENSORS INTEGRAL CONTROLLER:
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
    } */

    monitor.await?;

    Ok(())
}
