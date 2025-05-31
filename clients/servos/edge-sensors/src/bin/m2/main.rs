/*!
# M2 ASMS Edge Sensors Feed-Forward and Shell Off-load

Model of the GMT servo-mechanisms with M2 edge sensors feed-forward to the the
voice coil actuators and off-load from the voice coil actuators to the reference bodies.

*/
use anyhow::Result;
use edge_sensors::{HexToRbm, M2EdgeSensorsToRbm, N_ACTUATOR, RbmToShell, VoiceCoilToRbm};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    gain::Gain,
    integrator::Integrator,
    low_pass_filter::LowPassFilter,
    operator::{Left, Operator, Right},
    select::{Select, USelect},
    signals::Signals,
};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_fem::outputs::MCM2SmHexD,
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{
        M2EdgeSensors, M2RigidBodyMotions,
        asm::{M2ASMAsmCommand, M2ASMReferenceBodyNodes, M2ASMVoiceCoilsMotion},
    },
    optics::{SegmentPiston, SegmentTipTilt},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_scope::server::{Monitor, Scope};
use gmt_dos_clients_servos::{
    AsmsServo, EdgeSensors, GmtFem, GmtM2, GmtM2Hex, GmtServoMechanisms, WindLoads, asms_servo,
};
use gmt_dos_clients_windloads::{
    CfdLoads,
    system::{M1, M2, Mount, SigmoidCfdLoads},
};
use gmt_fem::FEM;
use interface::{
    filing::Filing,
    units::{Mas, NM},
};
use io::{M2ASMVoiceCoilsMotionAsRbms, M2EdgeSensorsAsRbms, M2S1Tz, M2S1VcAsTz, RbmAsShell};
use std::{env, path::Path};

const ACTUATOR_RATE: usize = 80; // 100Hz

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let data_repo = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap()).join("data");
    env::set_var("DATA_REPO", &data_repo);
    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set");
    let _fem_path = Path::new(&fem_var);

    let sim_sampling_frequency = 8000;
    let m1_freq = 100; // Hz
    assert!(m1_freq == sim_sampling_frequency / ACTUATOR_RATE);
    let sim_duration = 2_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = Option::<FEM>::None;

    // Voice coils displacements to rigid body motions
    let voice_coil_to_rbm = VoiceCoilToRbm::new()?;
    // Rigid body motions to facesheet displacements
    let rbm_2_shell = RbmToShell::new()?;

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
                .edge_sensors(EdgeSensors::both())
            },
        )?;

        (cfd_loads, gmt_servos)
    };
    /*     let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else(
        "edge-sensors_servos.bin",
        || {
            GmtServoMechanisms::new(
                sim_sampling_frequency as f64,
                FEM::from_env().expect("failed to load the FEM"),
            )
            .asms_servo(AsmsServo::new().reference_body(asms_servo::ReferenceBody::new()))
            .edge_sensors(EdgeSensors::both())
        },
    )?; */

    // Linear optical model
    let lom = LinearOpticalModel::new()?;

    // Low pass filter of ASMS command (actuator displacement)
    let lpf = LowPassFilter::new(N_ACTUATOR * 7, 0.05);

    // Lag-compensator? or leaky integral controller
    let lag = 2000f64.recip();
    let rbm_int = Integrator::new(42).gain(lag); //.forgetting_factor(1. - lag);

    // Substraction operator
    let substract_m2_rbms = Operator::new("-");

    // ASMS off-load to positionners
    let asms_offloading = M2EdgeSensorsToRbm::new()?;

    let asms_cmd = Signals::from((
        vec![1e-6; N_ACTUATOR]
            .into_iter()
            .chain([0f64; N_ACTUATOR].repeat(5).into_iter())
            .chain([1e-6; N_ACTUATOR].into_iter())
            .collect::<Vec<f64>>(),
        n_step,
    ));
    let asms_cmd = Signals::from((vec![0f64; N_ACTUATOR * 7], n_step));

    /// ASMS positioners to RBM
    let hex_2_rbm = HexToRbm::new()?;

    // SCOPES
    let mut monitor = Monitor::new();
    //  * WFE RMS
    let scope_sampling_frequency = sim_sampling_frequency / 32;
    let segment_piston_scope = Scope::<SegmentPiston<-9>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;
    let segment_tiptilt_scope = Scope::<Mas<SegmentTipTilt>>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;
    let ref_body_rbm_scope = Scope::<M2S1Tz>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;
    let voicecoil_rbm_scope = Scope::<M2S1VcAsTz>::builder(&mut monitor)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;

    // Select a vector element and convert it to nanometer
    let m2_s1_tz = USelect::<NM<_>>::new(2);
    let m2_s1_vc_as_tz = USelect::<NM<_>>::new(2);

    // Integrated Model
    actorscript! {
        #[labels(rbm_int = "Leaky integral\nController",
            substract_m2_rbms = "Sub",
            asms_cmd = "ASMS Actuators\nCommand",
            m2_s1_tz = "M2S1Tz", m2_s1_vc_as_tz = "M2S1Tz")]
        // 1: metronome[Tick] -> {gmt_servos::GmtFem}

        1: {cfd_loads::M1}[CFDM1WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::M2}[CFDM2WindLoads] -> {gmt_servos::GmtFem}
        1: {cfd_loads::Mount}[CFDMountWindLoads] -> {gmt_servos::GmtFem}
        // send the ASMS command (actuator displacement) to the ASMS controller
        1: asms_cmd[Left<M2ASMAsmCommand>] -> substract_m2_rbms[M2ASMAsmCommand]
            -> lpf[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        // read the voice coil displacement from the FEM
        1: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion]
            // transfrom them to rigid body motions (RBMS)
            -> voice_coil_to_rbm[M2ASMVoiceCoilsMotionAsRbms]
                // integrate the RBMS
                -> rbm_int[M2RigidBodyMotions]
                    // send the RBMS to the ASMS positioners
                    -> {gmt_servos::GmtM2Hex}

        // send the edge sensors data to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[M2EdgeSensors]! -> asms_offloading
        // send the reference body RBMS to the ASMS off-loading algorithm
        1: {gmt_servos::GmtFem}[MCM2SmHexD]! -> hex_2_rbm[M2ASMReferenceBodyNodes] -> asms_offloading[M2EdgeSensorsAsRbms]
            // transforms the RBMS to facesheet displacements
            -> rbm_2_shell[Right<RbmAsShell>] -> substract_m2_rbms

        1: voice_coil_to_rbm[M2ASMVoiceCoilsMotionAsRbms] -> m2_s1_vc_as_tz[M2S1VcAsTz] -> voicecoil_rbm_scope
        1: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes]! -> m2_s1_tz[M2S1Tz] -> ref_body_rbm_scope
        1: {gmt_servos::GmtFem}[M2RigidBodyMotions].. -> lom
        1: {gmt_servos::GmtFem}[M1RigidBodyMotions].. -> lom
        1: lom[SegmentPiston<-9>] -> segment_piston_scope
        1: lom[Mas<SegmentTipTilt>] -> segment_tiptilt_scope
    }

    monitor.await?;

    Ok(())
}
