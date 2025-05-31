use anyhow::{Context, Result};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    Gain, Integrator, Signal, Signals,
    operator::{Left, Operator, Right},
    select::Select,
};
use gmt_dos_clients_io::gmt_m2::asm::segment::VoiceCoilsMotion;
use gmt_dos_clients_io::{
    gmt_fem::outputs::M2EdgeSensors,
    gmt_m2::{
        M2PositionerForces, M2PositionerNodes, M2RigidBodyMotions,
        asm::{
            M2ASMAsmCommand, M2ASMReferenceBodyNodes, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
        },
    },
    optics::{M2modes, SegmentD21PistonRSS, SegmentPiston, SegmentTipTilt, Wavefront, WfeRms},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_servos::{
    AsmsServo, EdgeSensors, GmtFem, GmtM2, GmtM2Hex, GmtServoMechanisms, asms_servo,
};
use gmt_fem::FEM;
use interface::filing::Filing;
use interface::{
    Data, Read, UID, Update, Write,
    units::{Mas, NM},
};
use matio_rs::MatFile;
use nalgebra as na;
use std::{env, path::Path, sync::Arc};

const ACTUATOR_RATE: usize = 80; // 100Hz

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let data_repo = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("examples")
        .join("m2-edge-sensors");
    env::set_var("DATA_REPO", &data_repo);
    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set");
    let fem_path = Path::new(&fem_var);

    let sim_sampling_frequency = 8000;
    let m1_freq = 100; // Hz
    assert!(m1_freq == sim_sampling_frequency / ACTUATOR_RATE);
    let sim_duration = 2_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    // KARHUNEN-LOEVE MODES
    let path = fem_path.join("KLmodesGS36p90.mat");
    let mut kl_modes: Vec<na::DMatrix<f64>> = vec![];
    let n_mode = 6;
    for i in 1..=7 {
        let mat: na::DMatrix<f64> = MatFile::load(&path)
            .context("Failed to read from KLmodesGS36p90.mat")?
            .var(format!("KL_{i}"))?;
        let (_nact, nkl) = mat.shape();
        assert!(n_mode <= nkl);
        kl_modes.push(mat.remove_columns(n_mode, nkl - n_mode));
        dbg!(kl_modes.last().unwrap().shape());
    }

    // EDGE SENSORS
    //  * M1 EDGE SENSORS NODES
    let es_nodes_2_data: na::DMatrix<f64> =
        MatFile::load(fem_path.join("M1_edge_sensor_conversion.mat"))
            .context("Failed to read from M1_edge_sensor_conversion.mat")?
            .var("A1")?;
    //  * EDGE SENSORS TO RIGID-BODY MOTIONS TRANSFORM (M1 & M2)
    let (es_2_m1_rbm, es_2_m2_rbm) = {
        let mat = MatFile::load(fem_path.join("m12_e_rs").join("m12_r_es.mat"))
            .context("Failed to read from m12_r_es.mat")?;
        let m1_es_recon: na::DMatrix<f64> = mat.var("m1_r_es")?;
        let m2_es_recon: na::DMatrix<f64> = mat.var("m2_r_es")?;
        (
            m1_es_recon.insert_rows(36, 6, 0f64) * es_nodes_2_data,
            m2_es_recon, //.insert_rows(36, 6, 0f64),
        )
    };
    //  * M1 RIGID-BODY MOTIONS TO ASMS KARHUNEN-LOEVE MODES TRANSFORM
    let m1_rbm_2_mode: na::DMatrix<f64> = MatFile::load(&fem_path.join("m1_rbm_2_asm_kl.mat"))
        .context("Failed to read from m1_rbm_2_asm_kl.mat")?
        .var("r2kl")?;
    dbg!(m1_rbm_2_mode.shape());
    //  * M2 RIGID-BODY MOTIONS TO ASMS KARHUNEN-LOEVE MODES TRANSFORM
    // let m2_rbm_2_mode: na::DMatrix<f64> =
    //     MatFile::load(&fem_path.join("m2_rbm_2_asm_kl.mat"))?.var("r2kl")?;
    let m2_rbm_2_mode: na::DMatrix<f64> = MatFile::load(
        "/home/rconan/Dropbox/projects/dos-actors/clients/fem/mech/data/m2_rbm_2_asm_kl-7.mat",
    )
    .context("Failed to read from m2_rbm_2_asm_kl-7.mat")?
    .var("r2kl")?;
    // for i in 0..6 {
    //     m2_rbm_2_mode.swap_columns(i + 3, i + 4);
    // }
    dbg!(m2_rbm_2_mode.shape());
    // let es_2_m2_mode = &m2_rbm_2_mode * &es_2_m2_rbm;
    // dbg!(es_2_m2_mode.shape());
    let r7_2_r: na::DMatrix<f64> = MatFile::load(
        "/home/rconan/Dropbox/projects/dos-actors/clients/fem/mech/data/m12_e_rs/m2_r7_r.mat",
    )
    .context("Failed to read from m2_r7_r.mat")?
    .var("m2_r7_r")?;
    dbg!(r7_2_r.shape());
    let r7_2_es: na::DMatrix<f64> = MatFile::load(
        "/home/rconan/Dropbox/projects/dos-actors/clients/fem/mech/data/m12_e_rs/m2_r7_es.mat",
    )
    .context("Failed to read from m2_r7_es.mat")?
    .var("m2_r7_es")?;
    dbg!(r7_2_es.shape());

    // ASM PTT TO M2 RBM (0 0 Tz Rx Ry 0)
    let ptt_2_rbm_mat: na::DMatrix<f64> = MatFile::load(
        "/home/rconan/Dropbox/projects/dos-actors/clients/fem/mech/data/asm_ptt_2_m2_tzrxry.mat",
    )
    .context("Failed to read from asm_ptt_2_m2_tzrxry.mat")?
    .var("r2kl")?;
    dbg!(ptt_2_rbm_mat.shape());

    let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_data_repo_or_else(
        "edge-sensors_servos.bin",
        || {
            GmtServoMechanisms::new(
                sim_sampling_frequency as f64,
                FEM::from_env().expect("failed to load the FEM"),
            )
            .asms_servo(
                AsmsServo::new()
                    .voice_coils(asms_servo::VoiceCoils::new(kl_modes))
                    .reference_body(asms_servo::ReferenceBody::new()),
            )
            .edge_sensors(EdgeSensors::both())
        },
    )?;

    const SID: u8 = 7;

    let m2_rbm = Signals::new(6 * 7, n_step);
    let asm_cmd = Signals::new(n_mode * 7, n_step)
        .channel(0, Signal::Constant(100e-6))
        .channel(
            (SID as usize - 1) * n_mode,
            Signal::Constant(100e-6), /*         Signal::Sinusoid {
                                          amplitude: 50e-6,
                                          sampling_frequency_hz: sim_sampling_frequency as f64,
                                          frequency_hz: 1.,
                                          phase_s: 0.,
                                      } + Signal::Sinusoid {
                                          amplitude: 50e-6,
                                          sampling_frequency_hz: sim_sampling_frequency as f64,
                                          frequency_hz: 10.,
                                          phase_s: 0.01,
                                      }, */
        );

    // LINEAR OPTICAL MODELS
    let lom = LinearOpticalModel::new()?;
    // let m1_lom = LinearOpticalModel::new()?;
    // let asm_shell_lom = LinearOpticalModel::new()?;
    // let asm_rb_lom = LinearOpticalModel::new()?;

    // EDGE SENSORS INTEGRAL CONTROLLERS:
    //  * M1
    // let m1_es_int = Integrator::new(42).gain(0.2);
    //  * M2
    let m2_es_int = Integrator::new(42).gain(0.2);

    // let m1_add = Operator::new("+");
    let m2_add = Operator::new("+");

    // RIGID-BODY MOTIONS 2 ASMS KARHUNEN-LOEVE MODES TRANSFORM
    //  * M1
    // let m1_rbm_2_kls = Gain::new(
    //     m1_rbm_2_mode
    //         .insert_columns(36, 6, 0f64)
    //         .insert_rows(36, 6, 0f64),
    // );
    //  * M2

    // let m2_rbm_2_kls = Gain::new(
    //     m2_rbm_2_mode
    //         .clone()
    //         .insert_columns(36, 6, 0f64)
    //         .insert_rows(36, 6, 0f64),
    // );
    let m2_rbm_2_kls = Gain::new(m2_rbm_2_mode.clone().insert_columns(41, 1, 0f64));

    let add_asm_cmd = Operator::new("+");

    dbg!(es_2_m2_rbm.shape());
    let es_2_m2_rbm_c = Gain::new(es_2_m2_rbm.clone().insert_rows(36, 6, 0f64));

    // let ptt = Select::new(0..3);
    let tzrxry = Select::<f64>::new((0..7).map(|i| 2 + i * 6).collect::<Vec<_>>());

    let ptt_to_rbm = AsmsPtt2TxRxRz::new(ptt_2_rbm_mat);

    let asms_offloading = AsmsOffLoading::new(m2_rbm_2_mode, r7_2_es, es_2_m2_rbm);

    actorscript! {
    #[labels(m2_rbm_2_kls = "M2 RBM\nto\nASMS KLS",
    ptt_to_rbm = "PTT to RBM", m2_es_int = "RBM\nIntegrator")]

    // 1: asm_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}


    // ASMS positioners feedback loop
    1: m2_rbm[Right<M2RigidBodyMotions>] ->  m2_add[M2RigidBodyMotions]
            -> {gmt_servos::GmtM2Hex}
    // ASMS voice coils feedback loop
    1: asm_cmd[Right<M2ASMAsmCommand>] -> add_asm_cmd[M2ASMAsmCommand]
             -> {gmt_servos::GmtM2}
    // ASMS edge sensors feedback loop to ASMS positioners
    // 500: plant[M2EdgeSensors]! -> m2_es_int

    1: m2_add[M2RigidBodyMotions] -> asms_offloading
    1: {gmt_servos::GmtFem}[M2EdgeSensors]!-> asms_offloading[EdgeSensorsAsRbms]//${42}
    // 32: asms_offloading[EdgeSensorsAsRbms]~
    // 1: asms_offloading[EdgeSensorsAsRbms] -> m2_rbm_2_kls

    500: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion]!
        -> ptt_to_rbm[M2RigidBodyMotions]${42}
            -> m2_es_int
    1: m2_es_int[Left<M2RigidBodyMotions>]!->  m2_add
     // M2 edge sensors feed-forward loop to ASMS KL modes
    //  1: plant[M2EdgeSensors]! -> es_2_m2_rbm_c[EdgeSensorsAsRbms] -> m2_rbm_2_kls
     1: asms_offloading[EdgeSensorsAsRbms] ->  m2_rbm_2_kls
     8: m2_rbm_2_kls[Left<EdgeSensorsAsRbms>] ->  add_asm_cmd


    1: {gmt_servos::GmtFem}[NM<VoiceCoilsMotion<1>>]~
    1: {gmt_servos::GmtFem}[NM<VoiceCoilsMotion<2>>]~
    1: {gmt_servos::GmtFem}[NM<VoiceCoilsMotion<3>>]~
    1: {gmt_servos::GmtFem}[NM<VoiceCoilsMotion<SID>>]~
    1: {gmt_servos::GmtFem}[NM<M2ASMReferenceBodyNodes>] -> tzrxry[Tz]~
    }

    /*     actorscript! {
            #[model(name = model, state = completed)]
            #[labels(m2_rbm_2_kls = "M2 RBM\nto\nASMS KLS",
                     ptt_to_rbm = "PTT to RBM", m2_es_int = "RBM\nIntegrator")]

            // ASMS positioners feedback loop
            1: m2_rbm[Right<M2RigidBodyMotions>] ->  m2_add[M2RigidBodyMotions]
                    -> {gmt_servos::GmtM2Hex}
            // ASMS voice coils feedback loop
            1: asm_cmd[Right<M2ASMAsmCommand>] -> add_asm_cmd[M2ASMAsmCommand]
                     -> {gmt_servos::GmtM2}
            // ASMS edge sensors feedback loop to ASMS positioners
            // 500: plant[M2EdgeSensors]! -> m2_es_int

    /*         1: m2_add[M2RigidBodyMotions] -> asms_offloading
            1: {gmt_servos::GmtFem}[M2EdgeSensors]!-> asms_offloading[EdgeSensorsAsRbms]${42}
            // 32: asms_offloading[EdgeSensorsAsRbms]~
            // 1: asms_offloading[EdgeSensorsAsRbms] -> m2_rbm_2_kls

            500: {gmt_servos::GmtFem}[M2ASMVoiceCoilsMotion]! -> ptt_to_rbm[M2RigidBodyMotions] -> m2_es_int
            1: m2_es_int[Left<M2RigidBodyMotions>]!->  m2_add
             // M2 edge sensors feed-forward loop to ASMS KL modes
            //  1: plant[M2EdgeSensors]! -> es_2_m2_rbm_c[EdgeSensorsAsRbms] -> m2_rbm_2_kls
             1: asms_offloading[EdgeSensorsAsRbms] ->  m2_rbm_2_kls
             8: m2_rbm_2_kls[Left<EdgeSensorsAsRbms>] ->  add_asm_cmd */

            32: {gmt_servos::GmtFem}[VoiceCoilsMotion<1>]!~
            32: {gmt_servos::GmtFem}[VoiceCoilsMotion<2>]!~
            32: {gmt_servos::GmtFem}[VoiceCoilsMotion<3>]!~
            32: {gmt_servos::GmtFem}[VoiceCoilsMotion<SID>]!~
            32: {gmt_servos::GmtFem}[M2ASMReferenceBodyNodes] -> tzrxry[Tz] ~

            // 1: plant[M1RigidBodyMotions].. -> lom
            // 1: plant[M2RigidBodyMotions].. -> lom
            // 32: lom[WfeRms<-9>]~

        } */

    Ok(())
}

#[derive(UID)]
pub enum LpfSegmentTiptilt {}
#[derive(UID)]
pub enum LpfIntSegmentTiptilt {}
#[derive(UID)]
pub enum SegmentTiptilt7 {}
#[derive(UID)]
#[uid(port = 5001)]
pub enum KLs {}
#[derive(UID)]
#[uid(port = 5002)]
pub enum TzRxRy {}
#[derive(UID)]
#[uid(port = 5003)]
pub enum Tz {}
#[derive(UID)]
pub enum EdgeSensorsAsRbms {}

pub struct AsmsOffLoading {
    // 36x36
    rbm_2_mode: na::DMatrix<f64>,
    // 36x6
    r7_2_es: na::DMatrix<f64>,
    // 36x48
    es_2_r: na::DMatrix<f64>,
    // 42
    rbms: Arc<Vec<f64>>,
    // 42
    edge_sensors: Arc<Vec<f64>>,
    // 42
    data: Arc<Vec<f64>>,
}
impl AsmsOffLoading {
    pub fn new(
        rbm_2_mode: na::DMatrix<f64>,
        r7_2_es: na::DMatrix<f64>,
        es_2_r: na::DMatrix<f64>,
    ) -> Self {
        Self {
            rbm_2_mode,
            r7_2_es,
            es_2_r,
            rbms: Default::default(),
            edge_sensors: Default::default(),
            data: Default::default(),
        }
    }
}
impl Update for AsmsOffLoading {
    fn update(&mut self) {
        let r7 = &self.rbms[36..];
        let es_from_r7 = &self.r7_2_es * na::DVector::from_column_slice(r7);
        // let rbm_2_mode =
        //     &self.rbm_2_mode * na::DVector::from_column_slice(&self.edge_sensors[..36]);
        let data: Vec<_> = self
            .edge_sensors
            .iter()
            .zip(es_from_r7.into_iter())
            .map(|(x, y)| x - y)
            .collect();
        let rbm = &self.es_2_r * na::DVector::from_column_slice(&data);
        let data: Vec<_> = rbm
            .into_iter()
            .map(|x| *x)
            .chain(r7.into_iter().map(|x| *x))
            .collect::<Vec<_>>();
        self.data = Arc::new(data);
    }
}
impl Read<M2RigidBodyMotions> for AsmsOffLoading {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.rbms = data.into_arc();
    }
}
impl Read<M2EdgeSensors> for AsmsOffLoading {
    fn read(&mut self, data: Data<M2EdgeSensors>) {
        self.edge_sensors = data.into_arc();
    }
}
impl Write<EdgeSensorsAsRbms> for AsmsOffLoading {
    fn write(&mut self) -> Option<Data<EdgeSensorsAsRbms>> {
        Some(self.data.clone().into())
    }
}

pub struct AsmsPtt2TxRxRz {
    transform: na::DMatrix<f64>,
    modes: Arc<Vec<Arc<Vec<f64>>>>,
    rbm: Vec<f64>,
}

impl AsmsPtt2TxRxRz {
    pub fn new(transform: na::DMatrix<f64>) -> Self {
        Self {
            transform,
            modes: Default::default(),
            rbm: vec![0f64; 42],
        }
    }
}

impl Update for AsmsPtt2TxRxRz {
    fn update(&mut self) {
        for sid in 0..7 {
            let ptt: Vec<_> = self.modes[sid].iter().take(3).cloned().collect();
            let m = na::DVector::from_column_slice(&ptt);
            let tzxry = &self.transform * m;
            self.rbm
                .chunks_mut(6)
                .nth(sid)
                .unwrap()
                .iter_mut()
                .skip(2)
                .take(3)
                .zip(tzxry.as_slice())
                .for_each(|(rbm, v)| *rbm = *v);
        }
    }
}

impl Read<M2ASMVoiceCoilsMotion> for AsmsPtt2TxRxRz {
    fn read(&mut self, data: Data<M2ASMVoiceCoilsMotion>) {
        self.modes = data.into_arc();
    }
}

impl Write<M2RigidBodyMotions> for AsmsPtt2TxRxRz {
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        Some(self.rbm.clone().into())
    }
}
