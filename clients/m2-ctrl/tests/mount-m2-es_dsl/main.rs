use gmt_dos_actors::{actorscript, prelude::*, system::Sys};
use gmt_dos_clients::{Logging, Signal, Signals};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix, Model, Switch};
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::MCM2SmHexF,
        outputs::{M2EdgeSensors, MCM2Lcl6D, MCM2SmHexD},
    },
    gmt_m2::{
        M2PositionerForces, M2PositionerNodes, M2RigidBodyMotions,
        asm::{
            M2ASMAsmCommand, M2ASMFluidDampingForces, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
            segment::VoiceCoilsMotion,
        },
    },
    mount::{MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_m2_ctrl::{ASMS, positioner::AsmsPositioners};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use interface::{Data, Read, UID, UniqueIdentifier, Update, Write};
use matio_rs::MatFile;
use nalgebra as na;
use std::{env, path::Path, sync::Arc, time::Instant};

#[derive(Debug, Default)]
pub struct Multiplex {
    data: Arc<Vec<f64>>,
    slices: Vec<usize>,
}
impl Multiplex {
    fn new(slices: Vec<usize>) -> Self {
        Self {
            slices,
            ..Default::default()
        }
    }
}
#[derive(UID)]
pub enum RBMCmd {}
#[derive(UID)]
pub enum ActuatorCmd {}

impl Update for Multiplex {}
impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for Multiplex {
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}
impl<U: UniqueIdentifier<DataType = Vec<Arc<Vec<f64>>>>> Write<U> for Multiplex {
    fn write(&mut self) -> Option<Data<U>> {
        let mut mx_data = vec![];
        let data = self.data.as_slice();
        let mut a = 0_usize;
        for s in &self.slices {
            let b = a + *s;
            mx_data.push(Arc::new(data[a..b].to_vec()));
            a = b;
        }
        Some(mx_data.into())
    }
}

/*
export FEM_REPO=/home/rconan/mnt/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/
cargo test  --release  --package gmt_dos-clients_m2-ctrl --features serde --test mount-m2-es_dsl -- main --exact --nocapture */

#[tokio::test]
async fn main() -> anyhow::Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let sim_sampling_frequency = 8000;
    let sim_duration = 3_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = FEM::from_env()?;

    let positioners = AsmsPositioners::from_fem(&mut fem)?;

    // println!("{fem}");
    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set");
    let fem_path = Path::new(&fem_var);

    let path = fem_path.join("KLmodesGS36p.mat");
    let now = Instant::now();
    let mut vc_f2d = vec![];
    let mut kl_modes: Vec<na::DMatrix<f64>> = vec![];
    let mut asms_nact = vec![];
    let n_mode = 6;
    for i in 1..=7 {
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);

        vc_f2d.push(
            fem.switch_inputs_by_name(vec![format!("MC_M2_S{i}_VC_delta_F")], Switch::On)
                .and_then(|fem| {
                    fem.switch_outputs_by_name(vec![format!("MC_M2_S{i}_VC_delta_D")], Switch::On)
                })
                .map(|fem| {
                    fem.reduced_static_gain()
                        .unwrap_or_else(|| fem.static_gain())
                })?,
        );
        // println!("{:?}", vc_f2d.shape());
        let mat: na::DMatrix<f64> = MatFile::load(&path)?.var(format!("KL_{i}"))?;
        let (nact, nkl) = mat.shape();
        assert!(n_mode <= nkl);
        asms_nact.push(n_mode);
        kl_modes.push(mat.remove_columns(n_mode, nkl - n_mode));
    }
    fem.switch_inputs(Switch::On, None)
        .switch_outputs(Switch::On, None);
    println!("stiffness from FEM in {}ms", now.elapsed().as_millis());
    /*     let mat = MatFile::load(
        Path::new(&env::var("CARGO_MANIFEST_DIR")?)
            .join("tests")
            .join("mount-m1a-es_dsl")
            .join("M1_edge_sensor_conversion.mat"),
    )?;
    let es_nodes_2_data: nalgebra::DMatrix<f64> = mat.var("A1")?; */
    let mat = MatFile::load(
        Path::new(&env::var("CARGO_MANIFEST_DIR")?)
            .join("tests")
            .join("mount-m2-es_dsl")
            .join("m12_r_es.mat"),
        // .join("M1M2ESRecs.mat"),
    )?;
    let m2_es_recon: nalgebra::DMatrix<f64> = mat.var("m2_r_es")?;
    let es_2_rbm = m2_es_recon;
    dbg!(es_2_rbm.shape());
    let mat = MatFile::load(
        Path::new(&env::var("CARGO_MANIFEST_DIR")?)
            .join("tests")
            .join("mount-m2-es_dsl")
            .join("m2_rbm_2_asm_kl.mat"),
    )?;
    let rbm_2_mode: nalgebra::DMatrix<f64> = mat.var("r2kl")?;
    dbg!(rbm_2_mode.shape());
    let es_2_mode = rbm_2_mode * &es_2_rbm;

    let kl_modes_t = kl_modes.iter().map(|x| x.transpose()).collect::<Vec<_>>();

    let sids = vec![1, 2, 3, 4, 5, 6, 7];
    let fem_dss = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        // .truncate_hankel_singular_values(1e-7)
        // .hankel_frequency_lower_bound(50.)
        .including_mount()
        .including_asms(
            Some(sids.clone()),
            Some(kl_modes.iter().map(|x| x.as_view()).collect()),
            Some(kl_modes_t.iter().map(|x| x.as_view()).collect()),
        )?
        .ins::<MCM2SmHexF>()
        .outs::<MCM2SmHexD>()
        .outs::<MCM2Lcl6D>()
        .outs_with::<M2EdgeSensors>(es_2_mode.as_view())
        .use_static_gain_compensation()
        .build()?;
    println!("{fem_dss}");

    let plant = fem_dss;
    // .image("../icons/fem.png");

    let rbm_fun =
        |i: usize, sid: u8| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64 + sid as f64 / 10_f64;
    /*     let rbm = (1..=6).fold(Signals::new(6 * 7, n_step), |signals_sid, sid| {
        (0..6).fold(signals_sid, |signals, i| {
            signals.channel(
                i + 6 * (sid - 1) as usize,
                Signal::Sigmoid {
                    amplitude: rbm_fun(i, sid) * 1e-6,
                    sampling_frequency_hz: sim_sampling_frequency as f64,
                },
            )
        })
    }); */
    // let rbm = Signals::new(6 * 7, n_step).channel(
    //     2,
    //     Signal::Sigmoid {
    //         amplitude: 1e-6,
    //         sampling_frequency_hz: sim_sampling_frequency as f64,
    //     },
    // );

    let rbm_mx = Multiplex::new(vec![6; 7]);

    let ks: Vec<_> = vc_f2d.iter().map(|x| Some(x.as_slice().to_vec())).collect();
    let mut asms = ASMS::<1>::new(asms_nact, ks)?;

    // MOUNT CONTROL
    let mount_setpoint = Signals::new(3, n_step);
    let mount = Mount::new();

    let m2_rbm = Signals::new(6 * 7, n_step).channel(2, Signal::Constant(1e-6));

    actorscript! {
        1: mount_setpoint[MountSetPoint] -> mount[MountTorques] -> plant[MountEncoders]! -> mount

        1: m2_rbm[M2RigidBodyMotions] -> positioners[M2PositionerForces] -> plant[M2PositionerNodes]! -> positioners

        // 1: rbm[RBMCmd] -> rbm_mx[assembly::M2RigidBodyMotions]
        //     -> {asms}[assembly::M1HardpointsForces]
        //         -> plant[assembly::M1HardpointsMotion]! -> {m1}
        // 1: actuators[ActuatorCmd]
        //     -> actuators_mx[assembly::M1ActuatorCommandForces]
        //         -> {m1}[assembly::M1ActuatorAppliedForces] -> plant

        // 1: rbm[RBMCmd] -> rbm_mx[assembly::M2RigidBodyMotions]
        //     -> asms_mx[M2ASMAsmCommand]

        // 1: {asms}[M2ASMVoiceCoilsForces]-> plant
        // 1: {asms}[M2ASMFluidDampingForces] -> plant[M2ASMVoiceCoilsMotion]! -> {asms}
    }

    let mut plant_lock = plant.lock().await;

    println!("RIGID BODY MOTIONS:");
    let rbms = <DiscreteModalSolver<ExponentialMatrix> as Write<M2RigidBodyMotions>>::write(
        &mut plant_lock,
    )
    .unwrap();

    let rbm_err = rbms
        .chunks(6)
        .map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
        .enumerate()
        .inspect(|(i, x)| println!("{:2}: {:+.1?}", i, x))
        .map(|(i, x)| {
            x.iter()
                .enumerate()
                .map(|(j, x)| x - rbm_fun(j, i as u8 + 1))
                .map(|x| x * x)
                .sum::<f64>()
                / 6f64
        })
        .map(|x| x.sqrt())
        .sum::<f64>()
        / 7f64;
    // assert!(dbg!(rbm_err) < 5e-2);

    println!("EDGE SENSORS:");
    let es =
        <DiscreteModalSolver<ExponentialMatrix> as Write<M2EdgeSensors>>::write(&mut plant_lock)
            .unwrap();
    es.chunks(6)
        .map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
        .enumerate()
        .for_each(|(i, x)| println!("{:2}: {:+.3?}", i, x));

    Ok(())
}
