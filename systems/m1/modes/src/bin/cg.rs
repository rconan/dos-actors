use std::{fs::File, time::Instant};

use faer::Mat;
use faer_ext::IntoFaer;
use gmt_dos_clients_fem::{Model, Switch, fem_io::GetOut};
use gmt_dos_clients_io::gmt_m1::segment::{
    ActuatorAppliedForces, BarycentricForce, HardpointsMotion,
};
use gmt_dos_clients_m1_ctrl::{Actuators, LoadCells};
use gmt_dos_systems_m1::{Calibration, SegmentSingularModes};
use gmt_fem::{FEM, FemError};
use interface::{Data, Read, Update, Write};
use serde::{Deserialize, Serialize};

const SID: u8 = 1;
const MID: usize = 6;

fn main() -> anyhow::Result<()> {
    println!("loading the fem ...");
    let now = Instant::now();
    let mut fem = FEM::from_env()?;
    println!("elapsed: {:}s", now.elapsed().as_secs());

    println!("loading the modes...");
    let now = Instant::now();
    let modes: Vec<SegmentSingularModes> =
        serde_pickle::from_reader(File::open("m1_singular_modes.pkl")?, Default::default())?;
    println!("elapsed: {:}s", now.elapsed().as_micros());
    dbg!(modes[0].shape());
    let m2f = modes[0].mode2force_mat_ref();
    dbg!(m2f.shape());

    // M1 actuator forces
    let fa = 1e-6 * m2f;

    let sid = SID as usize;
    let input = format!("M1_actuators_segment_{sid}");
    let output = format!("M1_segment_{sid}_axial_d");
    let g_d = fem_gain(&mut fem, &input, &output)?;
    dbg!(&g_d.shape());

    let idx = Box::<dyn GetOut>::try_from(output.clone())
        .map(|x| x.position(&fem.outputs))?
        .unwrap();
    let xy: Vec<_> = fem.outputs[idx]
        .as_ref()
        .map(|i| i.get_by(|i| i.properties.location.clone()))
        .expect(&format!("failed to read nodes locations from {output}"))
        .into_iter()
        .flat_map(|xyz| xyz[..2].to_vec())
        .collect();

    // M1 surface figure
    let shapes = &g_d * &fa;

    // M1 hardpoint displacements
    let g_h = fem_gain(&mut fem, &input, "OSS_Hardpoint_D")?;
    let hp = &g_h * &fa;
    dbg!(hp.shape());

    let mid = MID - 1;
    // M1 S1 figure
    let d1: Vec<f64> = shapes.col(mid).iter().cloned().collect();
    // M1 S1 HP
    let hp1: Vec<f64> = hp
        .col(mid)
        .iter()
        .skip((sid - 1) * 12)
        .take(12)
        .cloned()
        .collect();

    // M1 controller
    let m1_calib = Calibration::new(&mut fem);
    let mut loadcells = LoadCells::new(m1_calib.stiffness, m1_calib.lc_2_cg[0]);
    let mut actuators = Actuators::<SID>::new();

    <_ as Read<HardpointsMotion<SID>>>::read(&mut loadcells, Data::new(hp1));
    loadcells.update();
    let cg = <_ as Write<BarycentricForce<SID>>>::write(&mut loadcells).unwrap();
    dbg!(&cg);
    <_ as Read<BarycentricForce<SID>>>::read(&mut actuators, cg);
    actuators.update();
    let cg_fa = <_ as Write<ActuatorAppliedForces<SID>>>::write(&mut actuators).unwrap();
    let na = cg_fa.len();
    let cg_fa =
        faer::MatRef::from_column_major_slice(cg_fa.into_arc().as_slice(), na, 1).to_owned();
    let cg_d1 = &g_d * cg_fa;
    let cg_d1: Vec<f64> = cg_d1.col(0).iter().cloned().collect();

    let cg_data = CgData {
        xy,
        modal_surface: d1,
        cg_surface: cg_d1,
    };
    serde_pickle::to_writer(
        &mut File::create(format!("cg_data_m1s{SID}m{MID}.pkl"))?,
        &cg_data,
        Default::default(),
    )?;

    Ok(())
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct CgData {
    xy: Vec<f64>,
    modal_surface: Vec<f64>,
    cg_surface: Vec<f64>,
}

fn fem_gain(fem: &mut FEM, input: &str, output: &str) -> Result<Mat<f64>, FemError> {
    let inputs = vec![input.to_string()];
    let outputs = vec![output.to_string()];
    println!("extracting the static gain {:?} -> {:?}", inputs, outputs);
    fem.switch_inputs(Switch::Off, None)
        .switch_outputs(Switch::Off, None);
    // M1 actuator forces to M1 surface displacement gain
    let gain_d = fem
        .switch_inputs_by_name(inputs.clone(), Switch::On)
        .and_then(|fem| fem.switch_outputs_by_name(outputs.clone(), Switch::On))
        .map(|fem| fem.reduced_static_gain().unwrap())?;
    let gain = gain_d.view_range(.., ..).into_faer();
    Ok(gain.to_owned())
}
