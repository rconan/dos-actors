//! FEM STATIC GAIN
//!
//! Computes the FEM static gain between given inputs and outputs.
//!
//! M2 S7 gain matrix for the voice coils:
//! ```shell
//! cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
//!     -i MC_M2_S7_VC_delta_F -o MC_M2_S7_VC_delta_D \
//!     -f m2s7_vc_gain.pkl
//! ```
//!
//! M2 S1 & S7 gain matrix for the voice coils:
//! ```shell
//! cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
//!     -i MC_M2_S1_VC_delta_F -i MC_M2_S7_VC_delta_F \
//!     -o MC_M2_S1_VC_delta_D -o MC_M2_S7_VC_delta_D \
//!     -f m2s1-7_vc_gain.pkl
//! ```

use std::{fs::File, time::Instant};

use clap::Parser;
use faer::col::AsColRef;
use faer_ext::IntoFaer;
use gmt_dos_clients_fem::{fem_io, Model, Switch};
use gmt_fem::FEM;
use serde::Serialize;

#[derive(Parser, Debug)]
pub struct Cli {
    /// FEM inputs
    #[arg(short, long)]
    inputs: Vec<String>,
    /// FEM outputs
    #[arg(short, long)]
    outputs: Vec<String>,
    /// static gain filename
    #[arg(short, long)]
    filename: String,
}

fn main() -> anyhow::Result<()> {
    let args = Cli::parse();

    println!("loading the fem ...");
    let now = Instant::now();
    let mut fem = FEM::from_env()?;
    println!("elapsed: {:}s", now.elapsed().as_secs());

    println!(
        "extracting the static gain {:?} -> {:?}",
        args.inputs, args.outputs
    );
    fem.switch_inputs(Switch::Off, None)
        .switch_outputs(Switch::Off, None);
    let k = fem
        .switch_inputs_by_name(args.inputs.clone(), Switch::On)
        .and_then(|fem| fem.switch_outputs_by_name(args.outputs.clone(), Switch::On))
        .map(|fem| fem.reduced_static_gain().unwrap())?;
    let k = k.view_range(.., ..).into_faer();

    println!("extracting {:?} nodes", args.outputs);
    let xyz: Vec<_> = args
        .outputs
        .iter()
        .flat_map(|output| {
            let get_out = Box::<dyn fem_io::GetOut>::try_from(output.clone()).unwrap();
            let idx = get_out.position(&fem.outputs).unwrap();
            fem.outputs[idx]
                .as_ref()
                .map(|i| i.get_by(|i| i.properties.location.clone()))
                .unwrap()
        })
        .collect();
    let n = xyz.len();
    println!("extracting {:?} nodes", args.inputs);
    let in_xyz: Vec<_> = args
        .inputs
        .iter()
        .flat_map(|output| {
            let get_out = Box::<dyn fem_io::GetIn>::try_from(output.clone()).unwrap();
            let idx = get_out.position(&fem.inputs).unwrap();
            fem.inputs[idx]
                .as_ref()
                .map(|i| i.get_by(|i| i.properties.location.clone()))
                .unwrap()
        })
        .collect();
    let na = in_xyz.len();

    // surface nodes max radius
    let r_max = xyz
        .iter()
        .map(|xyz| xyz[0].hypot(xyz[1]))
        .max_by(|x, y| x.partial_cmp(y).unwrap())
        .unwrap();

    println!("computing the bending modes ...");
    let now = Instant::now();

    // Piston, tip & tilt
    let z123: Vec<_> = vec![1f64; n]
        .into_iter()
        .chain(xyz.iter().map(|xyz| xyz[0] / r_max))
        .chain(xyz.iter().map(|xyz| xyz[1] / r_max))
        .collect();

    // Filtering out piston, tip * tilt
    let svd_k = k.svd();
    let z_123 = faer::mat::from_column_major_slice::<f64>(&z123, n, 3);
    // fitting of bending modes
    let q_123 = z_123.svd().pseudoinverse() * svd_k.u().subcols(0, na);
    // removing piston, tip & tilt
    let up = svd_k.u().subcols(0, na) - z_123 * q_123;
    // normalizing
    let kp = up * svd_k.s_diagonal().column_vector_as_diagonal() * svd_k.v().transpose();
    let svd_k = kp.svd();

    // bending modes
    let u: Vec<f64> = svd_k
        .u()
        .subcols(0, na)
        .col_iter()
        .flat_map(|c| c.iter().cloned().collect::<Vec<f64>>())
        .collect();

    // modes to forces matrix
    let mut is = svd_k.s_diagonal().to_owned();
    is.as_mut().iter_mut().for_each(|x| *x = x.recip());
    let vis: Vec<_> = (svd_k.v() * is.as_col_ref().column_vector_as_diagonal())
        .col_iter()
        .flat_map(|c| c.iter().cloned().collect::<Vec<f64>>())
        .collect();

    let sms = SingularModes {
        mode_nodes: xyz,
        actuator_nodes: in_xyz,
        modes: u,
        mode_2_force: vis,
        shape: k.shape(),
    };
    println!("elapsed: {:}ms", now.elapsed().as_millis());
    serde_pickle::to_writer(&mut File::create(args.filename)?, &sms, Default::default())?;

    Ok(())
}

#[derive(Debug, Default, Clone, Serialize)]
pub struct SingularModes {
    mode_nodes: Vec<Vec<f64>>,
    actuator_nodes: Vec<Vec<f64>>,
    modes: Vec<f64>,
    mode_2_force: Vec<f64>,
    shape: (usize, usize),
}