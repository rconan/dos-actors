//! FEM STATIC GAIN
//!
//! Computes the FEM static gain between given inputs and outputs.
//!
//! M2 S7 gain matrix for the voice coils:
//! ```shell
//! cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap toml" -- \
//!     -i MC_M2_S7_VC_delta_F -o MC_M2_S7_VC_delta_D \
//!     -f m2s7_vc_gain.pkl
//! ```
//!
//! M2 S1 & S7 gain matrix for the voice coils:
//! ```shell
//! cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap toml" -- \
//!     -i MC_M2_S1_VC_delta_F -i MC_M2_S7_VC_delta_F \
//!     -o MC_M2_S1_VC_delta_D -o MC_M2_S7_VC_delta_D \
//!     -f m2s1-7_vc_gain.pkl
//! ```
//!
//! Inputs and ouputs can insteadn be read from a config file name `gain_io.toml`
//! ```toml
//! inputs = ["M1_actuators_segment_1"]
//! outputs = ["M1_segment_1_axial_d"]
//! filename = "gain.pkl"
//! ```
//! ```shell
//! cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap toml" //!
//! ```

use std::{
    fs::{exists, File},
    io::Read,
};

use clap::Parser;
use gmt_dos_clients_fem::{Model, Switch};
use gmt_fem::FEM;
use serde::{Deserialize, Serialize};

#[derive(Parser, Debug, Serialize, Deserialize)]
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
    let args: Cli = if exists("gain_io.toml")? {
        let mut file = File::open("gain_io.toml")?;
        let mut buffer = String::new();
        file.read_to_string(&mut buffer)?;
        toml::from_str(&buffer)?
    } else {
        Cli::parse()
    };
    let mut fem = FEM::from_env()?;

    fem.switch_inputs(Switch::Off, None)
        .switch_outputs(Switch::Off, None);
    let k = fem
        .switch_inputs_by_name(args.inputs, Switch::On)
        .and_then(|fem| fem.switch_outputs_by_name(args.outputs, Switch::On))
        .map(|fem| fem.reduced_static_gain().unwrap())?;
    serde_pickle::to_writer(&mut File::create(args.filename)?, &k, Default::default())?;

    Ok(())
}
