/*
This demo simulates the behavior of the telescope when subject to wind loads. A feedback controller stabilizes the moun axes: azimuth, elevation, and GIR.
Some of the crates::structures used in the implementation are
- gmt_fem::FEM <--> The structural dynamics model of the GMT
- gmt_dos_clients_fem::DiscreteModalSolver <--> discretizes the structural model
- gmt_dos_clients_mount::Mount <--> The mount feedback controller
- gmt_dos_clients_windloads::CfdLoads <--> time-series wind load forces and torques
-
Four scopes provide wavefront error (WFE) contributions from different sources:
- M1 RBM & M2 SHELL rigid-body motions (RBM);
- M1 RBM;
- M2 SHELL RBM;
- M2 ASM reference body.
Those WFE contributions are the RMS values computed through the GMT linear optical model (gmt_dos_clients_lom::LinearOpticalModel).
*/

use std::env;
use std::path::Path;

use demos::{AsmRefBodyWfeRms, AsmShellWfeRms, M1RbmWfeRms};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    signals::{OneSignal, Signal, Signals},
    smooth::{Smooth, Weight},
};
use gmt_dos_clients_fem::{solvers::ExponentialMatrix, DiscreteModalSolver};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_fem::{
        inputs::{MCM2Lcl6F, OSSM1Lcl6F, CFD2021106F},
        outputs::{MCM2Lcl6D, OSSM1Lcl, MCM2RB6D},
    },
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{asm::M2ASMReferenceBodyNodes, M2RigidBodyMotions},
    mount::{MountEncoders, MountSetPoint, MountTorques},
    optics::WfeRms,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_mount::Mount;
use gmt_dos_clients_windloads::CfdLoads;
use gmt_fem::FEM;

/*
MOUNT_MODEL=MOUNT_PDR_8kHz FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release --bin windloaded-mount
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bin")
            .join("windloaded-mount"),
    );

    let sim_sampling_frequency = 8000;
    let sim_duration = 5_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = Option::<FEM>::None;

    let cfd_loads = CfdLoads::foh(".", sim_sampling_frequency)
        .duration(sim_duration as f64)
        .windloads(fem.get_or_insert(FEM::from_env()?), Default::default())
        .build()?;

    // FEM MODEL
    let state_space =
        DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem.unwrap_or(FEM::from_env()?))
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            //.max_eigen_frequency(75f64)
            .including_mount()
            .ins::<CFD2021106F>()
            .ins::<OSSM1Lcl6F>()
            .ins::<MCM2Lcl6F>()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .outs::<MCM2RB6D>()
            .use_static_gain_compensation()
            .build()?;
    println!("{state_space}");

    // SET POINT
    let setpoint = Signals::new(3, n_step); //.channel(1, Signal::Constant(1f64.from_arcsec()));

    // FEM
    let fem = state_space;
    // MOUNT CONTROL
    let mount = Mount::new();
    // LOM
    let lom = LinearOpticalModel::new()?;
    let m1_lom = LinearOpticalModel::new()?;
    let asm_shell_lom = LinearOpticalModel::new()?;
    let asm_rb_lom = LinearOpticalModel::new()?;

    let sigmoid = OneSignal::try_from(Signals::new(1, n_step).channel(
        0,
        Signal::Sigmoid {
            amplitude: 1f64,
            sampling_frequency_hz: sim_sampling_frequency as f64,
        },
    ))?;

    let m1_smoother = Smooth::new();
    let m2_smoother = Smooth::new();
    let mount_smoother = Smooth::new();

    actorscript! {
    #[labels(fem = "GMT FEM", setpoint="Mount SP", mount = "Mount\nControl", lom="Linear Optical\nModel")]
    1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount

    1: cfd_loads[CFDM1WindLoads] -> m1_smoother
    1: sigmoid[Weight] -> m1_smoother[CFDM1WindLoads] -> fem

    1: cfd_loads[CFDM2WindLoads] -> m2_smoother
    1: sigmoid[Weight] -> m2_smoother[CFDM2WindLoads] -> fem

    1: cfd_loads[CFDMountWindLoads] -> mount_smoother
    1: sigmoid[Weight] -> mount_smoother[CFDMountWindLoads] -> fem

    8: lom[WfeRms<-6>]~
    1: fem[M1RigidBodyMotions] -> lom
    1: fem[M2RigidBodyMotions] -> lom

    8: m1_lom[M1RbmWfeRms]~
    1: fem[M1RigidBodyMotions] -> m1_lom

    8: asm_shell_lom[AsmShellWfeRms]~
    1: fem[M2RigidBodyMotions] -> asm_shell_lom

    8: asm_rb_lom[AsmRefBodyWfeRms]~
    1: fem[M2ASMReferenceBodyNodes] -> asm_rb_lom
    }

    Ok(())
}
