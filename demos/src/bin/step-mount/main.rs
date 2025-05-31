use std::{env, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::signals::{Signal, Signals};
use gmt_dos_clients_fem::{
    DiscreteModalSolver, fem_io::actors_outputs::*, solvers::ExponentialMatrix,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    mount::{MountEncoders, MountSetPoint, MountTorques},
    optics::TipTilt,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use interface::units::Arcsec;
use skyangle::Conversion;

// Move the mount 1arcsec along the elevation axis of the telescope
// DATA:
//  * FEM 2nd order model: FEM_REPO
//  * linear optical sensitivity matrices: LOM

/*
MOUNT_MODEL=MOUNT_PDR_8kHz FEM_REPO=`pwd`/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/ cargo run --release --bin step-mount
*/

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    unsafe {
        env::set_var(
            "DATA_REPO",
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("src")
                .join("bin")
                .join("step-mount"),
        );
    }
    // simulation sampling frequency
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency(); // Hz

    // simulation duration
    let sim_duration = 4_usize; // second

    // simulation discrete time steps #
    let n_step = sim_sampling_frequency * sim_duration;

    // FEM MODEL
    //  * model identification: 20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111
    //     (s3://gmto.im.grim/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/modal_state_space_model_2ndOrder.zip)
    //  * 2% damping coefficient
    //  * static gain compensation
    //  * inputs/outputs:
    //      * mount: MountTorques & MountEncoders
    //      * M1: M1RigidBodyMotions (OSSM1Lcl)
    //      * M2: M2RigidBodyMotions (MCM2Lcl6D)
    let state_space = {
        let fem = FEM::from_env()?;
        println!("{fem}");
        DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            //.max_eigen_frequency(75f64)
            .including_mount()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .use_static_gain_compensation()
            .build()?
    };
    println!("{state_space}");

    // [SET POINT](crates.io/crates/gmt_dos-clients)
    // command signal for the 3 axes of the mount:
    //   * azimuth (channel #0), cmd = 0
    //   * elevation (channel #1), cmd = 1arcsec
    //   * GIR (channel #2), cmd = 0
    // * output: MoutSetPoint
    let setpoint = Signals::new(3, n_step).channel(1, Signal::Constant(1f64.from_arcsec()));
    // [FEM](crates.io/crates/gmt_dos-clients_fem)
    // * inputs: MountTorques
    // * outputs: M1RigidBodyMotions, M2RigidBodyMotions
    let fem = state_space;
    // [MOUNT CONTROL](crates.io/crates/gmt_dos-clients_mount)
    // input: MountSetPoint, MountEnCoders
    // outputs: MountTorques
    let mount = Mount::new();
    // [LOM (Linear Optical Model)](crates.io/crates/gmt_dos-clients_lom)
    // * inputs: M1RigidBodyMotions, M2RigidBodyMotions
    // * output: Arcsec<TipTilt>
    let lom = LinearOpticalModel::new()?;

    actorscript! {
        #[labels(fem = "GMT Structural Model", mount = "Mount\nControl", lom="Linear Optical\nModel")]
        1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount
        1: fem[M1RigidBodyMotions] -> lom
        1: fem[M2RigidBodyMotions] -> lom[Arcsec<TipTilt>]~
    }

    Ok(())
}
