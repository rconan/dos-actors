use gmt_dos_actors::{actorscript, prelude::*};
use gmt_dos_clients::{Signal, Signals};
use gmt_dos_clients_fem::{fem_io::actors_outputs::*, DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    mount::{MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use gmt_lom::{OpticalMetrics, LOM};
use skyangle::Conversion;

// Move the mount 1arcsec along the elevation axis of the telescope
// DATA:
//  * FEM 2nd order model: FEM_REPO
//  * linear optical sensitivity matrices: LOM

// cargo test --release --package gmt_dos-clients_mount --test setpoint_mount --features mount-fdr -- setpoint_mount --exact --nocapture
#[tokio::test]
async fn setpoint_mount() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 1000;
    let sim_duration = 4_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    // FEM MODEL
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

    // SET POINT
    let setpoint = Signals::new(3, n_step).channel(1, Signal::Constant(1f64.from_arcsec()));
    // FEM
    let fem = state_space;
    // MOUNT CONTROL
    let mount = Mount::new();
    // let mount = Mount::builder(&mut setpoint).build(&mut fem)?;
    // Logger
    // let logging = Arrow::builder(n_step).build();

    actorscript! {
        #[labels(fem = "FEM", mount = "Mount\nControl")]
        1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount
        1: fem[M1RigidBodyMotions]$
        1: fem[M2RigidBodyMotions]$
    }

    // Linear optical sensitivities to derive segment tip and tilt
    let lom = LOM::builder()
        .rigid_body_motions_record(
            (*logging_1.lock().await).record()?,
            Some("M1RigidBodyMotions"),
            Some("M2RigidBodyMotions"),
        )?
        .build()?;
    let segment_tiptilt = lom.segment_tiptilt();
    let stt = segment_tiptilt.items().last().unwrap();

    println!("Segment TT: {:.3?}mas", stt.to_mas());
    //assert!(tt[0].hypot(tt[1]) < 0.25);

    Ok(())
}