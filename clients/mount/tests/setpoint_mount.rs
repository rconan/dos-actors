use gmt_dos_actors::prelude::*;
use gmt_dos_clients::{Signal, Signals};
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions};
use gmt_dos_clients_mount::Mount;
use gmt_fem::fem_io::actors_outputs::{MCM2Lcl6D, OSSM1Lcl};
use gmt_fem::FEM;
use lom::{OpticalMetrics, LOM};
use skyangle::Conversion;

// Move the mount 1arcsec along the elevation axis of the telescope
// DATA:
//  * FEM 2nd order model: FEM_REPO
//  * linear optical sensitivity matrices: LOM

#[tokio::test]
async fn setpoint_mount() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 8000;
    let sim_duration = 1_usize; // second
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
    let mut setpoint: Initiator<_> = Signals::new(3, n_step)
        .channel(1, Signal::Constant(1f64.from_arcsec()))
        .into();
    // FEM
    let mut fem: Actor<_> = state_space.into();
    // MOUNT CONTROL
    // let mut mount: Actor<_> = Mount::new().into();
    let mount: Actor<_> = Mount::builder(&mut setpoint).build(&mut fem)?;
    // Logger
    let logging = Arrow::builder(n_step).build().into_arcx();
    let mut sink = Terminator::<_>::new(logging.clone());

    /*     source
        .add_output()
        .build::<MountSetPoint>()
        .into_input(&mut mount)?;
    mount
        .add_output()
        .build::<MountTorques>()
        .into_input(&mut fem)?;
    fem.add_output()
        .bootstrap()
        .multiplex(2)
        .build::<MountEncoders>()
        .into_input(&mut mount)
        .logn(&mut sink, 14)
        .await?; */
    fem.add_output()
        .unbounded()
        .build::<M1RigidBodyMotions>()
        .log(&mut sink)
        .await?;
    fem.add_output()
        .unbounded()
        .build::<M2RigidBodyMotions>()
        .log(&mut sink)
        .await?;

    model!(setpoint, mount, fem, sink)
        .check()?
        .flowchart()
        .run()
        .wait()
        .await?;

    // Linear optical sensitivities to derive segment tip and tilt
    let lom = LOM::builder()
        .rigid_body_motions_record(
            (*logging.lock().await).record()?,
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
