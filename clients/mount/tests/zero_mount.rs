//! Mount controller null test
//!
//! Run the mount controller with the mount torques and encoders of the FEM model
//! and with the mount control set points set to 0
//! The FEM model repository is read from the `FEM_REPO` environment variable
//! The LOM sensitivity matrices are located in the directory given by the `LOM` environment variable

use gmt_dos_actors::prelude::*;
use gmt_dos_clients::Signals;
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions};
use gmt_dos_clients_mount::Mount;
use gmt_fem::fem_io::actors_outputs::{MCM2Lcl6D, OSSM1Lcl};
use gmt_fem::FEM;
use lom::{Stats, LOM};

#[tokio::test]
async fn zero_mount_at() -> anyhow::Result<()> {
    let sim_sampling_frequency = 8000;
    let sim_duration = 4_usize;
    let n_step = sim_sampling_frequency * sim_duration;

    let state_space = {
        let fem = FEM::from_env()?;
        DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            .including_mount()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .use_static_gain_compensation()
            .build()?
    };

    // SET POINT
    let mut setpoint: Initiator<_> = Signals::new(3, n_step).into();
    // FEM
    let mut fem: Actor<_> = state_space.into();
    // MOUNT
    let mount: Actor<_> = Mount::builder(&mut setpoint).build(&mut fem)?;
    
    let logging = Arrow::builder(n_step).build().into_arcx();
    let mut sink = Terminator::<_>::new(logging.clone());

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

    let lom = LOM::builder()
        .rigid_body_motions_record(
            (*logging.lock().await).record()?,
            Some("M1RigidBodyMotions"),
            Some("M2RigidBodyMotions"),
        )?
        .build()?;
    let tiptilt = lom.tiptilt_mas();
    let n_sample = 1000;
    let tt = tiptilt.std(Some(n_sample));
    println!("TT STD.: {:.3?}mas", tt);

    assert!(tt[0].hypot(tt[1]) < 0.25);

    Ok(())
}
