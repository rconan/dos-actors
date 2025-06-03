use interface::UID;
use nalgebra::{DMatrix, DVector};
use std::{env, fs::DirBuilder, path::Path};

use gmt_dos_actors::prelude::*;
use gmt_dos_clients::{
    gain::Gain,
    signals::{OneSignal, Signal, Signals},
    smooth::{Smooth, Weight},
};
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_fem::{
    DiscreteModalSolver,
    fem_io::{
        actors_inputs::{CFD2021106F, MCM2Lcl6F, OSSM1Lcl6F},
        actors_outputs::{MCM2Lcl6D, OSSGIR6d, OSSM1Lcl, OSSPayloads6D},
    },
    solvers::ExponentialMatrix,
};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    mount::MountEncoders,
};
use gmt_dos_clients_mount::Mount;
use gmt_dos_clients_windloads::CfdLoads;
use gmt_fem::FEM;
use parse_monitors::cfd;

#[derive(UID)]
enum EncAvg {}

async fn task(
    cfd_path: &Path,
    sim_sampling_frequency: usize,
    sim_duration: usize,
    data_repo: &Path,
) -> anyhow::Result<()> {
    let n_step = sim_sampling_frequency * sim_duration;

    // GMT FEM
    let mut fem = FEM::from_env()?;
    fem.filter_outputs_by(&[26], |x| {
        x.descriptions
            .contains("Instrument at Direct Gregorian Port B (employed)")
    });

    // CFD WIND LOADS
    let cfd_loads_client = CfdLoads::foh(cfd_path.to_str().unwrap(), sim_sampling_frequency)
        .duration(sim_duration as f64)
        .windloads(&mut fem, Default::default())
        .build()?
        .into_arcx();

    // Model IO transformation Vectors
    // let gir_tooth_axfo = DVector::kronecker(
    //     &DVector::from_vec(vec![1., -1., 1., -1., 1., -1., 1., -1.]),
    //     &DVector::from_vec(vec![0., 0., 0.25, 0., 0., 0.]),
    // );
    // Discrete-time FEM STATE SPACE
    let state_space = {
        DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            //.max_eigen_frequency(75f64)
            .including_mount()
            .ins::<CFD2021106F>()
            // .ins_with::<OSSGIRTooth6F>(gir_tooth_axfo.as_view())
            .ins::<OSSM1Lcl6F>()
            .ins::<MCM2Lcl6F>()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .outs::<OSSGIR6d>()
            .outs::<OSSPayloads6D>()
            .use_static_gain_compensation()
            .build()?
    };

    // SET POINT
    let mut setpoint: Initiator<_> = Signals::new(3, n_step).into();
    // FEM
    let mut fem: Actor<_> = state_space.into();
    // MOUNT CONTROL
    let mount: Actor<_> = Mount::builder(&mut setpoint).build(&mut fem)?;
    // Logger
    let logging = Arrow::builder(n_step)
        .filename(data_repo.join("windloading").to_str().unwrap())
        .build()
        .into_arcx();
    let mut sink = Terminator::<_>::new(logging.clone());

    let avg_4ins = DVector::from_vec(vec![1., 1., 1., 1.])
        .unscale(4.0)
        .transpose();
    let avg_6ins = DVector::from_vec(vec![1., 1., 1., 1., 1., 1.])
        .unscale(6.0)
        .transpose();
    let mut mnt_avg_gain = DMatrix::<f64>::zeros(3, 14);
    mnt_avg_gain
        .fixed_view_mut::<1, 4>(0, 0)
        .copy_from(&avg_4ins);
    mnt_avg_gain
        .fixed_view_mut::<1, 6>(1, 4)
        .copy_from(&avg_6ins);
    mnt_avg_gain
        .fixed_view_mut::<1, 4>(2, 10)
        .copy_from(&avg_4ins);
    let mut mnt_enc_avg_map: Actor<_> = (Gain::new(mnt_avg_gain), "Mount ENC Avg").into();
    // CFD WL actor
    let mut cfd_loads: Initiator<_> = Actor::new(cfd_loads_client.clone()).name("CFD Wind loads");
    let signals = Signals::new(1, n_step).channel(
        0,
        Signal::Sigmoid {
            amplitude: 1f64,
            sampling_frequency_hz: sim_sampling_frequency as f64,
        },
    );
    let signal = OneSignal::try_from(signals)?.into_arcx();
    let m1_smoother = Smooth::new().into_arcx();
    let m2_smoother = Smooth::new().into_arcx();
    let mount_smoother = Smooth::new().into_arcx();

    let mut sigmoid: Initiator<_> = Actor::new(signal.clone()).name("Sigmoid");
    let mut smooth_m1_loads: Actor<_> = Actor::new(m1_smoother.clone());
    let mut smooth_m2_loads: Actor<_> = Actor::new(m2_smoother.clone());
    let mut smooth_mount_loads: Actor<_> = Actor::new(mount_smoother.clone());

    sigmoid
        .add_output()
        .multiplex(3)
        .build::<Weight>()
        .into_input(&mut smooth_m1_loads)
        .into_input(&mut smooth_m2_loads)
        .into_input(&mut smooth_mount_loads)?;
    cfd_loads
        .add_output()
        .build::<CFDM1WindLoads>()
        .into_input(&mut smooth_m1_loads)?;
    smooth_m1_loads
        .add_output()
        .build::<CFDM1WindLoads>()
        .into_input(&mut fem)?;
    cfd_loads
        .add_output()
        .build::<CFDM2WindLoads>()
        .into_input(&mut smooth_m2_loads)?;
    smooth_m2_loads
        .add_output()
        .build::<CFDM2WindLoads>()
        .into_input(&mut fem)?;
    cfd_loads
        .add_output()
        .build::<CFDMountWindLoads>()
        .into_input(&mut smooth_mount_loads)?;
    smooth_mount_loads
        .add_output()
        .build::<CFDMountWindLoads>()
        .into_input(&mut fem)?;

    // GIR tooth contact axial force (DRV to plant model connection)
    // mount.add_output()
    //     .build::<OSSGIRTooth6F>()
    //     .into_input(&mut fem)?;
    // Connection of Mount ENC positions to the averaging computation block
    fem.add_output()
        .bootstrap()
        .build::<MountEncoders>()
        .into_input(&mut mnt_enc_avg_map)?;
    // LOG outputs
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
    fem.add_output()
        .unbounded()
        .build::<OSSGIR6d>()
        .logn(&mut sink, 6)
        .await?;
    fem.add_output()
        .unbounded()
        .build::<OSSPayloads6D>()
        .logn(&mut sink, 6)
        .await?;
    mnt_enc_avg_map
        .add_output()
        .unbounded()
        .build::<EncAvg>()
        .logn(&mut sink, 3)
        .await?;

    model!(
        setpoint,
        mount,
        mnt_enc_avg_map,
        cfd_loads,
        sigmoid,
        smooth_m1_loads,
        smooth_m2_loads,
        smooth_mount_loads,
        fem,
        sink
    )
    .name("mountloading")
    .quiet()
    .check()?
    // .flowchart()
    .run()
    .wait()
    .await?;

    Ok(())
}

use indicatif::ProgressIterator;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sim_sampling_frequency = 1000;
    let sim_duration = 400_usize; // second

    let fem_repo = env::var("FEM_REPO").expect("FEM_REPO env var missing");
    let fem_model = format!(
        "{:}_mount-data",
        Path::new(&fem_repo)
            .components()
            .last()
            .and_then(|path| path.as_os_str().to_str())
            .unwrap()
    );
    let cfd_repo = env::var("CFD_REPO").expect("CFD_REPO env var missing");

    let mut handles = vec![];

    for cfd_case in cfd::Baseline::<2021>::default().into_iter() {
        let cfd_path = Path::new(&cfd_repo).join(cfd_case.to_string());
        let data_repo = cfd_path.join(&fem_model);
        if !data_repo.is_dir() {
            DirBuilder::new().recursive(true).create(&data_repo)?;
        }
        let h: tokio::task::JoinHandle<Result<(), anyhow::Error>> = tokio::spawn(async move {
            task(&cfd_path, sim_sampling_frequency, sim_duration, &data_repo).await?;
            Ok(())
        });
        handles.push(h);
    }

    for handle in handles.into_iter().progress() {
        let _ = handle.await?;
    }

    Ok(())
}
