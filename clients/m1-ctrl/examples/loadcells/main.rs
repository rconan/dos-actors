use std::env;
use std::path::Path;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::signals::Signals;
use gmt_dos_clients_fem::{
    DiscreteModalSolver, fem_io::actors_outputs::OSSM1Lcl, solvers::ExponentialMatrix,
};
use gmt_dos_clients_io::{
    gmt_m1::segment::{
        ActuatorAppliedForces, ActuatorCommandForces, BarycentricForce, HardpointsForces,
        HardpointsMotion, RBM,
    },
    mount::{AverageMountEncoders, MountEncoders, MountTorques},
};
use gmt_dos_clients_m1_ctrl::{Actuators, Calibration, Hardpoints, LoadCells};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
use interface::Size;

const SX: u8 = 1;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    unsafe {
        env::set_var(
            "DATA_REPO",
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("examples")
                .join("loadcells"),
        );
        env::set_var("FLOWCHART", "dot");
    }

    let sim_sampling_frequency = 1000;
    let sim_duration = 10_usize; // second
    let whole_fem = FEM::from_env()?;

    #[cfg(any(not(m1_hp_force_extension), feature = "explicit-loadcells"))]
    {
        println!("M1 S{SX} as-designed model");
        explicit_loadcells(sim_sampling_frequency, sim_duration, whole_fem.clone()).await?;
    }
    #[cfg(all(m1_hp_force_extension, not(feature = "explicit-loadcells")))]
    {
        println!("M1 S{SX} as-built model");
        implicit_loadcells(sim_sampling_frequency, sim_duration, whole_fem.clone()).await?;
    }
    Ok(())
}
#[cfg(any(not(m1_hp_force_extension), feature = "explicit-loadcells"))]
async fn explicit_loadcells(
    sim_sampling_frequency: usize,
    sim_duration: usize,
    mut whole_fem: FEM,
) -> anyhow::Result<()> {
    let n_step = sim_sampling_frequency * sim_duration;
    let m1_calibration = Calibration::new(&mut whole_fem);
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(whole_fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .including_m1(Some(vec![SX]))?
        .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;

    let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let hp_setpoint = (0..6).fold(Signals::new(6, n_step), |signals, i| {
        signals.channel(
            i,
            rbm_fun(i) * 1e-6, // Signal::Sigmoid {
                               //     amplitude: rbm_fun(i) * 1e-6,
                               //     sampling_frequency_hz: sim_sampling_frequency as f64,
                               // },
        )
    });

    let mount = Mount::new();

    // Hardpoints
    let hardpoints = Hardpoints::<SX>::new(
        m1_calibration.stiffness,
        m1_calibration.rbm_2_hp[SX as usize - 1],
    );
    // Loadcells
    let loadcell = LoadCells::<SX>::builder()
        .hardpoints_barycentric_transform(m1_calibration.lc_2_cg[SX as usize - 1])
        .hardpoints_stiffness(m1_calibration.stiffness)
        .build();
    // Actuators
    let actuators = Actuators::<SX>::new();
    let actuators_setpoint = Signals::new(
        Size::<ActuatorCommandForces<SX>>::len(&Actuators::<SX>::new()),
        n_step,
    );

    actorscript! {
        #[model(name=rust_loadcells)]
        #[labels(
            plant = "Plant",
            hp_setpoint = "RBM",
            actuators_setpoint = "Actuators\nSetpoint")]
        1: mount[MountTorques] -> plant[MountEncoders]! -> mount
        1: hp_setpoint[RBM<SX>]
            -> hardpoints[HardpointsForces<SX>]
                -> loadcell
        1: hardpoints[HardpointsForces<SX>]
            -> plant[RBM<SX>]!$
        1: actuators[ActuatorAppliedForces<SX>]
            -> plant[HardpointsMotion<SX>]!
                -> loadcell
        10: actuators_setpoint[ActuatorCommandForces<SX>] -> actuators
        10: loadcell[BarycentricForce<SX>]! -> actuators
    }
    Ok(())
}
#[cfg(all(m1_hp_force_extension, not(feature = "explicit-loadcells")))]
async fn implicit_loadcells(
    sim_sampling_frequency: usize,
    sim_duration: usize,
    mut whole_fem: FEM,
) -> anyhow::Result<()> {
    let n_step = sim_sampling_frequency * sim_duration;

    let m1_calibration = Calibration::new(&mut whole_fem);
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(whole_fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .including_m1(Some(vec![SX]))?
        .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;

    let mount = Mount::new();

    let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let hp_setpoint = (0..6).fold(Signals::new(6, n_step), |signals, i| {
        signals.channel(
            i,
            rbm_fun(i) * 1e-6, // Signal::Sigmoid {
                               // Signal::Sigmoid {
                               //     amplitude: rbm_fun(i) * 1e-6,
                               //     sampling_frequency_hz: sim_sampling_frequency as f64,
                               // },
        )
    });
    // Hardpoints
    let hardpoints = Hardpoints::<SX>::new(
        m1_calibration.stiffness,
        m1_calibration.rbm_2_hp[SX as usize - 1],
    );
    // Loadcells
    let loadcell = LoadCells::<SX>::builder()
        .hardpoints_barycentric_transform(m1_calibration.lc_2_cg[SX as usize - 1])
        .build();
    // Actuators
    let actuators = Actuators::<SX>::new();
    let actuators_setpoint =
        Signals::new(Size::<ActuatorCommandForces<SX>>::len(&actuators), n_step);
    actorscript! {
        #[model(name=fem_loadcells)]
        #[labels(
            plant = "Plant",
            hp_setpoint = "RBM",
            actuators_setpoint = "Actuators\nSetpoint")]
        1: mount[MountTorques] -> plant[MountEncoders]! -> mount
        1: actuators[ActuatorAppliedForces<SX>]
            -> plant[HardpointsForces<SX>]!
                -> loadcell
        1: hp_setpoint[RBM<SX>]
            -> hardpoints[HardpointsMotion<SX>] -> plant
        1: plant[RBM<SX>]!$
        1: plant[AverageMountEncoders]!$
        10: actuators_setpoint[ActuatorCommandForces<SX>] -> actuators
        10: loadcell[BarycentricForce<SX>] -> actuators
    }
    Ok(())
}
