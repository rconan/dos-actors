use std::env;
use std::path::Path;

use gmt_dos_actors::actorscript;
use gmt_dos_actors::model::{Model, Running};
use gmt_dos_clients::signals::{Signal, Signals};
use gmt_dos_clients_fem::fem_io::actors_outputs::OSSM1Lcl;
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::gmt_m1::segment::{
    ActuatorAppliedForces, ActuatorCommandForces, BarycentricForce, HardpointsForces,
    HardpointsMotion, RBM,
};
use gmt_dos_clients_m1_ctrl::{Actuators, Calibration, Hardpoints, LoadCells};
use gmt_fem::FEM;
use interface::Size;

const S1: u8 = 1;

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

    println!("M1 S1 as-designed model");
    let explicit_loadcells_model =
        explicit_loadcells(sim_sampling_frequency, sim_duration, whole_fem.clone()).await?;

    println!("M1 S1 as-built model");
    let implicit_loadcells_model =
        implicit_loadcells(sim_sampling_frequency, sim_duration, whole_fem.clone()).await?;

    explicit_loadcells_model.await?;
    implicit_loadcells_model.await?;

    Ok(())
}
async fn explicit_loadcells(
    sim_sampling_frequency: usize,
    sim_duration: usize,
    mut whole_fem: FEM,
) -> anyhow::Result<Model<Running>> {
    let n_step = sim_sampling_frequency * sim_duration;
    let m1_calibration = Calibration::new(&mut whole_fem);
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(whole_fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_m1(Some(vec![1]))?
        .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;

    let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let hp_setpoint = (0..6).fold(Signals::new(6, n_step), |signals, i| {
        signals.channel(
            i,
            Signal::Sigmoid {
                amplitude: rbm_fun(i) * 1e-6,
                sampling_frequency_hz: sim_sampling_frequency as f64,
            },
        )
    });
    // Hardpoints
    let hardpoints = Hardpoints::new(
        m1_calibration.stiffness,
        m1_calibration.rbm_2_hp[S1 as usize - 1],
    );
    // Loadcells
    let loadcell = LoadCells::builder()
        .hardpoints_barycentric_transform(m1_calibration.lc_2_cg[S1 as usize - 1])
        .hardpoints_stiffness(m1_calibration.stiffness)
        .build();
    // Actuators
    let actuators = Actuators::<S1>::new();
    let actuators_setpoint = Signals::new(
        Size::<ActuatorCommandForces<S1>>::len(&Actuators::<S1>::new()),
        n_step,
    );

    actorscript! {
        #[model(name=rust_loadcells, state=running)]
        #[labels(
            plant = "Plant",
            hp_setpoint = "RBM",
            actuators_setpoint = "Actuators\nSetpoint")]
        1: hp_setpoint[RBM<S1>]
            -> hardpoints[HardpointsForces<S1>]
                -> loadcell
        1: hardpoints[HardpointsForces<S1>]
            -> plant[RBM<S1>]!$
        1: actuators[ActuatorAppliedForces<S1>]
            -> plant[HardpointsMotion<S1>]!
                -> loadcell
        10: actuators_setpoint[ActuatorCommandForces<S1>] -> actuators
        10: loadcell[BarycentricForce<S1>]! -> actuators
    }
    Ok(model)
}
async fn implicit_loadcells(
    sim_sampling_frequency: usize,
    sim_duration: usize,
    mut whole_fem: FEM,
) -> anyhow::Result<Model<Running>> {
    let n_step = sim_sampling_frequency * sim_duration;
    let m1_calibration = Calibration::new(&mut whole_fem);
    let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(whole_fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_m1(Some(vec![1]))?
        .outs::<OSSM1Lcl>()
        .use_static_gain_compensation()
        .build()?;

    let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let hp_setpoint = (0..6).fold(Signals::new(6, n_step), |signals, i| {
        signals.channel(
            i,
            Signal::Sigmoid {
                amplitude: rbm_fun(i) * 1e-6,
                sampling_frequency_hz: sim_sampling_frequency as f64,
            },
        )
    });
    // Hardpoints
    let hardpoints = Hardpoints::new(
        m1_calibration.stiffness,
        m1_calibration.rbm_2_hp[S1 as usize - 1],
    );
    // Loadcells
    let loadcell = LoadCells::builder()
        .hardpoints_barycentric_transform(m1_calibration.lc_2_cg[S1 as usize - 1])
        .build();
    // Actuators
    let actuators = Actuators::<S1>::new();
    let actuators_setpoint =
        Signals::new(Size::<ActuatorCommandForces<S1>>::len(&actuators), n_step);
    actorscript! {
        #[model(name=fem_loadcells, state=running)]
        #[labels(
            plant = "Plant",
            hp_setpoint = "RBM",
            actuators_setpoint = "Actuators\nSetpoint")]
        1: hp_setpoint[RBM<S1>]
            -> hardpoints[HardpointsMotion<S1>]
            -> plant[RBM<S1>]!$
        1: actuators[ActuatorAppliedForces<S1>]
            -> plant[HardpointsForces<S1>]!
                -> loadcell
        10: actuators_setpoint[ActuatorCommandForces<S1>] -> actuators
        10: loadcell[BarycentricForce<S1>] -> actuators
    }
    Ok(model)
}
