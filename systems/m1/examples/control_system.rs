use std::env;
use std::path::Path;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_fem::{
    DiscreteModalSolver, DiscreteStateSpace,
    solvers::{CuStateSpace, ExponentialMatrix},
};
use gmt_dos_clients_io::gmt_m1::assembly::{
    M1ActuatorAppliedForces, M1HardpointsForces, M1HardpointsMotion,
};
use gmt_dos_systems_m1::M1;
use gmt_fem::FEM;
use interface::Tick;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    unsafe {
        env::set_var(
            "DATA_REPO",
            Path::new(env!("CARGO_MANIFEST_DIR")).join("examples"),
        );
        env::set_var("FLOWCHART", "dot");
    }

    let sim_sampling_frequency = 1000;
    let sim_duration = 10_usize; // second
    let mut fem = FEM::from_env()?;

    let m1 = M1::<10>::builder(&mut fem).build()?;

    let plant: DiscreteModalSolver<CuStateSpace> =
        DiscreteStateSpace::<ExponentialMatrix>::from(fem)
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            // .including_mount()
            .including_m1(None)?
            // .outs::<OSSM1Lcl>()
            .use_static_gain_compensation()
            .build()?;

    let timer: Timer = Timer::new(sim_duration * sim_sampling_frequency);

    #[cfg(m1_hp_force_extension)]
    actorscript!(
        1: timer[Tick] -> plant
        1: {m1}[M1HardpointsMotion] -> plant
        1: {m1}[M1ActuatorAppliedForces] -> plant
        1: plant[M1HardpointsForces]! -> {m1}
    );

    #[cfg(not(m1_hp_force_extension))]
    actorscript!(
        1: timer[Tick] -> plant
        1: {m1}[M1HardpointsForces] -> plant
        1: {m1}[M1ActuatorAppliedForces] -> plant
        1: plant[M1HardpointsMotion]! -> {m1}
    );

    Ok(())
}
