use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::gmt_fem::outputs::{MCM2Lcl6D, OSSM1Lcl};

fn main() -> anyhow::Result<()> {
    let sim_sampling_frequency: usize = gmt_dos_clients_mount::sampling_frequency();

    let fem = DiscreteModalSolver::<ExponentialMatrix>::from_env()?
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .use_static_gain_compensation()
        .build()?;
    fem.save("gmt-fem.bin")?;

    Ok(())
}
