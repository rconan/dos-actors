use criterion::*;
use gmt_dos_clients_fem::{DiscreteModalSolver, DiscreteStateSpace, solvers};
use gmt_dos_clients_io::gmt_fem::{
    inputs::{MCM2PZTF, MCM2SmHexF, OSSM1Lcl6F},
    outputs::{MCM2Lcl6D, MCM2PZTD, MCM2SmHexD, OSSM1Lcl},
};
use gmt_fem::FEM;
use interface::Update;

#[cfg(not(feature = "cuda"))]
type Solver = solvers::Exponential;
#[cfg(feature = "cuda")]
type Solver = solvers::CuStateSpace;

#[inline]
fn update(fem: &mut DiscreteModalSolver<Solver>) {
    fem.update();
}

pub fn fem(c: &mut Criterion) {
    let mut fem_ss: DiscreteModalSolver<Solver> =
        DiscreteStateSpace::<solvers::Exponential>::from(FEM::from_env().unwrap())
            .sampling(1000_f64)
            .proportional_damping(2. / 100.)
            .use_static_gain_compensation()
            .ins::<OSSM1Lcl6F>()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .build()
            .unwrap();
    c.bench_function("fem", |b| b.iter(|| update(&mut fem_ss)));
}

pub fn servos_fem(c: &mut Criterion) {
    let mut fem_ss: DiscreteModalSolver<Solver> =
        DiscreteStateSpace::<solvers::Exponential>::from(FEM::from_env().unwrap())
            .sampling(1000_f64)
            .proportional_damping(2. / 100.)
            .use_static_gain_compensation()
            .including_mount()
            .including_m1(Some(vec![1, 2, 3, 4, 5, 6, 7]))
            .unwrap()
            .ins::<MCM2PZTF>()
            .ins::<MCM2SmHexF>()
            .outs::<MCM2PZTD>()
            .outs::<MCM2SmHexD>()
            .outs::<MCM2Lcl6D>()
            .outs::<OSSM1Lcl>()
            .build()
            .unwrap();
    c.bench_function("servos. fem", |b| b.iter(|| update(&mut fem_ss)));
}

criterion_group!(benches, fem, servos_fem);
criterion_main!(benches);
