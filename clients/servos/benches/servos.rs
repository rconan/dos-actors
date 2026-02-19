use criterion::*;
use gmt_dos_actors::{
    actor::Actor,
    model::Model,
    prelude::{AddActorOutput, AddOuput, TryIntoInputs, vec_box},
    system::Sys,
};
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers};
use gmt_dos_clients_servos::{GmtServoMechanisms, M1SegmentFigure};
use gmt_fem::FEM;
use interface::Tick;

#[cfg(not(feature = "cuda"))]
type Solver = solvers::ExponentialMatrix;
#[cfg(feature = "cuda")]
type Solver = solvers::CuStateSpace;

// #[inline]
async fn update(servos_prime: &Sys<GmtServoMechanisms<10>>) {
    let mut servos = Sys::<GmtServoMechanisms<10>>::clone(servos_prime);
    let mut timer: Actor<Timer, 0, 1> = Timer::new(100).into();
    timer
        .add_output()
        .build::<Tick>()
        .into_input::<DiscreteModalSolver<Solver>>(&mut servos)
        .unwrap();
    Model::new(vec_box!(timer, servos))
        .quiet()
        .check()
        .unwrap()
        .run()
        .await
        .unwrap();
    // servos.try_update();
}

pub fn servos(c: &mut Criterion) {
    let servos = GmtServoMechanisms::<10, 1>::new(1000f64, FEM::from_env().unwrap())
        .build()
        .unwrap();

    c.bench_function("servos", |b| {
        b.to_async(tokio::runtime::Runtime::new().unwrap())
            .iter(|| update(&servos))
    });
}

pub fn servos_m1_figure(c: &mut Criterion) {
    let servos = GmtServoMechanisms::<10, 1>::new(1000f64, FEM::from_env().unwrap())
        .m1_segment_figure(M1SegmentFigure::new())
        .build()
        .unwrap();

    c.bench_function("servos M1 figure", |b| {
        b.to_async(tokio::runtime::Runtime::new().unwrap())
            .iter(|| update(&servos))
    });
}

criterion_group!(benches, servos, servos_m1_figure);
criterion_main!(benches);
