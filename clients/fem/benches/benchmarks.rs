use criterion::{Criterion, black_box, criterion_group, criterion_main};
use gmt_dos_clients_fem::{
    DiscreteModalSolver,
    solvers::{Exponential, ExponentialMatrix, Solver},
};

pub fn exponential_solve(c: &mut Criterion) {
    let n_io = 10_000;
    let data = vec![1f64; n_io];
    let mut ss = Exponential::from_second_order(1e-3, 1f64, 0.005, data.clone(), data);
    let u = vec![0f64; n_io];
    c.bench_function(&format!("exponential solver ({n_io})"), |b| {
        b.iter(|| {
            let _ = ss.solve(&u);
        })
    });
}

pub fn exponential_matrix_solve(c: &mut Criterion) {
    let n_io = 10_000;
    let data = vec![1f64; n_io];
    let mut ss = ExponentialMatrix::from_second_order(1e-3, 1f64, 0.005, data.clone(), data);
    let u = vec![0f64; n_io];
    c.bench_function(&format!("exponential matrix solver ({n_io})"), |b| {
        b.iter(|| {
            let _ = ss.solve(&u);
        })
    });
}

pub fn statespace(c: &mut Criterion) {
    let mut dsss = vec![];
    for i in 2..5 {
        let n_io = 2 << i;
        for m in 2..5 {
            let n_mode = 2 << m;

            let data = vec![1f64; n_io];
            let ss = Exponential::from_second_order(1e-3, 1f64, 0.005, data.clone(), data);
            let dss = DiscreteModalSolver::<Exponential> {
                u: vec![0f64; n_io],
                y: vec![0f64; n_io],
                y_sizes: vec![],
                state_space: vec![ss; n_mode],
                psi_dcg: None,
                psi_times_u: vec![],
                ins: vec![],
                outs: vec![],
            };
            dsss.push((dss, (n_io, n_mode)));
        }
    }

    let mut group = c.benchmark_group("State Space ");
    for (mut dss, (n_io, n_mode)) in dsss.into_iter() {
        group.bench_function(&format!("({n_io},{n_mode})"), |b| {
            b.iter(|| {
                let _ = dss.next();
            })
        });
    }
    group.finish()
}

criterion_group!(
    benches,
    exponential_solve,
    exponential_matrix_solve,
    statespace
);
criterion_main!(benches);
