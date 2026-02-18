use criterion::*;
use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
use interface::Update;

#[inline]
fn ray_tracing_fn(om: &mut OpticalModel) {
    om.update();
}

pub fn ray_tracing(c: &mut Criterion) {
    let mut om = OpticalModel::<NoSensor>::builder().build().unwrap();
    println!("{om}");
    c.bench_function("ray tracing", |b| b.iter(|| ray_tracing_fn(&mut om)));
}

criterion_group!(benches, ray_tracing);
criterion_main!(benches);
