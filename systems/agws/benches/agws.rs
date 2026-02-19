use criterion::*;
use gmt_dos_clients_crseo::{OpticalModel, calibration::ClosedLoopReconstructor, sensors::Camera};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;
use interface::Update;

#[inline]
fn update(om: &mut OpticalModel<Camera>) {
    om.update();
}

pub fn sh48(c: &mut Criterion) {
    let sh48 = ShackHartmannBuilder::<ClosedLoopReconstructor>::sh48();
    let mut om = OpticalModel::try_from(&sh48).unwrap();
    println!("{om}");
    c.bench_function("sh48", |b| b.iter(|| update(&mut om)));
}

pub fn sh48_calibration_srcs(c: &mut Criterion) {
    let sh48 = ShackHartmannBuilder::<ClosedLoopReconstructor>::sh48().use_calibration_src();
    let mut om = OpticalModel::try_from(&sh48).unwrap();
    println!("{om}");
    c.bench_function("sh48_calibration_srcs", |b| b.iter(|| update(&mut om)));
}

pub fn sh24(c: &mut Criterion) {
    let sh24 = ShackHartmannBuilder::<ClosedLoopReconstructor>::sh24();
    let mut om = OpticalModel::try_from(&sh24).unwrap();
    println!("{om}");
    c.bench_function("sh24", |b| b.iter(|| update(&mut om)));
}

pub fn sh24_calibration_srcs(c: &mut Criterion) {
    let sh24 = ShackHartmannBuilder::<ClosedLoopReconstructor>::sh24().use_calibration_src();
    let mut om = OpticalModel::try_from(&sh24).unwrap();
    println!("{om}");
    c.bench_function("sh24_calibration_srcs", |b| b.iter(|| update(&mut om)));
}

criterion_group!(
    benches,
    sh48,
    sh48_calibration_srcs,
    sh24,
    sh24_calibration_srcs
);
criterion_main!(benches);
