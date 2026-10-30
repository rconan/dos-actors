use criterion::*;
use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
    calibration::ClosedLoopReconstructor,
    centroiding::CentroidsProcessing,
    sensors::{Camera, builders::CameraBuilder},
};
use gmt_dos_clients_io::optics::{Dev, Frame, SensorData};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;
use interface::{Read, Update, Write};

/// AGWS SH48 with the default parameters
fn agws_sh48() -> (OpticalModel<Camera<1>>, CentroidsProcessing) {
    let sh48 = ShackHartmannBuilder::<ClosedLoopReconstructor>::sh48();
    let omb = OpticalModelBuilder::<CameraBuilder<1>>::from(&sh48);
    let mut centroids: CentroidsProcessing = CentroidsProcessing::try_from(&omb).unwrap();
    omb.initialize(&mut centroids);
    let om = omb.build().unwrap();
    (om, centroids)
}

#[inline]
fn centroiding(om: &mut OpticalModel<Camera<1>>, centroids: &mut CentroidsProcessing) {
    <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(om)
        .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(centroids, data));
    centroids.update();
    black_box(<CentroidsProcessing as Write<SensorData>>::write(centroids));
}

pub fn sh48(c: &mut Criterion) {
    let (mut om, mut centroids) = agws_sh48();
    println!("{om}");
    let mut group = c.benchmark_group("AGWS SH48");
    group.bench_function("ray tracing", |b| b.iter(|| om.update()));
    om.update();
    group.bench_function("centroiding", |b| {
        b.iter(|| centroiding(&mut om, &mut centroids))
    });
    group.bench_function("ray tracing + centroiding", |b| {
        b.iter(|| {
            om.update();
            centroiding(&mut om, &mut centroids)
        })
    });
    group.finish();
}

criterion_group!(benches, sh48);
criterion_main!(benches);
