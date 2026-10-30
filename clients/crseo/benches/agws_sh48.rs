use criterion::*;
use crseo::{FromBuilder, Source, imaging::LensletArray};
use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModel, centroiding::CentroidsProcessing, sensors::Camera,
};
use gmt_dos_clients_io::optics::{Dev, Frame, SensorData};
use interface::{Read, Update, Write};
use skyangle::Conversion;

/// AGWS SH48: 3 guide stars on a 6' ring, 48x48 lenslets
fn agws_sh48(n_px_lenslet: usize) -> (OpticalModel<Camera<1>>, CentroidsProcessing) {
    let n_gs = 3;
    let agws_gs = Source::builder().size(n_gs).on_ring(6f32.from_arcmin());
    let sh48 = Camera::<1>::builder()
        .n_sensor(n_gs)
        .lenslet_array(
            LensletArray::default()
                .n_side_lenslet(48)
                .n_px_lenslet(n_px_lenslet),
        )
        .lenslet_flux(0.75);
    let mut centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sh48).unwrap();
    let omb = OpticalModel::<Camera<1>>::builder()
        .source(agws_gs)
        .sensor(sh48);
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
    let mut group = c.benchmark_group("AGWS SH48");
    for n_px_lenslet in [16, 32] {
        let (mut om, mut centroids) = agws_sh48(n_px_lenslet);
        println!("{om}");
        group.bench_with_input(
            BenchmarkId::new("ray tracing", n_px_lenslet),
            &n_px_lenslet,
            |b, _| b.iter(|| om.update()),
        );
        om.update();
        group.bench_with_input(
            BenchmarkId::new("centroiding", n_px_lenslet),
            &n_px_lenslet,
            |b, _| b.iter(|| centroiding(&mut om, &mut centroids)),
        );
        group.bench_with_input(
            BenchmarkId::new("ray tracing + centroiding", n_px_lenslet),
            &n_px_lenslet,
            |b, _| {
                b.iter(|| {
                    om.update();
                    centroiding(&mut om, &mut centroids)
                })
            },
        );
    }
    group.finish();
}

criterion_group!(benches, sh48);
criterion_main!(benches);
