use criterion::*;
use crseo::{
    FromBuilder, Source,
    imaging::{Detector, LensletArray},
};
use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModel, centroiding::CentroidsProcessing, sensors::Camera,
};
use gmt_dos_clients_io::optics::{Dev, Frame, SensorData};
use interface::{Read, Update, Write};
use skyangle::Conversion;

/// AGWS SH48 with the default parameters of `gmt_dos-systems_agws`
/// (`ShackHartmannBuilder::sh48()`)
///
/// 3 guide stars (VIS) at (6',15°), (5',145°) and (7',255°),
/// 48x48 lenslets, 0.4" pixel scale and 3.2" lenslet field-of-view
fn agws_sh48() -> (OpticalModel<Camera<1>>, CentroidsProcessing) {
    let agws_gs = Source::builder().band("VIS").size(3).zenith_azimuth(
        vec![6f32.from_arcmin(), 5f32.from_arcmin(), 7f32.from_arcmin()],
        vec![15f32.to_radians(), 145f32.to_radians(), 255f32.to_radians()],
    );
    let sh48 = Camera::<1>::builder()
        .n_sensor(3)
        .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(18))
        .detector(Detector::default().n_px_imagelet(24).n_px_framelet(8))
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
