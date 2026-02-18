use std::path::Path;

use criterion::*;
use gmt_dos_systems_agws::{
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::Kernel,
    qp::{ActiveOptics, QP},
};
use interface::TryUpdate;

const N_MODE: usize = 271;
const M1_BM: usize = 27;
const M1_RBM: usize = 41;
const M2_RBM: usize = 41;

type K48 = ActiveOptics<1, M1_RBM, M2_RBM, M1_BM, N_MODE>;

#[inline]
fn update(om: &mut Kernel<K48>) {
    om.try_update().unwrap();
}

pub fn sh48(c: &mut Criterion) {
    let data_path = Path::new("/home/ubuntu/projects/im-sim-scripts/aco_loop_example/data");
    let aco = QP::<M1_RBM, M2_RBM, M1_BM, N_MODE>::new(
        //"../aco_impl_stdalone/SHAcO_qp_rhoP1e-3_kIp5.rs.pkl")
        //"rustCalib_AcO_rhoP1e-12_kIp5.rs.pkl")
        data_path.join("rustCalib_AcO_rhoP1e-12_kIp5.agws.pickle"),
    )
    .unwrap()
    .update_calib(
        Path::new("/home/ubuntu/projects/gmt-ns-im")
            .join("qp")
            .join("sh48_calibration.pkl"),
    )
    .unwrap()
    .build()
    .unwrap();

    let sh48 = ShackHartmannBuilder::sh48().reconstructor(aco);
    let mut sh48_kern = Kernel::<K48>::try_from(sh48).unwrap();
    c.bench_function("sh48", |b| b.iter(|| update(&mut sh48_kern)));
}

criterion_group!(
    benches,
    sh48,
);
criterion_main!(benches);
