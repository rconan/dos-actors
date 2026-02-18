use std::{fs::File, path::Path};

use criterion::*;
use gmt_dos_actors::{
    actor::{Actor, Terminator},
    model::Model,
    prelude::{AddActorOutput, AddOuput, TryIntoInputs, vec_box},
    system::Sys,
};
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_crseo::calibration::Reconstructor;
use gmt_dos_clients_io::gmt_m2::fsm::M2FSMFsmCommand;
use gmt_dos_systems_agws::{
    Agws,
    agws::{
        sh24::{Sh24, kernel::Sh24Kern},
        sh48::{Sh48, kernel::Sh48Kern},
    },
    builder::shack_hartmann::ShackHartmannBuilder,
    qp::{ActiveOptics, QP},
};
use interface::{Read, Tick, UniqueIdentifier, Update, optics::OpticsState};

const N_MODE: usize = 271;
const M1_BM: usize = 27;
const M1_RBM: usize = 41;
const M2_RBM: usize = 41;

type K48 = ActiveOptics<SH48_RATE, M1_RBM, M2_RBM, M1_BM, N_MODE>;

const SH48_RATE: usize = 5000;
const SH24_RATE: usize = 5;

pub struct Void;
impl Update for Void {}
impl<U: UniqueIdentifier> Read<U> for Void {
    fn read(&mut self, _: interface::Data<U>) {}
}

// #[inline]
async fn update(agws_prime: &Sys<Agws<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>>) {
    let mut agws = Sys::<Agws<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>>::clone(agws_prime);
    let mut timer: Actor<Timer, 0, 1> = Timer::new(100).into();
    timer
        .add_output()
        .multiplex(2)
        .build::<Tick>()
        .into_input::<Sh48<SH48_RATE>>(&mut agws)
        .into_input::<Sh24<SH24_RATE>>(&mut agws)
        .unwrap();
    let mut void_48: Terminator<Void, SH48_RATE> = Void.into();
    AddActorOutput::<Sh48Kern<K48>, SH48_RATE, SH48_RATE>::add_output(&mut agws)
        .build::<OpticsState>()
        .into_input(&mut void_48)
        .unwrap();
    let mut void_24: Terminator<Void, SH24_RATE> = Void.into();
    AddActorOutput::<Sh24Kern<Sh24<SH24_RATE>>, SH24_RATE, SH24_RATE>::add_output(&mut agws)
        .build::<M2FSMFsmCommand>()
        .into_input(&mut void_24)
        .unwrap();

    Model::new(vec_box!(timer, agws, void_48, void_24))
        .quiet()
        .check()
        .unwrap()
        .run()
        .await
        .unwrap();
}

pub fn agws(c: &mut Criterion) {
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open(
            Path::new("/home/ubuntu/projects/gmt-ns-im")
                .join("calibrations/sh24/recon_sh24-to-pzt_pth.pkl"),
        )
        .unwrap(),
        Default::default(),
    )
    .unwrap();

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

    let agws = Agws::<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>::builder()
        .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
        .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        // .gmt(Gmt::builder().m1(
        //     config::m1::segment::RAW_MODES,
        //     config::m1::segment::N_RAW_MODE,
        // ))
        .sh24_calibration(recon)
        .sh48_calibration(aco)
        .build()
        .unwrap();
    // let timer

    c.bench_function("AGWS", |b| {
        b.to_async(tokio::runtime::Runtime::new().unwrap())
            .iter(|| update(&agws))
    });
}

criterion_group!(benches, agws);
criterion_main!(benches);
