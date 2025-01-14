/*! # ASM Influence Functions Pattern

```
export GMT_MODES_PATH=/home/ubuntu/CEO/gmtMirrors/
export FEM_REPO=/home/ubuntu/mnt/20240401_1605_zen_30_M1_202110_ASM_202403_Mount_202305_IDOM_concreteAndFoundation_M1Fans/
export MOUNT_MODEL=MOUNT_FDR_8kHz
```
*/

use crseo::{FromBuilder, Gmt};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::Signals;
use gmt_dos_clients_crseo::{sensors::NoSensor, OpticalModel};
use gmt_dos_clients_io::{
    gmt_m2::asm::{segment::FaceSheetFigure, M2ASMAsmCommand},
    optics::Wavefront,
};
use gmt_dos_clients_servos::{AsmsServo, GmtM2, GmtServoMechanisms};
use gmt_fem::FEM;
use interface::{units::MuM, Size, Write};

const ACTUATOR_RATE: usize = 80;
const N_MODE: usize = 675;
const M2_MODES: &'static str = "asms_ifs_gmt-fem";

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 8000;
    // let sim_duration = 1_usize; // second
    let n_step = 400;

    // $FEM_REPO-related variables
    let fem = FEM::from_env()?;

    let mut modes = vec![vec![0f64; N_MODE]; 7];
    [3, 27, 75, 147, 243, 363, 507, 675]
        .into_iter()
        .for_each(|i| {
            modes[0][i - 1] = 1e-6;
            modes[2][i - 1] = 1e-6;
            modes[6][i - 1] = 1e-6;
        });
    let asms_cmd = Signals::from((modes.into_iter().flatten().collect::<Vec<f64>>(), n_step));

    // GMT Servomechanisms system
    let gmt_servos =
        GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
            .asms_servo(AsmsServo::new().facesheet(Default::default()))
            .build()?;

    let optical_model: OpticalModel = OpticalModel::<NoSensor>::builder()
        .gmt(Gmt::builder().m2(M2_MODES, N_MODE))
        .build()?;

    actorscript! {
        1: asms_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
        8: optical_model
        1: {gmt_servos::GmtFem}[FaceSheetFigure<1>] -> optical_model
        1: {gmt_servos::GmtFem}[FaceSheetFigure<3>] -> optical_model
        1: {gmt_servos::GmtFem}[FaceSheetFigure<7>] -> optical_model
    }

    let mut opm = optical_model.lock().await;
    let phase = <OpticalModel as Write<MuM<Wavefront>>>::write(&mut opm).unwrap();
    let n_px = (<OpticalModel as Size<Wavefront>>::len(&mut opm) as f64).sqrt() as usize;

    let _: complot::Heatmap = ((phase.as_arc().as_slice(), (n_px, n_px)), None).into();

    Ok(())
}
