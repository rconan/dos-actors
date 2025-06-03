use crseo::{FromBuilder, Gmt, wavefrontsensor::PhaseSensor};
use gmt_dos_clients_crseo::OpticalModel;
use gmt_dos_clients_io::{gmt_m2::asm::segment::FaceSheetFigure, optics::Wavefront};
use interface::{Read, Size, Update, Write, units::MuM};

/*
GMT_MODES_PATH=... cargo run --release --example mirror-figure
 */

const N_MODE: usize = 675;
const M2_MODES: &'static str = "ASM_IFs";

fn main() -> anyhow::Result<()> {
    let mut optical_model: OpticalModel = OpticalModel::<PhaseSensor>::builder()
        .gmt(Gmt::builder().m2(M2_MODES, N_MODE))
        .build()?;

    let mut modes = vec![0f64; N_MODE];
    [3, 27, 75, 147, 243, 383, 507, 675]
        .into_iter()
        .for_each(|i| {
            modes[i - 1] = 1e-6;
        });
    <OpticalModel as Read<FaceSheetFigure<1>>>::read(&mut optical_model, modes.clone().into());
    <OpticalModel as Read<FaceSheetFigure<3>>>::read(&mut optical_model, modes.clone().into());
    <OpticalModel as Read<FaceSheetFigure<7>>>::read(&mut optical_model, modes.into());

    optical_model.update();

    let phase = <OpticalModel as Write<MuM<Wavefront>>>::write(&mut optical_model).unwrap();
    let n_px = (<OpticalModel as Size<Wavefront>>::len(&mut optical_model) as f64).sqrt() as usize;

    let _: complot::Heatmap = ((phase.as_arc().as_slice(), (n_px, n_px)), None).into();

    Ok(())
}
