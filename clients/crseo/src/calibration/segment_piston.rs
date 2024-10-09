use crate::{sensors::SegmentPistonSensor, OpticalModel, OpticalModelBuilder};
use crseo::{
    gmt::{GmtBuilder, GmtMirror, GmtMirrorBuilder, GmtMx, MirrorGetSet},
    Builder, FromBuilder, Gmt,
};
use gmt_dos_clients_io::optics::SegmentPiston;
use interface::{Update, Write};
use std::time::Instant;

use super::{Calib, Calibrate, CalibrateSegment, CalibrationMode, PushPull, SegmentSensorBuilder};

impl<const SID: u8> PushPull<SID> for SegmentPistonSensor {
    type Sensor = SegmentPistonSensor;

    fn push_pull<F>(
        &mut self,
        optical_model: &mut OpticalModel<<Self as PushPull<SID>>::Sensor>,
        i: usize,
        s: f64,
        cmd: &mut [f64],
        cmd_fn: F,
    ) -> Vec<f64>
    where
        F: Fn(&mut Gmt, u8, &[f64]),
    {
        cmd[i] = s;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        let push =
            <OpticalModel<SegmentPistonSensor> as Write<SegmentPiston>>::write(optical_model)
                .unwrap()
                .as_arc();

        cmd[i] *= -1.;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        let pull =
            <OpticalModel<SegmentPistonSensor> as Write<SegmentPiston>>::write(optical_model)
                .unwrap()
                .as_arc();

        cmd[i] = 0.0;

        push.iter()
            .zip(pull.iter())
            .map(|(x, y)| 0.5 * (x - y) as f64 / s)
            .collect()
    }
}

impl<M: GmtMx, const SID: u8> CalibrateSegment<M, SID> for SegmentPistonSensor
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = SegmentPistonSensor;

    fn calibrate(
        builder: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: CalibrationMode,
    ) -> super::Result<Calib> {
        let mut wave = SegmentPistonSensor::builder().build()?;
        let now = Instant::now();
        let (calib, n_mode) = match calib_mode {
            CalibrationMode::RBM(stroke) => {
                let mut optical_model = builder.build()?;
                optical_model.gmt.keep(&[SID as i32]);

                let mut tr_xyz = [0f64; 6];
                let mut calib = vec![];
                // let cmd_fn = |gmt, sid, cmd| {
                //     <Gmt as GmtMirror<M>>::as_mut(gmt).set_rigid_body_motions(sid, cmd);
                // };
                for i in calib_mode.range() {
                    let Some(s) = stroke[i] else {
                        continue;
                    };
                    calib.push(<SegmentPistonSensor as PushPull<SID>>::push_pull(
                        &mut wave,
                        &mut optical_model,
                        i,
                        s,
                        &mut tr_xyz,
                        |gmt, sid, cmd| {
                            <Gmt as GmtMirror<M>>::as_mut(gmt).set_rigid_body_motions(sid, cmd);
                        },
                    ));
                }
                (calib, 6)
            }
            CalibrationMode::Modes { n_mode, stroke, .. } => {
                log::info!("Calibrating segment modes ...");
                let gmt = builder.clone().gmt.n_mode::<M>(n_mode);
                let mut optical_model = builder.gmt(gmt).build()?;
                optical_model.gmt.keep(&[SID as i32]);

                let mut a = vec![0f64; n_mode];
                let mut calib = vec![];
                // let cmd_fn = |gmt, sid, cmd| {
                //     <Gmt as GmtMirror<M>>::as_mut(gmt).set_segment_modes(sid, cmd);
                // };
                for i in calib_mode.range() {
                    calib.push(<SegmentPistonSensor as PushPull<SID>>::push_pull(
                        &mut wave,
                        &mut optical_model,
                        i,
                        stroke,
                        &mut a,
                        |gmt, sid, cmd| {
                            <Gmt as GmtMirror<M>>::as_mut(gmt).set_segment_modes(sid, cmd);
                        },
                    ));
                }
                (calib, n_mode)
            } // _ => unimplemented!(),
        };
        let mut mask = vec![false; 7];
        mask[SID as usize - 1] = true;

        Ok(Calib {
            sid: SID,
            n_mode,
            mask: mask.into_iter().cycle().take(7 * calib[0].len()).collect(),
            c: calib.into_iter().flatten().collect(),
            mode: calib_mode,
            runtime: now.elapsed(),
            n_cols: None,
        })
    }
}

impl<M: GmtMx> Calibrate<M> for SegmentPistonSensor
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = SegmentPistonSensor;
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{gmt::GmtM1, Source};
    use skyangle::Conversion;

    use crate::{calibration::algebra::CalibProps, OpticalModel};

    use super::*;

    #[test]
    fn calibrate() -> Result<(), Box<dyn Error>> {
        let optical_model = OpticalModel::<SegmentPistonSensor>::builder();

        let mut recon = <SegmentPistonSensor as Calibrate<GmtM1>>::calibrate(
            &optical_model,
            CalibrationMode::t_z(1e-6),
        )?;
        recon.pseudoinverse();
        println!("{recon}");

        recon.calib_slice().iter().for_each(|c| {
            println!(
                "{:.2?} {:?}",
                c.as_slice(),
                c.mask_as_slice()
                    .iter()
                    .map(|x| if *x { 1 } else { 0 })
                    .collect::<Vec<_>>()
            )
        });

        Ok(())
    }

    #[test]
    fn calibrate_gs() -> Result<(), Box<dyn Error>> {
        let optical_model = OpticalModel::<SegmentPistonSensor>::builder()
            .source(Source::builder().size(3).on_ring(6f32.from_arcmin()));

        // let mut om = optical_model.build()?;
        // om.update();
        // dbg!(om.src.segment_piston());

        let mut recon = <SegmentPistonSensor as Calibrate<GmtM1>>::calibrate(
            &optical_model,
            CalibrationMode::t_z(1e-6),
        )?;
        recon.pseudoinverse();
        println!("{recon}");

        recon.calib_slice().iter().for_each(|c| {
            println!(
                "{:.2?} {:?}",
                c.as_slice(),
                c.mask_as_slice()
                    .iter()
                    .map(|x| if *x { 1 } else { 0 })
                    .collect::<Vec<_>>()
            )
        });

        Ok(())
    }
}
