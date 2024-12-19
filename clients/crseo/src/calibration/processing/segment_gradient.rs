use crate::{
    calibration::{
        Calib, Calibration, CalibrationMode, CalibrationSegment, PushPull, SegmentSensorBuilder,
    },
    sensors::SegmentGradientSensor,
    OpticalModel, OpticalModelBuilder,
};
use crseo::{
    gmt::{GmtBuilder, GmtMirror, GmtMirrorBuilder, GmtMx, MirrorGetSet},
    Builder, FromBuilder, Gmt,
};
use gmt_dos_clients_io::optics::SegmentTipTilt;
use interface::{Update, Write};
use std::time::Instant;

impl<const SID: u8> PushPull<SID> for SegmentGradientSensor {
    type Sensor = SegmentGradientSensor;

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
            <OpticalModel<SegmentGradientSensor> as Write<SegmentTipTilt>>::write(optical_model)
                .unwrap()
                .as_arc();

        cmd[i] *= -1.;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        let pull =
            <OpticalModel<SegmentGradientSensor> as Write<SegmentTipTilt>>::write(optical_model)
                .unwrap()
                .as_arc();

        cmd[i] = 0.0;

        push.iter()
            .zip(pull.iter())
            .map(|(x, y)| 0.5 * (x - y) as f64 / s)
            .collect()
    }
}

impl<M: GmtMx, const SID: u8> CalibrationSegment<M, SID> for SegmentGradientSensor
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = SegmentGradientSensor;

    fn calibrate(
        builder: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: CalibrationMode,
    ) -> super::Result<Calib> {
        let n_gs = builder.src.size;
        let mut wave = SegmentGradientSensor::builder().build()?;
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
                    calib.push(<SegmentGradientSensor as PushPull<SID>>::push_pull(
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
                    calib.push(<SegmentGradientSensor as PushPull<SID>>::push_pull(
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
            }
            _ => unimplemented!(),
        };
        let mut mask = vec![false; 14];
        mask[SID as usize - 1] = true;
        mask[SID as usize - 1 + 7] = true;

        Ok(Calib {
            sid: SID,
            n_mode,
            mask: mask.into_iter().cycle().take(14 * n_gs).collect(),
            c: calib.into_iter().flatten().collect(),
            mode: calib_mode,
            runtime: now.elapsed(),
            n_cols: None,
        })
    }
}

impl<M: GmtMx> Calibration<M> for SegmentGradientSensor
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = SegmentGradientSensor;
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{
        gmt::{GmtM1, GmtM2},
        Source,
    };
    use skyangle::Conversion;

    use crate::{calibration::algebra::CalibProps, OpticalModel};

    use super::*;

    #[test]
    fn gradients() -> Result<(), Box<dyn Error>> {
        let mut optical_model = OpticalModel::<SegmentGradientSensor>::builder().build()?;
        optical_model.gmt.keep(&[1]);
        optical_model.update();
        dbg!(optical_model.src.segment_gradients());
        Ok(())
    }

    #[test]
    fn calibrate() -> Result<(), Box<dyn Error>> {
        let optical_model = OpticalModel::<SegmentGradientSensor>::builder();

        let mut recon = <SegmentGradientSensor as Calibration<GmtM1>>::calibrate(
            &optical_model,
            CalibrationMode::r_xy(1f64.from_arcsec()),
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
    fn calibrate_m2_bm() -> Result<(), Box<dyn Error>> {
        let gmt_builder = Gmt::builder()
            .m1("bending modes", 27)
            .m2("Karhunen-Loeve", 66)
            .m1_truss_projection(false);

        let sampling_frequency = 1000_f64;

        let goiwfs_tt_om_builder = OpticalModel::<SegmentGradientSensor>::builder()
            .sampling_frequency(sampling_frequency)
            .gmt(gmt_builder.clone())
            .source(Source::builder().band("K"));

        let mut recon = <SegmentGradientSensor as Calibration<GmtM2>>::calibrate(
            &goiwfs_tt_om_builder,
            CalibrationMode::modes(3, 1e-8).start_from(2).ends_at(3),
        )?;

        // let optical_model =
        //     OpticalModel::<SegmentGradientSensor>::builder().gmt(Gmt::builder().m2_n_mode(15));

        // let mut recon = <SegmentGradientSensor as Calibration<GmtM2>>::calibrate(
        //     &optical_model,
        //     CalibrationMode::modes(3, 1e-8).start_from(2).ends_at(3),
        // )?;
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
        let optical_model = OpticalModel::<SegmentGradientSensor>::builder()
            .source(Source::builder().size(3).on_ring(6f32.from_arcmin()));

        // let mut om = optical_model.build()?;
        // om.update();
        // dbg!(om.src.segment_piston());

        let mut recon = <SegmentGradientSensor as Calibration<GmtM1>>::calibrate(
            &optical_model,
            CalibrationMode::r_xy(1f64.from_arcsec()),
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
