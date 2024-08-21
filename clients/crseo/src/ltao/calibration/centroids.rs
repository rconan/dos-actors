use super::{Calib, Calibrate, PushPull, Reconstructor};
use crate::{CalibrateSegment, CalibrationMode, Centroids, OpticalModel, OpticalModelBuilder};
use crseo::gmt::{GmtBuilder, GmtMirror, GmtMirrorBuilder, GmtMx, MirrorGetSet};
use crseo::imaging::ImagingBuilder;
use crseo::{FromBuilder, Gmt, Imaging};
use interface::Update;
use std::time::Instant;

impl<const SID: u8> PushPull<SID> for Centroids {
    type Sensor = Imaging;
    fn push_pull<F>(
        &mut self,
        optical_model: &mut OpticalModel<Self::Sensor>,
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

        self.centroids.process(
            &optical_model.sensor().unwrap().frame(),
            Some(&self.reference),
        );
        optical_model.sensor().unwrap().reset();

        let push = self
            .centroids
            .grab()
            .valids(Some(&self.reference.valid_lenslets));

        cmd[i] *= -1.;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        cmd[i] = 0.0;

        self.centroids.process(
            &optical_model.sensor().unwrap().frame(),
            Some(&self.reference),
        );
        optical_model.sensor().unwrap().reset();

        let pull = self
            .centroids
            .grab()
            .valids(Some(&self.reference.valid_lenslets));
        push.into_iter()
            .zip(pull.into_iter())
            .map(|(x, y)| 0.5 * (x - y) as f64 / s)
            .collect()
    }
}

impl<M: GmtMx, const SID: u8> CalibrateSegment<M, SID> for Centroids
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type SegmentSensorBuilder = ImagingBuilder;

    fn calibrate(
        builder: OpticalModelBuilder<Self::SegmentSensorBuilder>,
        calib_mode: CalibrationMode,
    ) -> super::Result<Calib> {
        let mut centroids = Centroids::try_from(builder.sensor.as_ref().unwrap())?;
        match calib_mode {
            CalibrationMode::RBM(stroke) => {
                let mut optical_model = builder.build()?;
                optical_model.gmt.keep(&[SID as i32]);
                centroids.setup(&mut optical_model);

                let mut tr_xyz = [0f64; 6];
                let mut calib = vec![];
                // let cmd_fn = |gmt, sid, cmd| {
                //     <Gmt as GmtMirror<M>>::as_mut(gmt).set_rigid_body_motions(sid, cmd);
                // };
                let now = Instant::now();
                for i in 0..6 {
                    let Some(s) = stroke[i] else {
                        continue;
                    };
                    calib.push(<Centroids as PushPull<SID>>::push_pull(
                        &mut centroids,
                        &mut optical_model,
                        i,
                        s,
                        &mut tr_xyz,
                        |gmt, sid, cmd| {
                            <Gmt as GmtMirror<M>>::as_mut(gmt).set_rigid_body_motions(sid, cmd);
                        },
                    ));
                }
                // self.mask = mask;
                //self.c = calib.into_iter().flatten().collect()
                Ok(Calib {
                    sid: SID,
                    n_mode: 6,
                    c: calib.into_iter().flatten().collect(),
                    mask: vec![],
                    mode: calib_mode,
                    runtime: now.elapsed(),
                })
            }
            CalibrationMode::Modes {
                n_mode,
                stroke,
                start_idx,
            } => {
                log::info!("Calibrating segment modes ...");
                let mut optical_model = builder.gmt(Gmt::builder().n_mode::<M>(n_mode)).build()?;
                optical_model.gmt.keep(&[SID as i32]);
                centroids.setup(&mut optical_model);

                let mut a = vec![0f64; n_mode];
                let mut calib = vec![];
                // let cmd_fn = |gmt, sid, cmd| {
                //     <Gmt as GmtMirror<M>>::as_mut(gmt).set_segment_modes(sid, cmd);
                // };
                let now = Instant::now();
                for i in start_idx..n_mode {
                    calib.push(<Centroids as PushPull<SID>>::push_pull(
                        &mut centroids,
                        &mut optical_model,
                        i,
                        stroke,
                        &mut a,
                        |gmt, sid, cmd| {
                            <Gmt as GmtMirror<M>>::as_mut(gmt).set_segment_modes(sid, cmd);
                        },
                    ));
                }
                let iter = centroids.reference.valid_lenslets.iter().map(|&v| v > 0);
                Ok(Calib {
                    sid: SID,
                    n_mode,
                    c: calib.into_iter().flatten().collect(),
                    mask: iter.clone().chain(iter).collect(),
                    mode: calib_mode,
                    runtime: now.elapsed(),
                })
            }
        }
    }
}

impl<M: GmtMx> Calibrate<M> for Centroids
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type SensorBuilder = ImagingBuilder;

    fn calibrate(
        optical_model: OpticalModelBuilder<Self::SensorBuilder>,
        calib_mode: CalibrationMode,
    ) -> super::Result<Reconstructor> {
        let c1 =
            <Centroids as CalibrateSegment<M, 1>>::calibrate(optical_model.clone(), calib_mode)?;
        let c2 =
            <Centroids as CalibrateSegment<M, 2>>::calibrate(optical_model.clone(), calib_mode)?;
        let c3 =
            <Centroids as CalibrateSegment<M, 3>>::calibrate(optical_model.clone(), calib_mode)?;
        let c4 =
            <Centroids as CalibrateSegment<M, 4>>::calibrate(optical_model.clone(), calib_mode)?;
        let c5 =
            <Centroids as CalibrateSegment<M, 5>>::calibrate(optical_model.clone(), calib_mode)?;
        let c6 =
            <Centroids as CalibrateSegment<M, 6>>::calibrate(optical_model.clone(), calib_mode)?;
        let c7 =
            <Centroids as CalibrateSegment<M, 7>>::calibrate(optical_model.clone(), calib_mode)?;
        Ok(Reconstructor::new(vec![c1, c2, c3, c4, c5, c6, c7]))
    }
}
