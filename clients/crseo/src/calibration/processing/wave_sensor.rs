use crate::{
    calibration::{
        Calib, Calibrate, CalibrateSegment, CalibrationMode, PushPull, SegmentSensorBuilder,
    },
    sensors::WaveSensor,
    OpticalModel, OpticalModelBuilder,
};
use crseo::{
    gmt::{GmtBuilder, GmtMirror, GmtMirrorBuilder, GmtMx, MirrorGetSet},
    Gmt,
};
use interface::Update;
use std::time::Instant;

impl<const SID: u8> PushPull<SID> for WaveSensor {
    type Sensor = WaveSensor;

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

        let mut mask: Vec<_> = optical_model
            .sensor()
            .unwrap()
            .amplitude()
            .iter()
            .map(|a| *a > 0.)
            .collect();
        let push = optical_model.sensor().unwrap().phase().to_vec();

        cmd[i] *= -1.;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        mask.iter_mut()
            .zip(optical_model.sensor().unwrap().amplitude().iter())
            .for_each(|(m, a)| *m &= *a > 0.);

        cmd[i] = 0.0;

        push.iter()
            .zip(optical_model.sensor().unwrap().phase().iter())
            .zip(mask.into_iter())
            .map(|((x, y), m)| if m { 0.5 * (x - y) as f64 / s } else { 0. })
            .collect()
    }
}

impl<M: GmtMx, const SID: u8> CalibrateSegment<M, SID> for WaveSensor
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = WaveSensor;

    fn calibrate(
        builder: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: CalibrationMode,
    ) -> super::Result<Calib> {
        // let mut centroids = Centroids::try_from(builder.sensor.as_ref().unwrap())?;
        let mut wave = Default::default();
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
                    calib.push(<WaveSensor as PushPull<SID>>::push_pull(
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
                    calib.push(<WaveSensor as PushPull<SID>>::push_pull(
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
        let n = calib[0].len();
        let mask = calib.iter().fold(vec![true; n], |mut m, c| {
            m.iter_mut()
                .zip(c.iter())
                .for_each(|(m, c)| *m &= c.abs() > 0.);
            m
        });

        Ok(Calib {
            sid: SID,
            n_mode,
            c: calib
                .into_iter()
                .flat_map(|c| c.into_iter().zip(&mask).flat_map(|(c, m)| m.then(|| c)))
                .collect(),
            mask,
            mode: calib_mode,
            runtime: now.elapsed(),
            n_cols: None,
        })
    }
}

impl<M: GmtMx> Calibrate<M> for WaveSensor
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = WaveSensor;

    /*     fn calibrate(
        optical_model: OpticalModelBuilder<SensorBuilder<M, Self>>,
        calib_mode: CalibrationMode,
    ) -> super::Result<Reconstructor> {
        let c1 =
            <WaveSensor as CalibrateSegment<M, 1>>::calibrate(optical_model.clone(), calib_mode)?;
        let c2 =
            <WaveSensor as CalibrateSegment<M, 2>>::calibrate(optical_model.clone(), calib_mode)?;
        let c3 =
            <WaveSensor as CalibrateSegment<M, 3>>::calibrate(optical_model.clone(), calib_mode)?;
        let c4 =
            <WaveSensor as CalibrateSegment<M, 4>>::calibrate(optical_model.clone(), calib_mode)?;
        let c5 =
            <WaveSensor as CalibrateSegment<M, 5>>::calibrate(optical_model.clone(), calib_mode)?;
        let c6 =
            <WaveSensor as CalibrateSegment<M, 6>>::calibrate(optical_model.clone(), calib_mode)?;
        let c7 =
            <WaveSensor as CalibrateSegment<M, 7>>::calibrate(optical_model.clone(), calib_mode)?;
        Ok(Reconstructor::new(vec![c1, c2, c3, c4, c5, c6, c7]))
    } */
}
