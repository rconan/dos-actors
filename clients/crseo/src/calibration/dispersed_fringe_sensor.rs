use super::{
    Calib, Calibrate, CalibrateSegment, PushPull, Reconstructor, SegmentSensorBuilder,
    SensorBuilder,
};
use crate::sensors::{DispersedFringeSensor, DispersedFringeSensorProcessing};
use crseo::{
    gmt::{GmtBuilder, GmtMirror, GmtMirrorBuilder, GmtMx, MirrorGetSet},
    Gmt,
};
use interface::Update;
use std::time::Instant;

impl<const SID: u8> PushPull<SID> for DispersedFringeSensorProcessing {
    type Sensor = DispersedFringeSensor<1, 1>;

    fn push_pull<F>(
        &mut self,
        optical_model: &mut crate::OpticalModel<<Self as PushPull<SID>>::Sensor>,
        i: usize,
        s: f64,
        cmd: &mut [f64],
        cmd_fn: F,
    ) -> Vec<f64>
    where
        F: Fn(&mut crseo::Gmt, u8, &[f64]),
    {
        cmd[i] = s;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        self.process(&optical_model.sensor_mut().unwrap().fft_frame())
            .intercept();
        let push = self.intercept.clone();

        cmd[i] *= -1.;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        cmd[i] = 0.0;

        self.process(&optical_model.sensor_mut().unwrap().fft_frame())
            .intercept();
        let pull = self.intercept.clone();

        push.into_iter()
            .zip(pull.into_iter())
            .map(|(x, y)| 0.5 * (x - y) as f64 / s)
            .collect()
    }
}

impl<M: GmtMx, const SID: u8> CalibrateSegment<M, SID> for DispersedFringeSensorProcessing
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = DispersedFringeSensor<1, 1>;

    fn calibrate(
        builder: crate::OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: super::CalibrationMode,
    ) -> super::Result<Calib> {
        let mut dfs_processor = DispersedFringeSensorProcessing::new();
        {
            let mut om_dfs11 = builder.clone().build()?;
            om_dfs11.update();
            let mut dfsp11 = DispersedFringeSensorProcessing::from(om_dfs11.sensor_mut().unwrap());
            dfs_processor.set_reference(dfsp11.intercept());
        }
        match calib_mode {
            super::CalibrationMode::RBM(stroke) => {
                let mut optical_model = builder.build()?;
                let mut tr_xyz = [0f64; 6];
                let mut calib = vec![];

                let now = Instant::now();
                for i in calib_mode.range() {
                    let Some(s) = stroke[i] else {
                        continue;
                    };
                    calib.push(
                        <DispersedFringeSensorProcessing as PushPull<SID>>::push_pull(
                            &mut dfs_processor,
                            &mut optical_model,
                            i,
                            s,
                            &mut tr_xyz,
                            |gmt, sid, cmd| {
                                <Gmt as GmtMirror<M>>::as_mut(gmt).set_rigid_body_motions(sid, cmd);
                            },
                        ),
                    );
                }
                Ok(Calib {
                    sid: SID,
                    n_mode: 6,
                    c: calib.into_iter().flatten().collect(),
                    mask: vec![],
                    mode: calib_mode,
                    runtime: now.elapsed(),
                    n_cols: None,
                })
            }
            super::CalibrationMode::Modes { n_mode, stroke, .. } => {
                let gmt = builder.clone().gmt.n_mode::<M>(n_mode);
                let mut optical_model = builder.gmt(gmt).build()?;

                let mut a = vec![0f64; n_mode];
                let mut calib = vec![];

                let now = Instant::now();
                for i in calib_mode.range() {
                    calib.push(
                        <DispersedFringeSensorProcessing as PushPull<SID>>::push_pull(
                            &mut dfs_processor,
                            &mut optical_model,
                            i,
                            stroke,
                            &mut a,
                            |gmt, sid, cmd| {
                                <Gmt as GmtMirror<M>>::as_mut(gmt).set_segment_modes(sid, cmd);
                            },
                        ),
                    );
                }
                Ok(Calib {
                    sid: SID,
                    n_mode,
                    c: calib.into_iter().flatten().collect(),
                    mask: vec![],
                    mode: calib_mode,
                    runtime: now.elapsed(),
                    n_cols: None,
                })
            }
        }
    }
}

impl<M: GmtMx> Calibrate<M> for DispersedFringeSensorProcessing
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
{
    type Sensor = DispersedFringeSensor<1, 1>;

    fn calibrate(
        optical_model: &crate::OpticalModelBuilder<SensorBuilder<M, Self>>,
        calib_mode: super::CalibrationMode,
    ) -> super::Result<super::Reconstructor> {
        let c1 = <DispersedFringeSensorProcessing as CalibrateSegment<M, 1>>::calibrate(
            optical_model.clone(),
            calib_mode,
        )?;
        let c2 = <DispersedFringeSensorProcessing as CalibrateSegment<M, 2>>::calibrate(
            optical_model.clone(),
            calib_mode,
        )?;
        let c3 = <DispersedFringeSensorProcessing as CalibrateSegment<M, 3>>::calibrate(
            optical_model.clone(),
            calib_mode,
        )?;
        let c4 = <DispersedFringeSensorProcessing as CalibrateSegment<M, 4>>::calibrate(
            optical_model.clone(),
            calib_mode,
        )?;
        let c5 = <DispersedFringeSensorProcessing as CalibrateSegment<M, 5>>::calibrate(
            optical_model.clone(),
            calib_mode,
        )?;
        let c6 = <DispersedFringeSensorProcessing as CalibrateSegment<M, 6>>::calibrate(
            optical_model.clone(),
            calib_mode,
        )?;

        let iter =
            c1.c.iter()
                .chain(c2.c.iter())
                .chain(c3.c.iter())
                .chain(c4.c.iter())
                .chain(c5.c.iter())
                .chain(c6.c.iter())
                .map(|x| *x);
        let mut calib = c1.clone();
        calib.sid = 0;
        calib.c = iter.collect();
        calib.mask = vec![true; calib.c.len() / 6];
        calib.runtime = c1.runtime + c2.runtime + c3.runtime + c4.runtime + c5.runtime + c6.runtime;
        calib.n_cols = Some(6);
        Ok(Reconstructor::new(vec![calib]))
    }
}
