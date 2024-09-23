use super::{Calib, Calibrate, CalibrateSegment, CalibrationMode, PushPull, SegmentSensorBuilder};
use crate::{
    centroiding::{CentroidKind, Centroids, Full, ZeroMean},
    OpticalModel, OpticalModelBuilder,
};
use crseo::{
    gmt::{GmtBuilder, GmtMirror, GmtMirrorBuilder, GmtMx, MirrorGetSet},
    Gmt, Imaging,
};
use interface::Update;
use std::time::Instant;

trait ValidCentroids {
    fn get(&mut self) -> Vec<Vec<f32>>;
}
impl ValidCentroids for Centroids<Full> {
    fn get(&mut self) -> Vec<Vec<f32>> {
        self.centroids
            .grab()
            .valids(Some(&self.reference.valid_lenslets))
    }
}
impl ValidCentroids for Centroids<ZeroMean> {
    fn get(&mut self) -> Vec<Vec<f32>> {
        self.centroids
            .grab()
            // .remove_mean(Some(&self.reference.valid_lenslets))
            .valids(Some(&self.reference.valid_lenslets))
    }
}

impl<K: CentroidKind, const SID: u8> PushPull<SID> for Centroids<K>
where
    Centroids<K>: ValidCentroids,
{
    type PushPullSensor = Imaging;
    fn push_pull<F>(
        &mut self,
        optical_model: &mut OpticalModel<Self::PushPullSensor>,
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
        optical_model.sensor_mut().unwrap().reset();

        // let push = self
        //     .centroids
        //     .grab()
        //     .valids(Some(&self.reference.valid_lenslets));
        let push = <Self as ValidCentroids>::get(self);

        cmd[i] *= -1.;
        cmd_fn(&mut optical_model.gmt, SID, cmd);
        optical_model.update();

        cmd[i] = 0.0;

        self.centroids.process(
            &optical_model.sensor().unwrap().frame(),
            Some(&self.reference),
        );
        optical_model.sensor_mut().unwrap().reset();

        let pull = <Self as ValidCentroids>::get(self);
        push.into_iter()
            .flatten()
            .zip(pull.into_iter().flatten())
            .map(|(x, y)| 0.5 * (x - y) as f64 / s)
            .collect()
    }
}

impl<K: CentroidKind, M: GmtMx, const SID: u8> CalibrateSegment<M, SID> for Centroids<K>
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
    Centroids<K>: ValidCentroids,
{
    type SegmentSensor = Imaging;

    fn calibrate(
        builder: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
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
                for i in calib_mode.range() {
                    let Some(s) = stroke[i] else {
                        continue;
                    };
                    calib.push(<Centroids<K> as PushPull<SID>>::push_pull(
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
                    n_cols: None,
                })
            }
            CalibrationMode::Modes { n_mode, stroke, .. } => {
                let gmt = builder.clone().gmt.n_mode::<M>(n_mode);
                let mut optical_model = builder.gmt(gmt).build()?;
                optical_model.gmt.keep(&[SID as i32]);
                centroids.setup(&mut optical_model);

                println!(
                    "Calibrating {:} modes {:?} with centroids for segment {SID}...",
                    <Gmt as GmtMirror<M>>::to_string(&optical_model.gmt),
                    calib_mode.mode_range()
                );

                let mut a = vec![0f64; n_mode];
                let mut calib = vec![];
                // let cmd_fn = |gmt, sid, cmd| {
                //     <Gmt as GmtMirror<M>>::as_mut(gmt).set_segment_modes(sid, cmd);
                // };
                let now = Instant::now();
                for i in calib_mode.range() {
                    calib.push(<Centroids<K> as PushPull<SID>>::push_pull(
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
                    n_cols: None,
                })
            }
        }
    }
}

impl<K: CentroidKind, M: GmtMx> Calibrate<M> for Centroids<K>
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
    Centroids<K>: ValidCentroids,
{
    type Sensor = Imaging;
    /*
       fn calibrate(
           optical_model: OpticalModelBuilder<SensorBuilder<M, Self>>,
           calib_mode: CalibrationMode,
       ) -> super::Result<Reconstructor> {
           let om = optical_model.clone();
           let cm = calib_mode.clone();
           let c1 = thread::spawn(move || <Centroids<K> as CalibrateSegment<M, 1>>::calibrate(om, cm));
           let om = optical_model.clone();
           let cm = calib_mode.clone();
           let c2 = thread::spawn(move || <Centroids<K> as CalibrateSegment<M, 2>>::calibrate(om, cm));
           let om = optical_model.clone();
           let cm = calib_mode.clone();
           let c3 = thread::spawn(move || <Centroids<K> as CalibrateSegment<M, 3>>::calibrate(om, cm));
           let om = optical_model.clone();
           let cm = calib_mode.clone();
           let c4 = thread::spawn(move || <Centroids<K> as CalibrateSegment<M, 4>>::calibrate(om, cm));
           let om = optical_model.clone();
           let cm = calib_mode.clone();
           let c5 = thread::spawn(move || <Centroids<K> as CalibrateSegment<M, 5>>::calibrate(om, cm));
           let om = optical_model.clone();
           let cm = calib_mode.clone();
           let c6 = thread::spawn(move || <Centroids<K> as CalibrateSegment<M, 6>>::calibrate(om, cm));
           let om = optical_model.clone();
           let cm = calib_mode.clone();
           let c7 = thread::spawn(move || <Centroids<K> as CalibrateSegment<M, 7>>::calibrate(om, cm));
           // let c2 =
           //     <Centroids as CalibrateSegment<M, 2>>::calibrate(optical_model.clone(), calib_mode)?;
           // let c3 =
           //     <Centroids as CalibrateSegment<M, 3>>::calibrate(optical_model.clone(), calib_mode)?;
           // let c4 =
           //     <Centroids as CalibrateSegment<M, 4>>::calibrate(optical_model.clone(), calib_mode)?;
           // let c5 =
           //     <Centroids as CalibrateSegment<M, 5>>::calibrate(optical_model.clone(), calib_mode)?;
           // let c6 =
           //     <Centroids as CalibrateSegment<M, 6>>::calibrate(optical_model.clone(), calib_mode)?;
           // let c7 =
           //     <Centroids as CalibrateSegment<M, 7>>::calibrate(optical_model.clone(), calib_mode)?;
           let mut ci = vec![];
           for c in [c1, c2, c3, c4, c5, c6, c7] {
               ci.push(c.join().unwrap().unwrap());
           }
           Ok(Reconstructor::new(ci))
       }
    */
}
