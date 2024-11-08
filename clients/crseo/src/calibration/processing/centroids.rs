use crate::{
    calibration::{
        Calib, Calibration, CalibrationMode, CalibrationSegment, PushPull, SegmentSensorBuilder,
    },
    centroiding::{CentroidKind, CentroidsProcessing, Full, ZeroMean},
    OpticalModel, OpticalModelBuilder,
};
use crseo::{
    gmt::{GmtBuilder, GmtMirror, GmtMirrorBuilder, GmtMx, MirrorGetSet},
    Gmt, Imaging,
};
use interface::Update;
use std::time::Instant;

pub trait ValidCentroids {
    fn get(&mut self) -> Vec<Vec<f32>>;
}
impl ValidCentroids for CentroidsProcessing<Full> {
    fn get(&mut self) -> Vec<Vec<f32>> {
        self.centroids
            .grab()
            .valids(Some(&self.reference.valid_lenslets))
    }
}
impl ValidCentroids for CentroidsProcessing<ZeroMean> {
    fn get(&mut self) -> Vec<Vec<f32>> {
        self.centroids
            .grab()
            .remove_mean(Some(&self.reference.valid_lenslets))
            .valids(Some(&self.reference.valid_lenslets))
    }
}

impl<K: CentroidKind, const SID: u8> PushPull<SID> for CentroidsProcessing<K>
where
    CentroidsProcessing<K>: ValidCentroids,
{
    type Sensor = Imaging;
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

impl<K, M, const SID: u8> CalibrationSegment<M, SID> for CentroidsProcessing<K>
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
    CentroidsProcessing<K>: ValidCentroids,
    K: CentroidKind,
    M: GmtMx,
{
    type Sensor = Imaging;

    fn calibrate(
        builder: OpticalModelBuilder<SegmentSensorBuilder<M, Self, SID>>,
        calib_mode: CalibrationMode,
    ) -> super::Result<Calib> {
        let mut centroids = CentroidsProcessing::try_from(builder.sensor.as_ref().unwrap())?;

        let mut optical_model = builder.build()?;
        println!(
            "Calibrating {:} modes {:?} with centroids for segment {SID}...",
            <Gmt as GmtMirror<M>>::to_string(&optical_model.gmt),
            calib_mode.mode_range()
        );

        match calib_mode {
            CalibrationMode::RBM(stroke) => {
                if <K as CentroidKind>::is_full() {
                    optical_model.gmt.keep(&[SID as i32]);
                }
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
                    calib.push(<CentroidsProcessing<K> as PushPull<SID>>::push_pull(
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
                let iter = centroids
                    .reference
                    .valid_lenslets
                    .chunks(centroids.centroids.n_lenslet_total)
                    .flat_map(|v| {
                        v.iter()
                            .map(|&v| v > 0)
                            .cycle()
                            .take(centroids.centroids.n_lenslet_total * 2)
                    });
                Ok(Calib {
                    sid: SID,
                    n_mode: 6,
                    c: calib.into_iter().flatten().collect(),
                    mask: iter.collect(),
                    mode: calib_mode,
                    runtime: now.elapsed(),
                    n_cols: None,
                })
            }
            CalibrationMode::Modes { n_mode, stroke, .. } => {
                if <K as CentroidKind>::is_full() {
                    optical_model.gmt.keep(&[SID as i32]);
                }
                centroids.setup(&mut optical_model);

                let mut a = vec![0f64; n_mode];
                let mut calib = vec![];
                // let cmd_fn = |gmt, sid, cmd| {
                //     <Gmt as GmtMirror<M>>::as_mut(gmt).set_segment_modes(sid, cmd);
                // };
                let now = Instant::now();
                for i in calib_mode.range() {
                    calib.push(<CentroidsProcessing<K> as PushPull<SID>>::push_pull(
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
                let iter = centroids
                    .reference
                    .valid_lenslets
                    .chunks(centroids.centroids.n_lenslet_total)
                    .flat_map(|v| {
                        v.iter()
                            .map(|&v| v > 0)
                            .cycle()
                            .take(centroids.centroids.n_lenslet_total * 2)
                    });
                Ok(Calib {
                    sid: SID,
                    n_mode,
                    c: calib.into_iter().flatten().collect(),
                    mask: iter.collect(),
                    mode: calib_mode,
                    runtime: now.elapsed(),
                    n_cols: None,
                })
            }
            _ => unimplemented!(),
        }
    }
}

impl<K: CentroidKind, M: GmtMx> Calibration<M> for CentroidsProcessing<K>
where
    Gmt: GmtMirror<M>,
    GmtBuilder: GmtMirrorBuilder<M>,
    CentroidsProcessing<K>: ValidCentroids,
{
    type Sensor = Imaging;
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{gmt::GmtM1, imaging::LensletArray, FromBuilder, Gmt, Source};
    use gmt_dos_clients_io::{
        gmt_m1::segment::BendingModes,
        optics::{Dev, Frame, SensorData},
    };
    use interface::{Read, Update, Write};
    use skyangle::Conversion;

    use crate::{
        calibration::algebra::CalibProps, sensors::Camera, DeviceInitialize, OpticalModel,
    };

    use super::*;

    #[test]
    fn centroids() -> Result<(), Box<dyn Error>> {
        let m1_n_mode = 6;
        let n_gs = 3;

        let agws_gs = Source::builder().size(n_gs).on_ring(6f32.from_arcmin());
        let sh48 = Camera::builder()
            .n_sensor(n_gs)
            .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32))
            .lenslet_flux(0.75);
        let mut sh48_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sh48)?;

        let gmt = Gmt::builder().m1("bending modes", m1_n_mode);

        let optical_model = OpticalModel::<Camera<1>>::builder()
            .gmt(gmt.clone())
            .source(agws_gs.clone())
            .sensor(sh48);

        optical_model.initialize(&mut sh48_centroids);
        dbg!(sh48_centroids.n_valid_lenslets());

        let calib = <CentroidsProcessing as CalibrationSegment<GmtM1, 1>>::calibrate(
            optical_model.clone().into(),
            CalibrationMode::modes(m1_n_mode, 1e-4),
        )?;
        println!("{calib}");
        let calib_pinv = calib.pseudoinverse().unwrap();
        dbg!(calib_pinv.cond());

        // sh48_centroids.valid_lenslets(&calib);

        let mut sh48_om = optical_model.build()?;
        println!("{sh48_om}");

        let mut m1_bm = vec![0f64; m1_n_mode];
        m1_bm[3] = 1e-4;

        <OpticalModel<Camera<1>> as Read<BendingModes<1>>>::read(
            &mut sh48_om,
            m1_bm.clone().into(),
        );

        sh48_om.update();

        <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(&mut sh48_om)
            .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut sh48_centroids, data));
        sh48_centroids.update();
        let y = <CentroidsProcessing as Write<SensorData>>::write(&mut sh48_centroids)
            .map(|data| {
                let s = data.as_arc();
                // serde_pickle::to_writer(
                //     &mut File::create("3gs-offaxis.pkl").unwrap(),
                //     &(s.as_ref(), &calib, sh48_centroids.get_valid_lenslets()),
                //     Default::default(),
                // )
                // .unwrap();
                calib.mask(&s)
            })
            .unwrap();
        dbg!(y.len());

        let m1_bm_e = &calib_pinv * y;
        println!("{:?}", m1_bm_e);

        assert!((m1_bm_e[3] - m1_bm[3]).abs() * 1e4 < 1e-3);

        Ok(())
    }
    #[test]
    fn zero_mean_centroids() -> Result<(), Box<dyn Error>> {
        let m1_n_mode = 6;
        let n_gs = 3;

        let agws_gs = Source::builder().size(n_gs).on_ring(6f32.from_arcmin());
        let sh48 = Camera::builder()
            .n_sensor(n_gs)
            .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32))
            .lenslet_flux(0.75);
        let mut sh48_centroids: CentroidsProcessing<ZeroMean> =
            CentroidsProcessing::try_from(&sh48)?;

        let gmt = Gmt::builder().m1("bending modes", m1_n_mode);

        let optical_model = OpticalModel::<Camera<1>>::builder()
            .gmt(gmt.clone())
            .source(agws_gs.clone())
            .sensor(sh48);

        optical_model.initialize(&mut sh48_centroids);
        dbg!(sh48_centroids.n_valid_lenslets());

        let calib = <CentroidsProcessing<ZeroMean> as CalibrationSegment<GmtM1, 1>>::calibrate(
            optical_model.clone().into(),
            CalibrationMode::modes(m1_n_mode, 1e-4),
        )?;
        println!("{calib}");
        let calib_pinv = calib.pseudoinverse().unwrap();
        dbg!(calib_pinv.cond());

        // sh48_centroids.valid_lenslets(&calib);

        let mut sh48_om = optical_model.build()?;
        println!("{sh48_om}");

        let mut m1_bm = vec![0f64; m1_n_mode];
        m1_bm[3] = 1e-4;

        <OpticalModel<Camera<1>> as Read<BendingModes<1>>>::read(
            &mut sh48_om,
            m1_bm.clone().into(),
        );

        sh48_om.update();

        <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(&mut sh48_om).map(|data| {
            <CentroidsProcessing<_> as Read<Frame<Dev>>>::read(&mut sh48_centroids, data)
        });
        sh48_centroids.update();
        let y = <CentroidsProcessing<_> as Write<SensorData>>::write(&mut sh48_centroids)
            .map(|data| {
                let s = data.as_arc();
                // serde_pickle::to_writer(
                //     &mut File::create("3gs-offaxis.pkl").unwrap(),
                //     &(s.as_ref(), &calib, sh48_centroids.get_valid_lenslets()),
                //     Default::default(),
                // )
                // .unwrap();
                calib.mask(&s)
            })
            .unwrap();
        dbg!(y.len());

        let m1_bm_e = &calib_pinv * y;
        println!("{:?}", m1_bm_e);

        assert!((m1_bm_e[3] - m1_bm[3]).abs() * 1e4 < 1e-3);

        Ok(())
    }
}
