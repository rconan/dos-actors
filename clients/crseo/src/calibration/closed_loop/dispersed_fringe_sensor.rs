use crate::calibration::{CalibProps, Modality};
use std::sync::Arc;

use crseo::{
    FromBuilder,
    gmt::{GmtM1, GmtM2},
};
use faer::ColRef;
use gmt_dos_clients_io::{
    gmt_m1::segment::{ModeShapes, RBM},
    gmt_m2::asm::{M2ASMAsmCommand, segment::AsmCommand},
    optics::{
        Dev, Wavefront, WfeRms,
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
    },
};
use interface::{Read, UniqueIdentifier, Update, Write};

use crate::{
    DeviceInitialize, DispersedFringeSensorProcessing, OpticalModel, OpticalModelBuilder,
    calibration::{
        CalibrateAssembly, CalibrationError, CalibrationSegment, ClosedLoopCalib, Reconstructor,
        algebra::Collapse, estimation::closed_loop::ClosedLoopEstimation,
    },
    sensors::{DispersedFringeSensor, NoSensor, WaveSensor},
};

use super::{
    Calib, CalibrationMode, ClosedLoopCalibrateSegment, ClosedLoopCalibration, ClosedLoopPushPull,
    SegmentClosedLoopSensorBuilder, SegmentSensorBuilder,
};

impl<const SID: u8> ClosedLoopPushPull<SID> for DispersedFringeSensorProcessing {
    type Sensor = DispersedFringeSensor<1, 1>;

    fn push_pull(
        &mut self,
        om: &mut crate::OpticalModel<<Self as ClosedLoopPushPull<SID>>::Sensor>,
        s: f64,
        cmd: &[f64],
        calib_mode: &CalibrationMode,
        c: ColRef<'_, f64>,
    ) -> Arc<Vec<f64>> {
        match calib_mode {
            CalibrationMode::RBM(_) => {
                <OpticalModel<Self::Sensor> as Read<RBM<SID>>>::read(om, cmd.to_vec().into());
            }
            CalibrationMode::Modes { .. } => {
                <OpticalModel<Self::Sensor> as Read<ModeShapes<SID>>>::read(
                    om,
                    cmd.to_vec().into(),
                );
            }
            _ => unimplemented!(),
        }
        let m2_cmd = c.iter().map(|x| x * -s).collect::<Vec<_>>();
        <OpticalModel<Self::Sensor> as Read<AsmCommand<SID>>>::read(om, m2_cmd.into());
        om.update();
        <OpticalModel<Self::Sensor> as Write<DfsFftFrame<Dev>>>::write(om).map(|data| {
            <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(self, data)
        });
        self.update();
        <DispersedFringeSensorProcessing as Write<Intercepts>>::write(self)
            .unwrap()
            .into_arc()
    }
}

impl<W: FromBuilder, const SID: u8> ClosedLoopCalibrateSegment<GmtM1, W, SID>
    for DispersedFringeSensorProcessing
where
    W: CalibrationSegment<GmtM2, SID, Sensor = W> + CalibrationSegment<GmtM1, SID, Sensor = W>,
    <W as FromBuilder>::ComponentBuilder: Clone,
{
    type Sensor = DispersedFringeSensor<1, 1>;

    fn calibrate(
        optical_model: OpticalModelBuilder<SegmentSensorBuilder<GmtM1, Self, W, SID>>,
        calib_mode: super::CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<SegmentClosedLoopSensorBuilder<W>>,
        closed_loop_calib_mode: super::CalibrationMode,
    ) -> super::Result<ClosedLoopCalib> {
        let mut m2_to_closed_loop_sensor: Reconstructor =
            <W as CalibrationSegment<GmtM2, SID>>::calibrate(
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )?
            .into();

        let mut m1_to_closed_loop_sensor: Reconstructor =
            <W as CalibrationSegment<GmtM1, SID>>::calibrate(
                closed_loop_optical_model,
                calib_mode.clone(),
            )?
            .into();

        m1_to_closed_loop_sensor.match_areas(&mut m2_to_closed_loop_sensor);
        m2_to_closed_loop_sensor.pseudoinverse();

        // println!("{m2_to_closed_loop_sensor}");
        // println!("{m1_to_closed_loop_sensor}");
        // print!("M1->M2 computation...");
        // let now = Instant::now();
        let m1_to_m2 = m2_to_closed_loop_sensor.pinv().next().unwrap()
            * &m1_to_closed_loop_sensor.calib_slice()[0];
        // println!(
        //     " ({},{}) in {:.3?}",
        //     m1_to_m2.nrows(),
        //     m1_to_m2.ncols(),
        //     now.elapsed()
        // );

        let mut om = optical_model.clone().build()?;

        let mut dfs_processor = DispersedFringeSensorProcessing::new();
        optical_model.initialize(&mut dfs_processor);

        let mut y = vec![];

        for (c, (m1_stroke, mut m1_cmd)) in m1_to_m2.col_iter().zip(calib_mode.stroke_command()) {
            /*             match calib_mode {
                CalibrationMode::RBM(_) => {
                    <OpticalModel<Sensor<Self, W, SID>> as Read<RBM<SID>>>::read(
                        &mut om,
                        m1_cmd.clone().into(),
                    );
                }
                CalibrationMode::Modes { .. } => {
                    <OpticalModel<Sensor<Self, W, SID>> as Read<ModeShapes<SID>>>::read(
                        &mut om,
                        m1_cmd.clone().into(),
                    );
                }
                _ => unimplemented!(),
            }
            let m2_cmd = c.iter().map(|x| x * -m1_stroke).collect::<Vec<_>>();
            <OpticalModel<Sensor<Self, W, SID>> as Read<AsmCommand<SID>>>::read(
                &mut om,
                m2_cmd.into(),
            );
            om.update();
            <OpticalModel<Sensor<Self, W, SID>> as Write<DfsFftFrame<Dev>>>::write(&mut om).map(
                |data| {
                    <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                        &mut dfs_processor,
                        data,
                    )
                },
            );
            dfs_processor.update();
            let push =
                <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
                    .unwrap(); */

            let push = <DispersedFringeSensorProcessing as ClosedLoopPushPull<SID>>::push_pull(
                &mut dfs_processor,
                &mut om,
                m1_stroke,
                &mut m1_cmd,
                &calib_mode,
                c,
            );

            m1_cmd.iter_mut().for_each(|x| *x *= -1.);
            /*             match calib_mode {
                CalibrationMode::RBM(_) => {
                    <OpticalModel<Sensor<Self, W, SID>> as Read<RBM<SID>>>::read(
                        &mut om,
                        m1_cmd.into(),
                    );
                }
                CalibrationMode::Modes { .. } => {
                    <OpticalModel<Sensor<Self, W, SID>> as Read<ModeShapes<SID>>>::read(
                        &mut om,
                        m1_cmd.into(),
                    );
                }
                _ => unimplemented!(),
            }
            let m2_cmd = c.iter().map(|x| x * m1_stroke).collect::<Vec<_>>();
            <OpticalModel<Sensor<Self, W, SID>> as Read<AsmCommand<SID>>>::read(
                &mut om,
                m2_cmd.into(),
            );
            om.update();
            <OpticalModel<Sensor<Self, W, SID>> as Write<DfsFftFrame<Dev>>>::write(&mut om).map(
                |data| {
                    <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                        &mut dfs_processor,
                        data,
                    )
                },
            );
            dfs_processor.update();
            let pull =
                <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
                    .unwrap(); */

            let pull = <DispersedFringeSensorProcessing as ClosedLoopPushPull<SID>>::push_pull(
                &mut dfs_processor,
                &mut om,
                -m1_stroke,
                &mut m1_cmd,
                &calib_mode,
                c,
            );
            y.extend(
                push.iter()
                    .zip(pull.iter())
                    .map(|(&x, &y)| 0.5 * (x - y) / m1_stroke),
            );
        }
        let n = dfs_processor.intercept.len();
        let m1_closed_loop_to_sensor = Calib::builder()
            .sid(SID)
            .c(y)
            .n_mode(calib_mode.n_mode())
            .mode(calib_mode)
            .mask(vec![true; n])
            .build();
        Ok(ClosedLoopCalib {
            m1_to_closed_loop_sensor,
            m2_to_closed_loop_sensor,
            m1_to_m2,
            m1_to_sensor: None,
            m2_to_sensor: None,
            m1_closed_loop_to_sensor,
        })
    }
}

impl<W: FromBuilder> ClosedLoopCalibration<GmtM1, W> for DispersedFringeSensorProcessing
where
    <W as FromBuilder>::ComponentBuilder: Clone,
    W: CalibrateAssembly<GmtM2, W> + CalibrateAssembly<GmtM1, W>,
{
    type Sensor = DispersedFringeSensor<1, 1>;
}

/* impl<W: FromBuilder> ClosedLoopCalibrate<W> for DispersedFringeSensorProcessing
where
    <W as FromBuilder>::ComponentBuilder: Clone,
    W: CalibrateAssembly<GmtM2, W> + CalibrateAssembly<GmtM1, W>,
{
    type Sensor = DispersedFringeSensor<1, 1>;
    fn calibrate(
        optical_model: &OpticalModelBuilder<super::SensorBuilder<Self, W>>,
        mirror_mode: impl Into<MirrorMode>,
        closed_loop_optical_model: &OpticalModelBuilder<ClosedLoopSensorBuilder<W>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> crate::calibration::Result<Reconstructor<CalibrationMode, ClosedLoopCalib>>
    where
        <<Self as ClosedLoopCalibrate<W>>::Sensor as crseo::FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
    {
        let mut mode_iter = Into::<MirrorMode>::into(mirror_mode).into_iter();

        let h1 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<W, 1>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h2 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<W, 2>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h3 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<W, 3>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h4 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<W, 4>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h5 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<W, 5>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h6 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<W, 6>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        });
        let h7 = mode_iter.next().unwrap().map(|calib_mode| {
            <Self as ClosedLoopCalibrateSegment<W, 7>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )
        }); // let mut ci = vec![];
            // for c in [c1, c2, c3, c4, c5, c6, c7] {
            //     ci.push(c.join().unwrap().unwrap());
            // }
            // ci
        let mat_ci: crate::calibration::Result<Vec<_>> = [h1, h2, h3, h4, h5, h6, h7]
            .into_iter()
            .filter_map(|h| h)
            .collect();
        // let c1 = <Self as CalibrateSegment<M, 1>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c2 = <Self as CalibrateSegment<M, 2>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c3 = <Self as CalibrateSegment<M, 3>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c4 = <Self as CalibrateSegment<M, 4>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c5 = <Self as CalibrateSegment<M, 5>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c6 = <Self as CalibrateSegment<M, 6>>::calibrate(optical_model.clone(), calib_mode)?;
        // let c7 = <Self as CalibrateSegment<M, 7>>::calibrate(optical_model.clone(), calib_mode)?;
        // let ci = vec![c1, c2, c3, c4, c5, c6, c7];
        Ok(Reconstructor::<CalibrationMode, ClosedLoopCalib>::new(
            mat_ci?,
        ))
        // mat_ci.map(|(mat, calib)| (mat, Reconstructor::new(calib)))
    }
}
 */
/* impl<W: FromBuilder> ClosedLoopCalibrate<W> for DispersedFringeSensorProcessing
where
    W: CalibrateAssembly<GmtM2, W> + CalibrateAssembly<GmtM1, W>,
    <W as FromBuilder>::ComponentBuilder: Clone,
{
    type Sensor = DispersedFringeSensor<1, 1>;

    fn calibrate(
        optical_model: OpticalModelBuilder<super::SensorBuilder<Self, W>>,
        calib_mode: CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<ClosedLoopSensorBuilder<W>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> crate::calibration::Result<Reconstructor<CalibrationMode, ClosedLoopCalib>>
    where
        <<Self as ClosedLoopCalibrate<W>>::Sensor as crseo::FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
    {
        let mat_ci: crate::calibration::Result<Vec<_>> = {
            let h1 = <Self as ClosedLoopCalibrateSegment<W, 1>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h2 = <Self as ClosedLoopCalibrateSegment<W, 2>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h3 = <Self as ClosedLoopCalibrateSegment<W, 3>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h4 = <Self as ClosedLoopCalibrateSegment<W, 4>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h5 = <Self as ClosedLoopCalibrateSegment<W, 5>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h6 = <Self as ClosedLoopCalibrateSegment<W, 6>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );

            [h1, h2, h3, h4, h5, h6].into_iter().collect()
        };

        mat_ci.map(|calibs| {
            // let mut calib = calibs[0].clone();
            // c1.c.iter()
            //     .chain(c2.c.iter())
            //     .chain(c3.c.iter())
            //     .chain(c4.c.iter())
            //     .chain(c5.c.iter())
            //     .chain(c6.c.iter())
            //     .map(|x| *x);
            // let mut calib = c1.clone();
            // calib.m1_closed_loop_to_sensor.sid = 0;
            // calib.m1_closed_loop_to_sensor.n_cols = Some(calib_mode.calibration_n_mode() * 6);
            // calib.m1_closed_loop_to_sensor.mask =
            //     vec![true; calib.m1_closed_loop_to_sensor.mask.len()];
            // calib.m1_closed_loop_to_sensor.c = calibs
            //     .into_iter()
            //     .flat_map(|c| c.m1_closed_loop_to_sensor.c)
            //     .collect();
            Reconstructor::<CalibrationMode, ClosedLoopCalib>::new(calibs)
        })
    }
}
 */
impl<U> ClosedLoopEstimation<WaveSensor, U> for DispersedFringeSensorProcessing
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    OpticalModel: Read<U>,
    OpticalModel<WaveSensor>: Read<U>,
    OpticalModel<DispersedFringeSensor>: Read<U>,
{
    type Sensor = DispersedFringeSensor;

    /// Applies the command to the [OpticalModel] and estimates it using the [Reconstructor]
    /// after applying a correction with the closed-loop [OpticalModel]
    fn estimate_with_closed_loop_reconstructor<M, C>(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        closed_loop_optical_model: &OpticalModelBuilder<
            <WaveSensor as FromBuilder>::ComponentBuilder,
        >,
        recon: &mut Reconstructor<M, C>,
        cmd: &[f64],
        m2_to_closed_loop_sensor: &mut Reconstructor,
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Default + Send + Sync,
        C: CalibProps<M> + Default + Send + Sync + Clone,
        // Reconstructor<CalibrationMode, C>: Collapse,
    {
        let mut dfs_processor = <Self as ClosedLoopEstimation<WaveSensor, U>>::processor(
            optical_model,
            closed_loop_optical_model,
            cmd,
            m2_to_closed_loop_sensor,
        )?;

        // let mut recon = recon.clone().collapse();
        // recon.pseudoinverse();

        <DispersedFringeSensorProcessing as ClosedLoopEstimation<WaveSensor, U>>::recon(
            &mut dfs_processor,
            recon,
        )
    }
    fn recon<M, C>(
        &mut self,
        recon: &mut Reconstructor<M, C>,
    ) -> std::result::Result<std::sync::Arc<Vec<f64>>, CalibrationError>
    where
        M: Modality + Default + Send + Sync,
        C: CalibProps<M> + Default + Send + Sync + Clone,
    {
        <DispersedFringeSensorProcessing as Write<Intercepts>>::write(self)
            .map(|data| <Reconstructor<M, C> as Read<Intercepts>>::read(recon, data));
        recon.update();
        Ok(recon.estimate.clone())
    }
    fn processor(
        optical_model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
        closed_loop_optical_model: &OpticalModelBuilder<
            <WaveSensor as FromBuilder>::ComponentBuilder,
        >,
        cmd: &[f64],
        m2_to_closed_loop_sensor: &mut Reconstructor,
    ) -> std::result::Result<Self, CalibrationError>
    where
        Reconstructor<CalibrationMode, ClosedLoopCalib<CalibrationMode>>: Collapse,
    {
        let mut com = closed_loop_optical_model.clone().build()?;
        <OpticalModel<_> as Read<U>>::read(&mut com, cmd.into());
        com.update();
        <OpticalModel<_> as Write<Wavefront>>::write(&mut com)
            .map(|cmd| <Reconstructor as Read<Wavefront>>::read(m2_to_closed_loop_sensor, cmd));
        m2_to_closed_loop_sensor.update();

        let m2_command: Vec<_> =
            <Reconstructor as Write<M2ASMAsmCommand>>::write(m2_to_closed_loop_sensor)
                .unwrap()
                .into_arc()
                .iter()
                .map(|x| -*x)
                .collect();

        let mut onaxis_om = OpticalModel::<NoSensor>::builder()
            .gmt(optical_model.gmt.clone())
            .build()?;

        <OpticalModel as Read<U>>::read(&mut onaxis_om, cmd.into());
        onaxis_om.update();
        let before = <OpticalModel as Write<WfeRms<-9>>>::write(&mut onaxis_om)
            .unwrap()
            .into_arc();
        <OpticalModel as Read<M2ASMAsmCommand>>::read(&mut onaxis_om, m2_command.clone().into());
        onaxis_om.update();
        let after = <OpticalModel as Write<WfeRms<-9>>>::write(&mut onaxis_om)
            .unwrap()
            .into_arc();
        println!(
            "On-axis correction, WFE RMS: {:.0}nm -> {:.0}nm",
            before[0], after[0]
        );

        let mut dfs_processor = DispersedFringeSensorProcessing::new();
        optical_model.initialize(&mut dfs_processor);
        let mut om = optical_model.clone().build()?;

        <OpticalModel<_> as Read<U>>::read(&mut om, cmd.into());
        <OpticalModel<_> as Read<M2ASMAsmCommand>>::read(&mut om, m2_command.into());
        om.update();
        <OpticalModel<_> as Write<DfsFftFrame<Dev>>>::write(&mut om).map(|data| {
            <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                &mut dfs_processor,
                data,
            )
        });
        dfs_processor.update();
        Ok(dfs_processor)
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{FromBuilder, Gmt, Source};
    use gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions;
    use skyangle::Conversion;

    use crate::{
        calibration::{Calibration, MirrorMode},
        sensors::WaveSensor,
    };

    use super::*;

    type DFS = DispersedFringeSensor<1, 1>;

    #[test]
    fn closed_loop_segment_calibrate() -> Result<(), Box<dyn Error>> {
        let m2_n_mode = 15;
        let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
        let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());
        let optical_model = OpticalModel::<DFS>::builder()
            .gmt(gmt)
            .source(agws_gs.clone())
            .sensor(DFS::builder().source(agws_gs.clone()));
        let calib = <DispersedFringeSensorProcessing as ClosedLoopCalibrateSegment<
            GmtM1,
            WaveSensor,
            1,
        >>::calibrate(
            optical_model.clone(),
            CalibrationMode::RBM([
                None,                     // Tx
                None,                     // Ty
                None,                     // Tz
                Some(1f64.from_arcsec()), // Rx
                Some(1f64.from_arcsec()), // Ry
                None,                     // Rz
            ]),
            closed_loop_optical_model,
            CalibrationMode::modes(m2_n_mode, 1e-6),
        )?;
        println!("{calib}");

        /* let mut dfs_processor = DispersedFringeSensorProcessing::new();
        optical_model.initialize(&mut dfs_processor);
        let mut dfs_om = optical_model.build()?;

        let mut m1_rbm = vec![0f64; 6];
        m1_rbm[3] = 1f64.from_arcsec();
        let cmd =
            calib.m1_to_m2() * -faer::mat::from_column_major_slice::<f64>(&m1_rbm[3..5], 2, 1);

        <OpticalModel<DFS> as Read<RBM<1>>>::read(&mut dfs_om, m1_rbm.into());
        <OpticalModel<DFS> as Read<AsmCommand<1>>>::read(
            &mut dfs_om,
            cmd.col_as_slice(0).to_vec().into(),
        );

        dfs_om.update();

        <OpticalModel<DFS> as Write<DfsFftFrame<Dev>>>::write(&mut dfs_om).map(|data| {
            <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                &mut dfs_processor,
                data,
            )
        });
        dfs_processor.update();
        let y = <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
            .unwrap()
            .into_arc();

        let tt = calib.pseudoinverse() * faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1);
        tt.col_as_slice(0)
            .iter()
            .take(1)
            .for_each(|x| assert!((dbg!(x.to_mas()) - 1000.).abs() < 1f64)); */

        Ok(())
    }

    #[test]
    fn closed_loop_calibrate() -> Result<(), Box<dyn Error>> {
        let m2_n_mode = 66;
        let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
        let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
        let optical_model = OpticalModel::<DFS>::builder()
            .gmt(gmt.clone())
            .source(agws_gs.clone())
            .sensor(DFS::builder().source(agws_gs.clone().band("J")));
        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());

        let mut recon = <DispersedFringeSensorProcessing as ClosedLoopCalibration<
            GmtM1,
            WaveSensor,
        >>::calibrate_serial(
            &optical_model,
            MirrorMode::from(CalibrationMode::RBM([
                None,                    // Tx
                None,                    // Ty
                None,                    // Tz
                Some(100f64.from_mas()), // Rx
                Some(100f64.from_mas()), // Ry
                None,                    // Rz
            ]))
            .update((7, CalibrationMode::empty_rbm())),
            &closed_loop_optical_model,
            CalibrationMode::modes(m2_n_mode, 1e-6),
        )?;
        recon.pseudoinverse();
        println!("{recon}");

        let mut data = vec![0.; 42];
        data[3] = 100f64.from_mas();
        data[6 * 1 + 4] = 100f64.from_mas();
        let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
            WaveSensor,
            M1RigidBodyMotions,
        >>::estimate(
            &optical_model,
            &closed_loop_optical_model,
            &mut recon,
            &data,
        )?;
        estimate
            .chunks(6)
            .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

        Ok(())
    }

    #[test]
    fn closed_loop_calibrate_7() -> Result<(), Box<dyn Error>> {
        let m2_n_mode = 66;
        let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
        let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
        let optical_model = OpticalModel::<DFS>::builder()
            .gmt(gmt.clone())
            .source(agws_gs.clone())
            .sensor(DFS::builder().source(agws_gs.clone().band("J")));
        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());

        let closed_loop_calib_mode = CalibrationMode::modes(m2_n_mode, 1e-6);
        let mut m2_to_closed_loop_sensor: Reconstructor =
            <WaveSensor as Calibration<GmtM2>>::calibrate(
                &closed_loop_optical_model,
                closed_loop_calib_mode.clone(),
            )?;
        m2_to_closed_loop_sensor.pseudoinverse();

        let mut recon = <DispersedFringeSensorProcessing as ClosedLoopCalibration<
            GmtM1,
            WaveSensor,
        >>::calibrate_serial(
            &optical_model,
            MirrorMode::from(CalibrationMode::RBM([
                None,                    // Tx
                None,                    // Ty
                None,                    // Tz
                Some(100f64.from_mas()), // Rx
                Some(100f64.from_mas()), // Ry
                None,                    // Rz
            ]))
            .update((7, CalibrationMode::empty_rbm())),
            &closed_loop_optical_model,
            closed_loop_calib_mode,
        )?;
        recon.pseudoinverse();
        println!("{recon}");

        let mut data = vec![0.; 42];
        data[36 + 3] = 100f64.from_mas();
        let estimate = <DispersedFringeSensorProcessing as ClosedLoopEstimation<
            WaveSensor,
            M1RigidBodyMotions,
        >>::estimate_with_closed_loop_reconstructor(
            &optical_model,
            &closed_loop_optical_model,
            &mut recon,
            &data,
            &mut m2_to_closed_loop_sensor,
        )?;
        estimate
            .chunks(6)
            .map(|c| c.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!("S{}: {:+6.0?}", i + 1, x));

        Ok(())
    }
}
