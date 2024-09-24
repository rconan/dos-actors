use std::time::Instant;

use crseo::gmt::{GmtM1, GmtM2};
use faer::Mat;
use gmt_dos_clients_io::{
    gmt_m1::segment::{BendingModes, RBM},
    gmt_m2::asm::segment::AsmCommand,
    optics::{
        dispersed_fringe_sensor::{DfsFftFrame, Intercepts},
        Dev,
    },
};
use interface::{Read, Update, Write};

use crate::{
    calibration::{closed_loop::Sensor, CalibrateSegment},
    sensors::{DispersedFringeSensor, DispersedFringeSensorProcessing, WaveSensor},
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
};

use super::{
    Calib, CalibrationMode, ClosedLoopCalibrate, ClosedLoopCalibrateSegment,
    SegmentClosedLoopSensorBuilder, SegmentSensorBuilder,
};

impl<const SID: u8> ClosedLoopCalibrateSegment<SID> for DispersedFringeSensorProcessing {
    type Sensor = DispersedFringeSensor<1, 1>;
    type ClosedLoopSensor = WaveSensor;

    fn calibrate(
        mut optical_model: OpticalModelBuilder<SegmentSensorBuilder<Self, SID>>,
        calib_mode: super::CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<SegmentClosedLoopSensorBuilder<Self, SID>>,
        closed_loop_calib_mode: super::CalibrationMode,
    ) -> super::Result<(Mat<f64>, Calib)> {
        let mut calib_m2_modes_onaxis =
            <Self::ClosedLoopSensor as CalibrateSegment<GmtM2, SID>>::calibrate(
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode,
            )?;
        let mut calib_m1_modes_onaxis =
            <Self::ClosedLoopSensor as CalibrateSegment<GmtM1, SID>>::calibrate(
                closed_loop_optical_model,
                calib_mode,
            )?;
        calib_m1_modes_onaxis.match_areas(&mut calib_m2_modes_onaxis);
        println!("{calib_m2_modes_onaxis}");
        println!("{calib_m1_modes_onaxis}");
        print!("M1->M2 computation...");
        let now = Instant::now();
        let m1_to_m2 = calib_m2_modes_onaxis.pseudoinverse() * calib_m1_modes_onaxis;
        println!(
            " ({},{}) in {:.3?}",
            m1_to_m2.nrows(),
            m1_to_m2.ncols(),
            now.elapsed()
        );

        let mut om = optical_model.clone().build()?;

        let mut dfs_processor = DispersedFringeSensorProcessing::new();
        optical_model.initialize(&mut dfs_processor);

        let mut y = vec![];

        for (c, (m1_stroke, m1_cmd)) in m1_to_m2.col_iter().zip(calib_mode.stroke_command()) {
            match calib_mode {
                CalibrationMode::RBM(_) => {
                    <OpticalModel<Sensor<Self, SID>> as Read<RBM<SID>>>::read(
                        &mut om,
                        m1_cmd.into(),
                    );
                }
                CalibrationMode::Modes { .. } => {
                    <OpticalModel<Sensor<Self, SID>> as Read<BendingModes<SID>>>::read(
                        &mut om,
                        m1_cmd.into(),
                    );
                }
            }
            let m2_cmd = c.iter().map(|x| x * -m1_stroke).collect::<Vec<_>>();
            <OpticalModel<Sensor<Self, SID>> as Read<AsmCommand<SID>>>::read(
                &mut om,
                m2_cmd.into(),
            );
            om.update();
            <OpticalModel<Sensor<Self, SID>> as Write<DfsFftFrame<Dev>>>::write(&mut om).map(
                |data| {
                    <DispersedFringeSensorProcessing as Read<DfsFftFrame<Dev>>>::read(
                        &mut dfs_processor,
                        data,
                    )
                },
            );
            dfs_processor.update();
            let data =
                <DispersedFringeSensorProcessing as Write<Intercepts>>::write(&mut dfs_processor)
                    .unwrap();

            y.extend(data.into_arc().iter().map(|x| x / m1_stroke));
        }
        let n = dfs_processor.intercept.len();
        Ok((
            m1_to_m2,
            Calib::builder()
                .c(y)
                .n_mode(calib_mode.n_mode())
                .mode(calib_mode)
                .mask(vec![true; n])
                .build(),
        ))
    }
}

impl ClosedLoopCalibrate for DispersedFringeSensorProcessing {
    type Sensor = DispersedFringeSensor<1, 1>;
    type ClosedLoopSensor = WaveSensor;

    fn calibrate(
        optical_model: OpticalModelBuilder<super::SensorBuilder<Self>>,
        calib_mode: CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<super::ClosedLoopSensorBuilder<Self>>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> crate::calibration::Result<(Vec<Mat<f64>>, crate::calibration::Reconstructor)>
    where
        <<Self as ClosedLoopCalibrate>::Sensor as crseo::FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
        <<Self as ClosedLoopCalibrate>::ClosedLoopSensor as crseo::FromBuilder>::ComponentBuilder:
            Clone + Send + Sync,
    {
        let mat_ci: crate::calibration::Result<(Vec<_>, Vec<_>)> = {
            let h1 = <Self as ClosedLoopCalibrateSegment<1>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h2 = <Self as ClosedLoopCalibrateSegment<2>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h3 = <Self as ClosedLoopCalibrateSegment<3>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h4 = <Self as ClosedLoopCalibrateSegment<4>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h5 = <Self as ClosedLoopCalibrateSegment<5>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );
            let h6 = <Self as ClosedLoopCalibrateSegment<6>>::calibrate(
                optical_model.clone(),
                calib_mode.clone(),
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            );

            [h1, h2, h3, h4, h5, h6].into_iter().collect()
        };

        mat_ci.map(|(mat, calibs)| {
            let mut calib = calibs[0].clone();
            // c1.c.iter()
            //     .chain(c2.c.iter())
            //     .chain(c3.c.iter())
            //     .chain(c4.c.iter())
            //     .chain(c5.c.iter())
            //     .chain(c6.c.iter())
            //     .map(|x| *x);
            // let mut calib = c1.clone();
            calib.sid = 0;
            calib.n_cols = Some(calib_mode.calibration_n_mode() * 6);
            calib.mask = vec![true; calib.mask.len()];
            calib.c = calibs.into_iter().flat_map(|c| c.c).collect();
            (mat, crate::calibration::Reconstructor::new(vec![calib]))
        })
    }
}
#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{FromBuilder, Gmt, Source};
    use gmt_dos_clients_io::{
        gmt_m1::M1RigidBodyMotions, gmt_m2::asm::M2ASMAsmCommand, optics::WfeRms,
    };
    use skyangle::Conversion;

    use crate::sensors::NoSensor;

    use super::*;

    type DFS = DispersedFringeSensor<1, 1>;

    #[test]
    fn closed_loop_segment_calibrate() -> Result<(), Box<dyn Error>> {
        let m2_n_mode = 15;
        let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
        let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());
        let mut optical_model = OpticalModel::<DFS>::builder()
            .gmt(gmt)
            .source(agws_gs.clone())
            .sensor(DFS::builder().source(agws_gs.clone()));
        let (m1_to_m2, calib) =
            <DispersedFringeSensorProcessing as ClosedLoopCalibrateSegment<1>>::calibrate(
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

        let mut dfs_processor = DispersedFringeSensorProcessing::new();
        optical_model.initialize(&mut dfs_processor);
        let mut dfs_om = optical_model.build()?;

        let mut m1_rbm = vec![0f64; 6];
        m1_rbm[3] = 1f64.from_arcsec();
        let cmd = m1_to_m2 * -faer::mat::from_column_major_slice::<f64>(&m1_rbm[3..5], 2, 1);

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
            .for_each(|x| assert!((dbg!(x.to_mas()) - 1000.).abs() < 1f64));

        Ok(())
    }

    #[test]
    fn closed_loop_calibrate() -> Result<(), Box<dyn Error>> {
        let m2_n_mode = 66;
        let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
        let gmt = Gmt::builder().m2("Karhunen-Loeve", m2_n_mode);
        let mut optical_model = OpticalModel::<DFS>::builder()
            .gmt(gmt.clone())
            .source(agws_gs.clone())
            .sensor(DFS::builder().source(agws_gs.clone()).nyquist_factor(3.));
        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());
        let (m1_to_m2, mut recon) =
            <DispersedFringeSensorProcessing as ClosedLoopCalibrate>::calibrate(
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
        recon.pseudoinverse();
        println!("{recon}");

        let mut dfs_processor = DispersedFringeSensorProcessing::new();
        optical_model.initialize(&mut dfs_processor);
        let mut dfs_om = optical_model.build()?;

        let mut m1_rxy = vec![vec![0f64; 2]; 7];
        m1_rxy[0][0] = 1f64.from_arcsec();
        m1_rxy[1][1] = 1f64.from_arcsec();
        let cmd: Vec<_> = m1_to_m2
            .iter()
            .zip(&m1_rxy)
            .map(|(m1_to_m2, m1_rxy)| {
                m1_to_m2 * -faer::mat::from_column_major_slice::<f64>(m1_rxy, 2, 1)
            })
            .flat_map(|m| m.col_as_slice(0).to_vec())
            .chain(vec![0.; m2_n_mode])
            .collect();

        let m1_rbm: Vec<f64> = m1_rxy
            .into_iter()
            .flat_map(|rxy| {
                vec![0.; 3]
                    .into_iter()
                    .chain(rxy.into_iter())
                    .chain(Some(0.))
                    .collect::<Vec<_>>()
            })
            .collect();

        let mut om = OpticalModel::<NoSensor>::builder()
            .gmt(gmt.clone())
            .build()?;
        dbg!(&cmd[..10]);

        <OpticalModel<NoSensor> as Read<M1RigidBodyMotions>>::read(&mut om, m1_rbm.clone().into());
        <OpticalModel<NoSensor> as Read<M2ASMAsmCommand>>::read(&mut om, cmd.clone().into());
        om.update();
        dbg!(<OpticalModel as Write<WfeRms<-9>>>::write(&mut om));

        println!("{:?}", cmd.len());
        <OpticalModel<DFS> as Read<M1RigidBodyMotions>>::read(&mut dfs_om, m1_rbm.into());
        <OpticalModel<DFS> as Read<M2ASMAsmCommand>>::read(&mut dfs_om, cmd.into());

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
        // dbg!(y.len());

        let tt = faer::mat::from_column_major_slice::<f64>(&y, y.len(), 1) / &recon;
        // dbg!(&tt);

        tt[0]
            .col_as_slice(0)
            .chunks(2)
            .map(|rxy| rxy.iter().map(|x| x.to_mas()).collect::<Vec<_>>())
            .for_each(|x| println!("{:+5.0?}", x));

        Ok(())
    }
}
