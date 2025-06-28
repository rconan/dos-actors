use crseo::{
    FromBuilder, Imaging,
    gmt::{GmtM2, GmtMx},
};

use crate::{
    OpticalModelBuilder,
    calibration::{
        CalibrateAssembly, CalibrationMode, CalibrationSegment, Reconstructor,
        algebra::ClosedLoopCalib,
    },
    centroiding::{CentroidKind, CentroidsProcessing},
    sensors::{SegmentPistonSensor, WaveSensor},
};

use super::{ClosedLoopCalibrateSegment, ClosedLoopCalibration};

pub trait LinearModel {
    type Sensor: FromBuilder;
    type Processing;
}

impl LinearModel for WaveSensor {
    type Sensor = WaveSensor;
    type Processing = Self;
}

impl<K: CentroidKind> LinearModel for CentroidsProcessing<K> {
    type Sensor = Imaging;
    type Processing = Self;
}

impl LinearModel for SegmentPistonSensor {
    type Sensor = SegmentPistonSensor;
    type Processing = Self;
}

impl LinearModel for Imaging {
    type Sensor = Self;
    type Processing = CentroidsProcessing;
}

impl<M: GmtMx, L, W, const SID: u8> ClosedLoopCalibrateSegment<M, W, SID> for L
where
    <W as FromBuilder>::ComponentBuilder: Clone,
    <<L as LinearModel>::Sensor as FromBuilder>::ComponentBuilder: Clone,
    W: FromBuilder + LinearModel,
    <W as LinearModel>::Processing:
        CalibrationSegment<GmtM2, SID, Sensor = W> + CalibrationSegment<M, SID, Sensor = W>,
    L: LinearModel
        + CalibrationSegment<M, SID, Sensor = <L as LinearModel>::Sensor>
        + CalibrationSegment<GmtM2, SID, Sensor = <L as LinearModel>::Sensor>,
{
    type Sensor = <L as LinearModel>::Sensor;

    fn calibrate(
        optical_model: OpticalModelBuilder<super::SegmentSensorBuilder<M, Self, W, SID>>,
        calib_mode: CalibrationMode,
        closed_loop_optical_model: OpticalModelBuilder<<W as FromBuilder>::ComponentBuilder>,
        closed_loop_calib_mode: CalibrationMode,
    ) -> crate::calibration::Result<ClosedLoopCalib> {
        let mut m2_to_closed_loop_sensor: Reconstructor =
            <<W as LinearModel>::Processing as CalibrationSegment<GmtM2, SID>>::calibrate(
                closed_loop_optical_model.clone(),
                closed_loop_calib_mode.clone(),
            )?
            .into();

        let mut m1_to_closed_loop_sensor: Reconstructor =
            <<W as LinearModel>::Processing as CalibrationSegment<M, SID>>::calibrate(
                closed_loop_optical_model,
                calib_mode.clone(),
            )?
            .into();

        m1_to_closed_loop_sensor.match_areas(&mut m2_to_closed_loop_sensor);
        m1_to_closed_loop_sensor.pseudoinverse();
        m2_to_closed_loop_sensor.pseudoinverse();

        // print!("M1->M2 computation...");
        // let now = Instant::now();
        let m1_to_m2 = m2_to_closed_loop_sensor.pinv_iter_mut().next().unwrap()
            * &m1_to_closed_loop_sensor.calib_slice()[0];
        // println!(
        //     " ({},{}) in {:.3?}",
        //     m1_to_m2.nrows(),
        //     m1_to_m2.ncols(),
        //     now.elapsed()
        // );

        let mut m2_to_sensor: Reconstructor = <Self as CalibrationSegment<GmtM2, SID>>::calibrate(
            optical_model.clone(),
            closed_loop_calib_mode.clone(),
        )?
        .into();
        m2_to_sensor.pseudoinverse();
        // println!("{m2_to_sensor}");
        // println!("cond. #: {}", m2_to_sensor.pseudoinverse().cond());

        let mut m1_to_sensor: Reconstructor =
            <Self as CalibrationSegment<M, SID>>::calibrate(optical_model, calib_mode.clone())?
                .into();
        m1_to_sensor.pseudoinverse();
        // println!("{m1_to_sensor}");
        // println!("cond. #: {}", m1_to_sensor.pseudoinverse().cond());

        m2_to_sensor.match_areas(&mut m1_to_sensor);

        let mut m1_closed_loop_to_sensor = m1_to_sensor.calib_slice()[0].clone();
        let mut m1_closed_loop_to_sensor_as_mut = &mut m1_closed_loop_to_sensor;
        m1_closed_loop_to_sensor_as_mut -= &m2_to_sensor.calib_slice()[0] * m1_to_m2.as_ref();
        // println!("M1 TO AGWS: {m1_closed_loop_to_sensor}");

        Ok(ClosedLoopCalib {
            m1_to_closed_loop_sensor,
            m2_to_closed_loop_sensor,
            m1_to_m2,
            m1_to_sensor: Some(m1_to_sensor),
            m2_to_sensor: Some(m2_to_sensor),
            m1_closed_loop_to_sensor,
        })
    }
}

impl<M: GmtMx, L, W> ClosedLoopCalibration<M, W> for L
where
    <W as FromBuilder>::ComponentBuilder: Clone,
    <<L as LinearModel>::Sensor as FromBuilder>::ComponentBuilder: Clone,
    W: FromBuilder + LinearModel,
    <W as LinearModel>::Processing: CalibrateAssembly<GmtM2, W> + CalibrateAssembly<M, W>,
    L: LinearModel
        + CalibrateAssembly<GmtM2, <L as LinearModel>::Sensor>
        + CalibrateAssembly<M, <L as LinearModel>::Sensor>,
{
    type Sensor = <L as LinearModel>::Sensor;
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crseo::{Gmt, Source, gmt::GmtM1, imaging::LensletArray};
    use faer::MatRef;
    use gmt_dos_clients_io::{
        gmt_m1::{M1ModeShapes, segment::ModeShapes},
        gmt_m2::asm::{M2ASMAsmCommand, segment::AsmCommand},
        optics::{Dev, Frame, SegmentWfeRms, SensorData, Wavefront},
    };
    use interface::{Read, Update, Write};
    use skyangle::Conversion;

    use crate::{
        DeviceInitialize, OpticalModel,
        calibration::{ClosedLoopCalibration, ClosedLoopReconstructor, algebra::CalibProps},
        sensors::{Camera, NoSensor},
    };

    use super::*;

    const SID: u8 = 2;

    #[test]
    fn segment_wave_sensor() -> Result<(), Box<dyn Error>> {
        let m1_n_mode = 6;
        let m2_n_mode = 15;
        let agws_gs = Source::builder().size(3).on_ring(6f32.from_arcmin());
        let gmt = Gmt::builder()
            .m1("bending modes", m1_n_mode)
            .m2("Karhunen-Loeve", m2_n_mode);
        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());
        let optical_model = OpticalModel::<WaveSensor>::builder()
            .gmt(gmt)
            .source(agws_gs.clone())
            .sensor(WaveSensor::builder().source(agws_gs.clone()));

        let calib = <WaveSensor as ClosedLoopCalibrateSegment<GmtM1, WaveSensor, 1>>::calibrate(
            optical_model.clone(),
            CalibrationMode::modes(m1_n_mode, 1e-4),
            closed_loop_optical_model,
            CalibrationMode::modes(m2_n_mode, 1e-4),
        )?;
        println!("{calib}");

        let mut om = optical_model.build()?;

        let mut m1_bm = vec![0f64; m1_n_mode];
        m1_bm[3] = 1e-4;
        let cmd = calib.m1_to_m2() * -MatRef::from_column_major_slice(&m1_bm, m1_n_mode, 1);

        <OpticalModel<WaveSensor> as Read<ModeShapes<1>>>::read(&mut om, m1_bm.into());
        <OpticalModel<WaveSensor> as Read<AsmCommand<1>>>::read(
            &mut om,
            cmd.col_as_slice(0).to_vec().into(),
        );

        om.update();

        <OpticalModel<WaveSensor> as Write<Wavefront>>::write(&mut om).map(|data| {
            println!(
                "{:?}",
                calib.pseudoinverse().as_ref().unwrap() * calib.mask(&data)
            )
        });

        Ok(())
    }

    #[test]
    fn segment_centroids() -> Result<(), Box<dyn Error>> {
        let m1_n_mode = 6;
        let m2_n_mode = 15;
        let n_gs = 3;

        let agws_gs = Source::builder().size(n_gs).on_ring(6f32.from_arcmin());
        let sh48 = Camera::builder()
            .n_sensor(n_gs)
            .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32))
            .lenslet_flux(0.75);
        let mut sh48_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sh48)?;

        let gmt = Gmt::builder()
            .m1("bending modes", m1_n_mode)
            .m2("Karhunen-Loeve", m2_n_mode);

        let optical_model = OpticalModel::<Camera<1>>::builder()
            .gmt(gmt.clone())
            .source(agws_gs.clone())
            .sensor(sh48);

        optical_model.initialize(&mut sh48_centroids);
        dbg!(sh48_centroids.n_valid_lenslets());

        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());
        let calib =
            <CentroidsProcessing as ClosedLoopCalibrateSegment<GmtM1, WaveSensor, SID>>::calibrate(
                optical_model.clone().into(),
                CalibrationMode::modes(m1_n_mode, 1e-4),
                closed_loop_optical_model,
                CalibrationMode::modes(m2_n_mode, 1e-6).start_from(2),
            )?;
        println!("{calib}");
        let calib_pinv = calib.pseudoinverse().unwrap();
        dbg!(calib_pinv.cond());

        let mut sh48_om = optical_model.build()?;
        println!("{sh48_om}");

        let mut m1_bm = vec![0f64; m1_n_mode];
        m1_bm[3] = 1e-4;
        let m2_fit = calib.m1_to_m2() * -MatRef::from_column_major_slice(&m1_bm, m1_n_mode, 1);
        dbg!(m2_fit.shape());
        let mut cmd = vec![0.];
        cmd.extend(m2_fit.col_as_slice(0));

        let mut om = OpticalModel::<NoSensor>::builder()
            .gmt(gmt.clone())
            .build()?;

        <OpticalModel<NoSensor> as Read<ModeShapes<SID>>>::read(&mut om, m1_bm.clone().into());
        <OpticalModel<NoSensor> as Read<AsmCommand<SID>>>::read(&mut om, cmd.clone().into());
        om.update();
        dbg!(<OpticalModel as Write<SegmentWfeRms<-9>>>::write(&mut om));

        <OpticalModel<Camera<1>> as Read<ModeShapes<SID>>>::read(
            &mut sh48_om,
            m1_bm.clone().into(),
        );
        <OpticalModel<Camera<1>> as Read<AsmCommand<SID>>>::read(&mut sh48_om, cmd.into());

        sh48_om.update();

        <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(&mut sh48_om)
            .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut sh48_centroids, data));
        sh48_centroids.update();
        let y = <CentroidsProcessing as Write<SensorData>>::write(&mut sh48_centroids)
            .map(|data| calib.mask(&data))
            .unwrap();
        dbg!(y.len());

        let m1_bm_e = &calib_pinv * y;
        println!("{:?}", m1_bm_e);

        assert!((m1_bm_e[3] - m1_bm[3]).abs() * 1e4 < 1e-2);

        Ok(())
    }

    #[test]
    fn centroids() -> Result<(), Box<dyn Error>> {
        let m1_n_mode = 27;
        let m2_n_mode = 66;
        let n_gs = 3;

        let agws_gs = Source::builder().size(n_gs).on_ring(6f32.from_arcmin());
        let sh48 = Camera::builder()
            .n_sensor(n_gs)
            .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(32))
            .lenslet_flux(0.75);
        let mut sh48_centroids: CentroidsProcessing = CentroidsProcessing::try_from(&sh48)?;

        let gmt = Gmt::builder()
            .m1("bending modes", m1_n_mode)
            .m2("Karhunen-Loeve", m2_n_mode);

        let optical_model = OpticalModel::<Camera<1>>::builder()
            .gmt(gmt.clone())
            .source(agws_gs.clone())
            .sensor(sh48);

        optical_model.initialize(&mut sh48_centroids);
        dbg!(sh48_centroids.n_valid_lenslets());

        let closed_loop_optical_model = OpticalModel::<WaveSensor>::builder().gmt(gmt.clone());
        let mut calib =
            <CentroidsProcessing as ClosedLoopCalibration<GmtM1, WaveSensor>>::calibrate(
                &(&optical_model).into(),
                CalibrationMode::modes(m1_n_mode, 1e-4),
                &closed_loop_optical_model,
                CalibrationMode::modes(m2_n_mode, 1e-6).start_from(2),
            )?;
        calib.pseudoinverse();
        println!("{calib}");

        let mut sh48_om = optical_model.build()?;
        println!("{sh48_om}");

        let mut m1_bm = vec![vec![0f64; m1_n_mode]; 7];
        for i in 0..7 {
            m1_bm[i][i % 6] = 1e-4 * (-1f64).powi(i as i32);
        }
        let cmd: Vec<_> = calib
            .calib_slice()
            .iter()
            .zip(&m1_bm)
            .map(|(c, m1_bm)| c.m1_to_m2() * -MatRef::from_column_major_slice(&m1_bm, m1_n_mode, 1))
            .flat_map(|m2_fit| {
                let mut cmd = vec![0.];
                cmd.extend(m2_fit.col_as_slice(0));
                cmd
            })
            .collect();

        let mut om = OpticalModel::<NoSensor>::builder()
            .gmt(gmt.clone())
            .build()?;

        <OpticalModel<NoSensor> as Read<M1ModeShapes>>::read(
            &mut om,
            m1_bm.iter().cloned().flatten().collect::<Vec<_>>().into(),
        );
        <OpticalModel<NoSensor> as Read<M2ASMAsmCommand>>::read(&mut om, cmd.clone().into());
        om.update();
        dbg!(<OpticalModel as Write<SegmentWfeRms<-9>>>::write(&mut om));

        <OpticalModel<Camera<1>> as Read<M1ModeShapes>>::read(
            &mut sh48_om,
            m1_bm.iter().cloned().flatten().collect::<Vec<_>>().into(),
        );
        <OpticalModel<Camera<1>> as Read<M2ASMAsmCommand>>::read(&mut sh48_om, cmd.into());

        sh48_om.update();

        <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(&mut sh48_om)
            .map(|data| <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut sh48_centroids, data));
        sh48_centroids.update();
        let m1_bm_e = <CentroidsProcessing as Write<SensorData>>::write(&mut sh48_centroids)
            .map(|data| {
                <ClosedLoopReconstructor as Read<SensorData>>::read(&mut calib, data);
                calib.update();
                <ClosedLoopReconstructor as Write<M1ModeShapes>>::write(&mut calib)
            })
            .flatten()
            .unwrap()
            .into_arc();
        dbg!(m1_bm_e.len());

        // let m1_bm_e = &calib_pinv * y;
        // println!("{:?}", m1_bm_e);

        m1_bm_e
            .chunks(m1_n_mode)
            .zip(m1_bm.iter())
            .inspect(|&(m1_bm_e, m1_bm)| {
                println!(
                    "{:+7.3?}",
                    m1_bm_e
                        .iter()
                        .zip(m1_bm)
                        .map(|(x, y)| (x * 1e4, y * 1e4))
                        .collect::<Vec<_>>()
                )
            })
            .for_each(|(m1_bm_e, m1_bm)| {
                let e = m1_bm_e
                    .iter()
                    .zip(m1_bm)
                    .map(|(x, y)| (x - y).powi(2))
                    .sum::<f64>()
                    .sqrt()
                    / m1_n_mode as f64;
                assert!(e < 1e-2);
            });

        Ok(())
    }
}
