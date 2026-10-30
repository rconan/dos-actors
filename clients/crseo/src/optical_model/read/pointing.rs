use gmt_dos_clients_io::optics::{PointingError, PointingRotationError};
use interface::{Data, Read};

use crate::{OpticalModel, sensors::SensorPropagation};

/// The source directions are offset by the opposite of the pointing error
///
/// The images move on a camera in absolute pointing ([OpticalModel::sensor_pointing])
impl<T: SensorPropagation> Read<PointingError> for OpticalModel<T> {
    fn read(&mut self, data: Data<PointingError>) {
        self.select_device();
        self.pointing.error = [data[0], data[1]];
        self.pointing.apply(&mut self.src);
    }
}

/// The source directions are rotated about the optical axis by the opposite of the rotation error
///
/// The images move on a camera in absolute pointing ([OpticalModel::sensor_pointing])
impl<T: SensorPropagation> Read<PointingRotationError> for OpticalModel<T> {
    fn read(&mut self, data: Data<PointingRotationError>) {
        self.select_device();
        self.pointing.rotation = data[0];
        self.pointing.apply(&mut self.src);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::Camera;
    use crseo::{FromBuilder, Source, imaging::Detector};
    use gmt_dos_clients_io::optics::{Frame, Host, TipTilt};
    use interface::{Update, Write};
    use skyangle::Conversion;

    // V band camera: lambda/(4D) DFT sampling binned 2x2 (2.2mas pixel)
    fn detector() -> Detector {
        Detector {
            n_px_framelet: 128,
            n_px_imagelet: Some(256),
            osf: 4,
            noise_specs: None,
        }
    }

    // optical model with a source at `x` [rd] along the x-axis imaged on a camera
    // in absolute pointing along the optical axis
    fn optical_model(x: f64) -> OpticalModel<Camera<1>> {
        let mut om = OpticalModel::<Camera<1>>::builder()
            .source(Source::builder().zenith_azimuth(vec![x as f32], vec![0.]))
            .sensor(Camera::<1>::builder().detector(detector()))
            .build()
            .unwrap();
        om.sensor_pointing(&[(-x, 0.)]).unwrap();
        om
    }

    // OSS [x,y] tip-tilt [mas] (crseo TipTilt is ordered [y,x])
    // and frame centroid [mas] (x along the columns)
    fn tt_centroid(om: &mut OpticalModel<Camera<1>>) -> ([f64; 2], [f64; 2]) {
        om.update();
        let tt = <_ as Write<TipTilt>>::write(om).unwrap();
        let pixel = (om.sensor().unwrap().pixel_scale(om.source()) as f64).to_mas();
        let frame = <_ as Write<Frame<Host>>>::write(om).unwrap();
        let n = (frame.len() as f64).sqrt() as usize;
        let c = (n - 1) as f64 / 2.;
        let (mut f, mut x, mut y) = (0f64, 0f64, 0f64);
        for (k, v) in frame.iter().enumerate() {
            let v = *v as f64;
            f += v;
            x += v * ((k % n) as f64 - c);
            y += v * ((k / n) as f64 - c);
        }
        (
            [tt[1].to_mas(), tt[0].to_mas()],
            [x / f * pixel, y / f * pixel],
        )
    }

    // image motion vs pointing error with the camera in absolute pointing and,
    // the camera pointing along the source, from the wavefront tilt
    #[test]
    #[ignore]
    fn pointing_scan() {
        // absolute pointing
        let mut abs = OpticalModel::<Camera<1>>::builder()
            .sensor(Camera::<1>::builder().detector(detector()))
            .build()
            .unwrap();
        abs.sensor_pointing(&[(0., 0.)]).unwrap();
        // wavefront tilt
        let mut wft = OpticalModel::<Camera<1>>::builder()
            .sensor(Camera::<1>::builder().detector(detector()))
            .build()
            .unwrap();
        let pixel = (abs.sensor().unwrap().pixel_scale(abs.source()) as f64).to_mas();
        println!("pixel scale: {pixel:.3}mas");
        for k in 0..13 {
            let p = pixel * k as f64 / 8.;
            <_ as Read<PointingError>>::read(&mut abs, Data::new(vec![p.from_mas(), 0.]));
            wft.gmt.pointing_error = Some((p.from_mas(), 0.));
            let (_, ca) = tt_centroid(&mut abs);
            let (tw, cw) = tt_centroid(&mut wft);
            println!(
                "SCAN {:5.3}px | abs {:7.3} {:7.3} | wft {:7.3} {:7.3} | wft tip-tilt {:7.3}",
                k as f64 / 8.,
                ca[0] / pixel,
                ca[1] / pixel,
                cw[0] / pixel,
                cw[1] / pixel,
                tw[0] / pixel
            );
        }
    }

    #[test]
    fn pointing_error() {
        let mut om = optical_model(0.);
        let (tt0, c0) = tt_centroid(&mut om);
        println!("no pointing error: tip-tilt {tt0:.3?}mas, centroid {c0:.3?}mas");
        for p in [[20., 0.], [0., 20.]] {
            <_ as Read<PointingError>>::read(
                &mut om,
                Data::new(vec![p[0].from_mas(), p[1].from_mas()]),
            );
            let (tt, c) = tt_centroid(&mut om);
            println!("pointing error {p:?}mas: tip-tilt {tt:.3?}mas, centroid {c:.3?}mas");
            // the image moves by -p (the frame centroid is ~2% short),
            // the wavefront tip-tilt does not change
            for i in 0..2 {
                assert!((c[i] - c0[i] + p[i]).abs() < 0.05 * p[i].abs() + 0.01);
                assert!((tt[i] - tt0[i]).abs() < 0.01);
            }
        }
    }

    #[test]
    fn pointing_rotation_error() {
        let off_axis = 100f64.from_mas();
        let mut om = optical_model(off_axis);
        let (tt0, c0) = tt_centroid(&mut om);
        println!("no rotation error: tip-tilt {tt0:.3?}mas, centroid {c0:.3?}mas");
        // the off-axis image is at its field position
        assert!((c0[0] - off_axis.to_mas()).abs() < 0.05 * off_axis.to_mas() && c0[1].abs() < 0.01);
        let rotation = 0.04;
        <_ as Read<PointingRotationError>>::read(&mut om, Data::new(vec![rotation]));
        let (tt, c) = tt_centroid(&mut om);
        println!("rotation error {rotation}rd: tip-tilt {tt:.3?}mas, centroid {c:.3?}mas");
        // the image is rotated by -rotation
        let (s, co) = rotation.sin_cos();
        assert!((c[0] - c0[0] - (co - 1.) * off_axis.to_mas()).abs() < 0.05);
        assert!((c[1] - c0[1] + s * off_axis.to_mas()).abs() < 0.05 * s * off_axis.to_mas());
        // back to no rotation error
        <_ as Read<PointingRotationError>>::read(&mut om, Data::new(vec![0.]));
        let (_, c) = tt_centroid(&mut om);
        assert!((c[0] - c0[0]).abs() < 0.01 && (c[1] - c0[1]).abs() < 0.01);
    }
}
