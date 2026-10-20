use gmt_dos_clients_io::optics::{PointingError, PointingRotationError};
use interface::{Data, Read};

use crate::{OpticalModel, sensors::SensorPropagation};

/// The sources are ray traced at their directions offset by the opposite of the pointing error
impl<T: SensorPropagation> Read<PointingError> for OpticalModel<T> {
    fn read(&mut self, data: Data<PointingError>) {
        self.pointing.error = [data[0], data[1]];
    }
}

/// The sources are ray traced at their directions rotated about the optical axis
/// by the opposite of the rotation error
impl<T: SensorPropagation> Read<PointingRotationError> for OpticalModel<T> {
    fn read(&mut self, data: Data<PointingRotationError>) {
        self.pointing.rotation = data[0];
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

    fn optical_model(n_src: usize, zenith: Vec<f32>) -> OpticalModel<Camera<1>> {
        OpticalModel::<Camera<1>>::builder()
            .source(
                Source::builder()
                    .size(n_src)
                    .zenith_azimuth(zenith, vec![0.; n_src]),
            )
            .sensor(
                Camera::<1>::builder()
                    .n_sensor(n_src)
                    .detector(Detector {
                        n_px_framelet: 64,
                        n_px_imagelet: None,
                        osf: 2,
                        noise_specs: None,
                    }),
            )
            .build()
            .unwrap()
    }

    // OSS [x,y] tip-tilt [mas] of each source (crseo TipTilt is ordered [y,x])
    fn tiptilt(om: &mut OpticalModel<Camera<1>>) -> Vec<[f64; 2]> {
        om.update();
        let tt = <_ as Write<TipTilt>>::write(om).unwrap();
        let n = tt.len() / 2;
        (0..n)
            .map(|i| [tt[n + i].to_mas(), tt[i].to_mas()])
            .collect()
    }

    // frame centroid [pixel] of a single source (x along the columns)
    fn centroid(om: &mut OpticalModel<Camera<1>>) -> [f64; 2] {
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
        [x / f, y / f]
    }

    #[test]
    fn pointing_error() {
        let mut om = optical_model(1, vec![0.]);
        let tt0 = tiptilt(&mut om)[0];
        let c0 = centroid(&mut om);
        println!("no pointing error: tip-tilt {tt0:.3?}mas, centroid {c0:.3?}px");
        for p in [[20., 0.], [0., 20.]] {
            <_ as Read<PointingError>>::read(
                &mut om,
                Data::new(vec![p[0].from_mas(), p[1].from_mas()]),
            );
            let tt = tiptilt(&mut om)[0];
            let c = centroid(&mut om);
            println!("pointing error {p:?}mas: tip-tilt {tt:.3?}mas, centroid {c:.3?}px");
            // the image moves by -p
            for i in 0..2 {
                assert!((tt[i] - tt0[i] + p[i]).abs() < 0.1);
                assert!((c[i] - c0[i]) * p[i] <= 0.);
            }
        }
    }

    #[test]
    fn pointing_rotation_error() {
        let off_axis = 250f64.from_mas();
        let mut om = optical_model(2, vec![0., off_axis as f32]);
        let tt0 = tiptilt(&mut om);
        println!("no rotation error: tip-tilt {tt0:.3?}mas");
        let rotation = 0.04;
        <_ as Read<PointingRotationError>>::read(&mut om, Data::new(vec![rotation]));
        let tt = tiptilt(&mut om);
        println!("rotation error {rotation}rd: tip-tilt {tt:.3?}mas");
        // the on-axis source does not move
        assert!((tt[0][0] - tt0[0][0]).abs() < 0.1 && (tt[0][1] - tt0[0][1]).abs() < 0.1);
        // the off-axis source along x is rotated by -rotation
        let (s, c) = rotation.sin_cos();
        assert!((tt[1][0] - tt0[1][0] - (c - 1.) * off_axis.to_mas()).abs() < 0.01);
        assert!((tt[1][1] - tt0[1][1] + s * off_axis.to_mas()).abs() < 0.01);
    }
}
