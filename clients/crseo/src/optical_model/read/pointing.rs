use gmt_dos_clients_io::optics::PointingError;
use interface::{Data, Read};

use crate::{OpticalModel, sensors::SensorPropagation};

/// The source is ray traced at its direction offset by the opposite of the pointing error
impl<T: SensorPropagation> Read<PointingError> for OpticalModel<T> {
    fn read(&mut self, data: Data<PointingError>) {
        let (x, y) = (data[0], data[1]);
        self.gmt.pointing_error = Some((x.hypot(y), y.atan2(x)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::Camera;
    use crseo::{FromBuilder, imaging::Detector};
    use gmt_dos_clients_io::optics::{Frame, Host, TipTilt};
    use interface::{Update, Write};
    use skyangle::Conversion;

    // tip-tilt [mas] and frame centroid [pixel] of the optical model
    fn tt_centroid(om: &mut OpticalModel<Camera<1>>) -> ([f64; 2], [f64; 2]) {
        om.update();
        let tt = <_ as Write<TipTilt>>::write(om).unwrap();
        let frame = <_ as Write<Frame<Host>>>::write(om).unwrap();
        let n = (frame.len() as f64).sqrt() as usize;
        let c = (n - 1) as f64 / 2.;
        let (mut f, mut i, mut j) = (0f64, 0f64, 0f64);
        for (k, v) in frame.iter().enumerate() {
            let v = *v as f64;
            f += v;
            i += v * ((k / n) as f64 - c);
            j += v * ((k % n) as f64 - c);
        }
        ([tt[0].to_mas(), tt[1].to_mas()], [i / f, j / f])
    }

    #[test]
    fn pointing_error() -> Result<(), Box<dyn std::error::Error>> {
        let mut om = OpticalModel::<Camera<1>>::builder()
            .sensor(Camera::<1>::builder().detector(Detector {
                n_px_framelet: 64,
                n_px_imagelet: None,
                osf: 2,
                noise_specs: None,
            }))
            .build()?;
        let (tt0, c0) = tt_centroid(&mut om);
        println!("no pointing error: tip-tilt {tt0:.3?}mas, centroid {c0:.3?}px");
        for (x, y) in [(20., 0.), (0., 20.)] {
            <_ as Read<PointingError>>::read(
                &mut om,
                Data::new(vec![x.from_mas(), y.from_mas()]),
            );
            let (tt, c) = tt_centroid(&mut om);
            println!("pointing error [{x},{y}]mas: tip-tilt {tt:.3?}mas, centroid {c:.3?}px");
        }
        Ok(())
    }
}
