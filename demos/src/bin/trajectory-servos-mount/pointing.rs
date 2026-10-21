//! Telescope pointing error from the mount tracking error
//!
//! The telescope rotation from the mount tracking error (encoders - setpoint)
//! Δaz about the azimuth axis (0, sin ζ, cos ζ) and Δel about the elevation axis (-1, 0, 0),
//! ζ being the zenith angle, is the rotation vector w = Δaz (0, sin ζ, cos ζ) + Δel (-1, 0, 0).
//! It moves the telescope optical axis (OSS z axis) by w × z = (w_y, -w_x, 0), i.e. the pointing
//! error in the OSS is (Δaz cos(el), Δel).
//! The elevation el is the telescope elevation: the initial elevation of the trajectory plus the
//! averaged elevation encoder, the encoders being offsets from the initial pointing.
//! As the M1 & M2 rigid body motions (`gir_rotation.rs`), the pointing error is rotated by the
//! GIR angle γ (average GIR encoder) about the optical axis, as seen by an instrument on the GIR.
//! The GIR tracking error Δγ (encoder - field rotation) is the rotation error of the instrument
//! about the optical axis relative to the sky: the sources seen from the instrument are rotated
//! by -Δγ. The field rotation is the GIR setpoint when the GIR tracks it, otherwise (GIR setpoint
//! at 0) Δγ is the opposite of the field rotation and the sources rotate with the field.

use gmt_dos_clients_io::{
    mount::{AverageMountEncoders, MountSetPoint},
    optics::{PointingError, PointingRotationError},
};
use interface::{Data, Read, Update, Write};
use std::sync::Arc;

use crate::FieldRotation;

/// Mount tracking error to telescope pointing error
///
/// Inputs: [MountSetPoint], [AverageMountEncoders] (azimuth, elevation, GIR), [FieldRotation]
/// Outputs: [PointingError] (x, y) in the GIR rotated OSS, [PointingRotationError] (GIR tracking error)
pub struct Pointing {
    // initial elevation of the trajectory [rad]
    elevation: f64,
    on: bool,
    field_rotation: f64,
    setpoint: Arc<Vec<f64>>,
    encoders: Arc<Vec<f64>>,
}
impl Pointing {
    /// Creates the pointing error client for a trajectory starting at `elevation` [rad]
    pub fn new(elevation: f64) -> Self {
        Self {
            elevation,
            on: true,
            field_rotation: 0.,
            setpoint: Arc::new(vec![0.; 3]),
            encoders: Arc::new(vec![0.; 3]),
        }
    }
    /// Turns the pointing error off (zero output)
    pub fn off(mut self) -> Self {
        self.on = false;
        self
    }
}
impl Update for Pointing {}
impl Read<MountSetPoint> for Pointing {
    fn read(&mut self, data: Data<MountSetPoint>) {
        self.setpoint = data.into_arc();
    }
}
impl Read<FieldRotation> for Pointing {
    fn read(&mut self, data: Data<FieldRotation>) {
        self.field_rotation = data[0];
    }
}
impl Read<AverageMountEncoders> for Pointing {
    fn read(&mut self, data: Data<AverageMountEncoders>) {
        self.encoders = data.into_arc();
    }
}
impl Write<PointingError> for Pointing {
    fn write(&mut self) -> Option<Data<PointingError>> {
        if !self.on {
            return Some(Data::new(vec![0.; 2]));
        }
        let d_az = self.encoders[0] - self.setpoint[0];
        let d_el = self.encoders[1] - self.setpoint[1];
        // telescope elevation: initial elevation + elevation encoder offset
        let el = self.elevation + self.encoders[1];
        let (x, y) = (d_az * el.cos(), d_el);
        let (s, c) = self.encoders[2].sin_cos();
        Some(Data::new(vec![c * x - s * y, s * x + c * y]))
    }
}
impl Write<PointingRotationError> for Pointing {
    fn write(&mut self) -> Option<Data<PointingRotationError>> {
        if !self.on {
            return Some(Data::new(vec![0.]));
        }
        Some(Data::new(vec![self.encoders[2] - self.field_rotation]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn azimuth_elevation() {
        let mut p = Pointing::new(45f64.to_radians());
        <_ as Read<MountSetPoint>>::read(&mut p, Data::new(vec![0., 0., 0.]));
        <_ as Read<AverageMountEncoders>>::read(&mut p, Data::new(vec![1e-6, 2e-6, 0.]));
        let e = <_ as Write<PointingError>>::write(&mut p).unwrap();
        assert!((e[0] - 1e-6 * (45f64.to_radians() + 2e-6).cos()).abs() < 1e-15);
        assert!((e[1] - 2e-6).abs() < 1e-15);
        // a quarter turn of the GIR: x -> y, y -> -x
        <_ as Read<AverageMountEncoders>>::read(
            &mut p,
            Data::new(vec![1e-6, 0., std::f64::consts::FRAC_PI_2]),
        );
        let e = <_ as Write<PointingError>>::write(&mut p).unwrap();
        assert!(e[0].abs() < 1e-15);
        assert!((e[1] - 1e-6 * 45f64.to_radians().cos()).abs() < 1e-15);
    }

    #[test]
    fn elevation() {
        // trajectory starting at 45deg, the telescope 15deg lower: cos(30deg)
        let mut p = Pointing::new(45f64.to_radians());
        let d_el = -15f64.to_radians();
        <_ as Read<MountSetPoint>>::read(&mut p, Data::new(vec![0., d_el, 0.]));
        <_ as Read<AverageMountEncoders>>::read(&mut p, Data::new(vec![1e-6, d_el, 0.]));
        let e = <_ as Write<PointingError>>::write(&mut p).unwrap();
        assert!((e[0] - 1e-6 * 30f64.to_radians().cos()).abs() < 1e-15);
        assert!(e[1].abs() < 1e-15);
    }

    #[test]
    fn gir() {
        let mut p = Pointing::new(45f64.to_radians());
        // GIR tracking the field rotation
        <_ as Read<MountSetPoint>>::read(&mut p, Data::new(vec![0., 0., 3e-6]));
        <_ as Read<FieldRotation>>::read(&mut p, Data::new(vec![3e-6]));
        <_ as Read<AverageMountEncoders>>::read(&mut p, Data::new(vec![0., 0., 1e-6]));
        let e = <_ as Write<PointingRotationError>>::write(&mut p).unwrap();
        assert!((e[0] + 2e-6).abs() < 1e-15);
        // GIR at rest, the field rotating
        <_ as Read<MountSetPoint>>::read(&mut p, Data::new(vec![0., 0., 0.]));
        <_ as Read<AverageMountEncoders>>::read(&mut p, Data::new(vec![0., 0., 0.]));
        let e = <_ as Write<PointingRotationError>>::write(&mut p).unwrap();
        assert!((e[0] + 3e-6).abs() < 1e-15);
    }
}
