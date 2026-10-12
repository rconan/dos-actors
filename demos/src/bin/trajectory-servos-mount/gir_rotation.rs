//! Rotation of the M1 & M2 segments rigid body motions by the GIR angle
//!
//! The mirrors are rotated as rigid bodies about the optical axis (OSS z axis) by the GIR
//! angle γ (mean of the GIR encoders), as an instrument on the GIR would see them.
//! For a segment with vertex r and local frame Q (v_oss = Q v_local), the rotation adds to the
//! segment rigid body motions:
//!  * the translation Qᵀ (Rz(γ) - I) r,
//!  * the rotation Qᵀ Rz(γ) Q, given as the (x,y,z) Euler angles of the ray tracing coordinate
//!    systems (R = Rz Ry Rx).
//!
//! The segment vertices and frames are from the geotrans crate.

use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions, mount::MountEncoders,
};
use interface::{Data, Read, Update, Write};
use std::sync::Arc;

type Vec3 = [f64; 3];
type Mat3 = [[f64; 3]; 3];

fn rx(a: f64) -> Mat3 {
    let (s, c) = a.sin_cos();
    [[1., 0., 0.], [0., c, -s], [0., s, c]]
}
fn ry(a: f64) -> Mat3 {
    let (s, c) = a.sin_cos();
    [[c, 0., s], [0., 1., 0.], [-s, 0., c]]
}
fn rz(a: f64) -> Mat3 {
    let (s, c) = a.sin_cos();
    [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
}
fn mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut m = [[0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    m
}
fn transpose(a: &Mat3) -> Mat3 {
    let mut m = [[0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = a[j][i];
        }
    }
    m
}
fn mulv(a: &Mat3, v: &Vec3) -> Vec3 {
    [0, 1, 2].map(|i| (0..3).map(|k| a[i][k] * v[k]).sum())
}

/// Conic sag
fn sag(r: f64, radius: f64, k: f64) -> f64 {
    r * r / (radius * (1. + (1. - (1. + k) * r * r / (radius * radius)).sqrt()))
}

/// Segment vertex & local frame in the OSS
#[derive(Debug, Clone)]
struct Segment {
    vertex: Vec3,
    frame: Mat3,
}

/// The 7 segments of a mirror: outer segments at `distance` with `clocking0` (deg),
/// the center segment frame given by `center_frame`
fn mirror(
    distance: f64,
    clocking0: f64,
    height: f64,
    radius: f64,
    k: f64,
    outer_frame: impl Fn(f64) -> Mat3,
    center_frame: Mat3,
) -> Vec<Segment> {
    let z = height + sag(distance, radius, k);
    (0..6)
        .map(|i| {
            let o = (clocking0 - 60. * i as f64).to_radians();
            let (s, c) = (90f64.to_radians() + o).sin_cos();
            Segment {
                vertex: [distance * c, distance * s, z],
                frame: outer_frame(o),
            }
        })
        .chain(Some(Segment {
            vertex: [0., 0., height],
            frame: center_frame,
        }))
        .collect()
}

/// Rigid body motions [7x6] in the segment local frames of a mirror rotated by `angle` about z
fn rotation(segments: &[Segment], angle: f64) -> Vec<f64> {
    let r = rz(angle);
    segments
        .iter()
        .flat_map(|s| {
            let qt = transpose(&s.frame);
            let v = mulv(&r, &s.vertex);
            let t = mulv(&qt, &[0, 1, 2].map(|i| v[i] - s.vertex[i]));
            // segment frame rotation in the local frame, R = Rz(g) Ry(b) Rx(a)
            let m = mul(&qt, &mul(&r, &s.frame));
            let b = -m[2][0].asin();
            let a = m[2][1].atan2(m[2][2]);
            let g = m[1][0].atan2(m[0][0]);
            t.into_iter().chain([a, b, g])
        })
        .collect()
}

/// Rotates the M1 & M2 segments rigid body motions by the GIR angle
///
/// Inputs: [MountEncoders], [M1RigidBodyMotions], [M2RigidBodyMotions]
/// Outputs: [M1RigidBodyMotions], [M2RigidBodyMotions] of the mirrors rotated by the GIR angle
pub struct GirRotation {
    m1: Vec<Segment>,
    m2: Vec<Segment>,
    gir: f64,
    m1_rbm: Arc<Vec<f64>>,
    m2_rbm: Arc<Vec<f64>>,
}
impl GirRotation {
    pub fn new() -> Self {
        let pi = std::f64::consts::PI;
        Self {
            m1: mirror(
                8.71,
                0.,
                3.9,
                36.,
                -0.9982857,
                |o| mul(&rz(o), &rx(13.601685f64.to_radians())),
                [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            ),
            // M2 S7 frame is flipped w.r.t. the OSS
            m2: mirror(
                1.08774,
                180.,
                3.9 + 20.26247614,
                -4.1639009,
                -0.71692784,
                |o| mul(&mul(&rz(o), &ry(pi)), &rx(14.777498f64.to_radians())),
                mul(&rz(pi), &ry(pi)),
            ),
            gir: 0.,
            m1_rbm: Default::default(),
            m2_rbm: Default::default(),
        }
    }
    fn rotate(&self, rbm: &[f64], segments: &[Segment]) -> Vec<f64> {
        rbm.iter()
            .zip(rotation(segments, self.gir))
            .map(|(x, r)| x + r)
            .collect()
    }
}
impl Update for GirRotation {}
impl Read<MountEncoders> for GirRotation {
    fn read(&mut self, data: Data<MountEncoders>) {
        // mean of the 4 GIR encoders
        self.gir = data[10..14].iter().sum::<f64>() / 4.;
    }
}
impl Read<M1RigidBodyMotions> for GirRotation {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        self.m1_rbm = data.into_arc();
    }
}
impl Read<M2RigidBodyMotions> for GirRotation {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.m2_rbm = data.into_arc();
    }
}
impl Write<M1RigidBodyMotions> for GirRotation {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        Some(Data::new(self.rotate(&self.m1_rbm, &self.m1)))
    }
}
impl Write<M2RigidBodyMotions> for GirRotation {
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        Some(Data::new(self.rotate(&self.m2_rbm, &self.m2)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_angle() {
        let g = GirRotation::new();
        assert!(rotation(&g.m1, 0.).iter().all(|x| x.abs() < 1e-15));
        assert!(rotation(&g.m2, 0.).iter().all(|x| x.abs() < 1e-15));
    }
    #[test]
    fn center_segment() {
        let g = GirRotation::new();
        let a = 1e-2;
        // M1 S7 on axis: no translation, rotation about z
        let s7 = &rotation(&g.m1, a)[36..];
        assert!(s7[..5].iter().all(|x| x.abs() < 1e-12));
        assert!((s7[5] - a).abs() < 1e-12);
    }
}
