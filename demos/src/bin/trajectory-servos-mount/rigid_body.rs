//! Rigid telescope motion removal from the M1 & M2 segment rigid body motions
//!
//! The rigid telescope motion is the coordinate transformation of the segment vertices
//! and frames by the rotation about the azimuth and elevation axes, both through the OSS
//! origin, linearized (small angles) as the FEM is linear:
//!  * azimuth axis: (0, sin(30deg), cos(30deg)) in the OSS for the FEM at 30deg zenith angle
//!  * elevation axis: -x
//!
//! For a telescope rotation vector w, the rigid body motion of a segment with vertex r and
//! local frame Q (v_oss = Q v_local) is [Q^T (w x r), Q^T w].
//! The segment vertices and frames are from the geotrans crate (see also `rigid_body.py`).

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
/// Q^T v
fn tmul(q: &Mat3, v: &Vec3) -> Vec3 {
    let mut u = [0f64; 3];
    for i in 0..3 {
        u[i] = (0..3).map(|k| q[k][i] * v[k]).sum();
    }
    u
}
fn cross(a: &Vec3, b: &Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Conic sag
fn sag(r: f64, radius: f64, k: f64) -> f64 {
    r * r / (radius * (1. + (1. - (1. + k) * r * r / (radius * radius)).sqrt()))
}

/// Segment vertex & local frame in the OSS
struct Segment {
    vertex: Vec3,
    frame: Mat3,
}

/// The 7 segments of a mirror: outer segments at `distance` with `clocking0` (deg),
/// tilted by `beta` (deg), the center segment frame given by `center_frame`
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

/// Rigid body motions [7x6] in the segment local frames per radian of azimuth and of elevation
fn sensitivities(segments: &[Segment]) -> [Vec<f64>; 2] {
    let azimuth: Vec3 = [0., 30f64.to_radians().sin(), 30f64.to_radians().cos()];
    let elevation: Vec3 = [-1., 0., 0.];
    [azimuth, elevation].map(|w| {
        segments
            .iter()
            .flat_map(|s| {
                let t = tmul(&s.frame, &cross(&w, &s.vertex));
                let r = tmul(&s.frame, &w);
                t.into_iter().chain(r)
            })
            .collect()
    })
}

/// Removes the rigid telescope motion from the M1 & M2 segment rigid body motions
///
/// Inputs: [MountEncoders], [M1RigidBodyMotions], [M2RigidBodyMotions]
/// Outputs: [M1RigidBodyMotions], [M2RigidBodyMotions] minus the rigid telescope motion
pub struct RigidBodyRemoval {
    m1: [Vec<f64>; 2],
    m2: [Vec<f64>; 2],
    azimuth: f64,
    elevation: f64,
    m1_rbm: Arc<Vec<f64>>,
    m2_rbm: Arc<Vec<f64>>,
}
impl RigidBodyRemoval {
    pub fn new() -> Self {
        let m1 = mirror(
            8.71,
            0.,
            3.9,
            36.,
            -0.9982857,
            |o| mul(&rz(o), &rx(13.601685f64.to_radians())),
            [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        );
        let pi = std::f64::consts::PI;
        let m2 = mirror(
            1.08774,
            180.,
            3.9 + 20.26247614,
            -4.1639009,
            -0.71692784,
            |o| mul(&mul(&rz(o), &ry(pi)), &rx(14.777498f64.to_radians())),
            // M2 S7 frame is flipped w.r.t. the OSS
            mul(&rz(pi), &ry(pi)),
        );
        Self {
            m1: sensitivities(&m1),
            m2: sensitivities(&m2),
            azimuth: 0.,
            elevation: 0.,
            m1_rbm: Default::default(),
            m2_rbm: Default::default(),
        }
    }
    fn remove(&self, rbm: &[f64], s: &[Vec<f64>; 2]) -> Vec<f64> {
        rbm.iter()
            .zip(s[0].iter().zip(&s[1]))
            .map(|(x, (a, e))| x - a * self.azimuth - e * self.elevation)
            .collect()
    }
}
impl Update for RigidBodyRemoval {}
impl Read<MountEncoders> for RigidBodyRemoval {
    fn read(&mut self, data: Data<MountEncoders>) {
        // mean of the 4 azimuth and of the 6 elevation encoders
        self.azimuth = data[..4].iter().sum::<f64>() / 4.;
        self.elevation = data[4..10].iter().sum::<f64>() / 6.;
    }
}
impl Read<M1RigidBodyMotions> for RigidBodyRemoval {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        self.m1_rbm = data.into_arc();
    }
}
impl Read<M2RigidBodyMotions> for RigidBodyRemoval {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.m2_rbm = data.into_arc();
    }
}
impl Write<M1RigidBodyMotions> for RigidBodyRemoval {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        Some(Data::new(self.remove(&self.m1_rbm, &self.m1)))
    }
}
impl Write<M2RigidBodyMotions> for RigidBodyRemoval {
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        Some(Data::new(self.remove(&self.m2_rbm, &self.m2)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn center_segment_azimuth_lever_arm() {
        let r = RigidBodyRemoval::new();
        // M1 S7 vertex 3.9m above the OSS origin: Tx = 0.5 x 3.9 per radian of azimuth
        assert!((r.m1[0][36] - 1.95).abs() < 1e-12);
        // the frame rotation has a unit norm for all segments
        for s in r.m1[0].chunks(6).chain(r.m2[0].chunks(6)) {
            let n = s[3..].iter().map(|x| x * x).sum::<f64>().sqrt();
            assert!((n - 1.).abs() < 1e-12);
        }
    }
}
