//! Rigid telescope motion removal from the M1 & M2 segment rigid body motions
//!
//! The rigid telescope motion is K [azimuth, elevation, GIR] with K the kinematic gain from
//! the mount encoders to the M1 & M2 rigid body motions derived from the DC gain of the FEM
//! (all modes, the mount rigid body modes included) from the mount drive torques
//! ([RigidBodyRemoval::from_fem]).
//!
//! [RigidBodyRemoval::from_geometry] is the geometric equivalent: the coordinate transformation of the segment vertices
//! and frames by the rotation about the azimuth and elevation axes, both through the OSS
//! origin, linearized (small angles) as the FEM is linear:
//!  * azimuth axis: (0, sin(30deg), cos(30deg)) in the OSS for the FEM at 30deg zenith angle
//!  * elevation axis: -x
//!
//! For a telescope rotation vector w, the rigid body motion of a segment with vertex r and
//! local frame Q (v_oss = Q v_local) is [Q^T (w x r), Q^T w].
//! The segment vertices and frames are from the geotrans crate (see also `rigid_body.py`).

use gmt_dos_clients_fem::Model;
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::{OSSAzDriveTorque, OSSElDriveTorque, OSSRotDriveTorque},
        outputs::{
            MCM2Lcl6D, OSSAzEncoderAngle, OSSElEncoderAngle, OSSM1Lcl, OSSRotEncoderAngle,
        },
    },
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    mount::MountEncoders,
};
use gmt_fem::FEM;
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

/// Rigid body motions [7x6] in the segment local frames per radian of azimuth, elevation & GIR
fn sensitivities(segments: &[Segment]) -> [Vec<f64>; 3] {
    let azimuth: Vec3 = [0., 30f64.to_radians().sin(), 30f64.to_radians().cos()];
    let elevation: Vec3 = [-1., 0., 0.];
    let gir: Vec3 = [0., 0., 0.];
    [azimuth, elevation, gir].map(|w| {
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
    m1: [Vec<f64>; 3],
    m2: [Vec<f64>; 3],
    angles: [f64; 3],
    m1_rbm: Arc<Vec<f64>>,
    m2_rbm: Arc<Vec<f64>>,
}

/// Inverse of a 3x3 matrix
fn inv3(m: &Mat3) -> Option<Mat3> {
    let c = |i: usize, j: usize| {
        let (i1, i2) = ((i + 1) % 3, (i + 2) % 3);
        let (j1, j2) = ((j + 1) % 3, (j + 2) % 3);
        m[i1][j1] * m[i2][j2] - m[i1][j2] * m[i2][j1]
    };
    let det: f64 = (0..3).map(|j| m[0][j] * c(0, j)).sum();
    (det != 0. && det.is_finite()).then(|| {
        let mut inv = [[0f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                inv[i][j] = c(j, i) / det;
            }
        }
        inv
    })
}

impl RigidBodyRemoval {
    /// Kinematics from the DC gain of the FEM
    ///
    /// H = Φ diag(1/ω²) B, the DC gain of all the FEM modes from the azimuth, elevation and
    /// GIR drive torques to the mount encoders and to the M1 & M2 rigid body motions,
    /// K = H_rbm E⁺ with E = A H_encoders, A averaging the encoders of each axis
    pub fn from_fem(fem: &FEM) -> anyhow::Result<Self> {
        let n = fem.n_modes();
        let w2: Vec<f64> = fem
            .eigen_frequencies_to_radians()
            .into_iter()
            .map(|w| w * w)
            .collect();
        let out = |phi: Option<Vec<f64>>, name: &str| {
            phi.ok_or_else(|| anyhow::anyhow!("{name} is missing from the FEM outputs"))
        };
        let inp = |b: Option<Vec<f64>>, name: &str| {
            b.ok_or_else(|| anyhow::anyhow!("{name} is missing from the FEM inputs"))
        };
        // modal forces per drive torque: [n_modes x n_torques] row-wise
        let b: Vec<Vec<f64>> = [
            inp(fem.in2modes::<OSSAzDriveTorque>(), "OSSAzDriveTorque")?,
            inp(fem.in2modes::<OSSElDriveTorque>(), "OSSElDriveTorque")?,
            inp(fem.in2modes::<OSSRotDriveTorque>(), "OSSRotDriveTorque")?,
        ]
        .iter()
        .fold(vec![vec![]; n], |mut b, x| {
            let m = x.len() / n;
            b.iter_mut()
                .zip(x.chunks(m))
                .for_each(|(b, x)| b.extend_from_slice(x));
            b
        });
        let n_t = b[0].len();
        // DC gain from the drive torques to outputs: [n_outputs x n_torques] row-wise
        let dc_gain = |phi: Vec<f64>| -> Vec<Vec<f64>> {
            phi.chunks(n)
                .map(|row| {
                    let mut h = vec![0f64; n_t];
                    row.iter().zip(&w2).zip(&b).for_each(|((p, w2), b)| {
                        let c = p / w2;
                        h.iter_mut().zip(b).for_each(|(h, b)| *h += c * b);
                    });
                    h
                })
                .collect()
        };
        let mean = |h: Vec<Vec<f64>>| -> Vec<f64> {
            let m = h.len() as f64;
            (0..n_t).map(|j| h.iter().map(|h| h[j]).sum::<f64>() / m).collect()
        };
        // averaged encoders DC gain: [3 x n_torques]
        let e = [
            mean(dc_gain(out(fem.modes2out::<OSSAzEncoderAngle>(), "OSSAzEncoderAngle")?)),
            mean(dc_gain(out(fem.modes2out::<OSSElEncoderAngle>(), "OSSElEncoderAngle")?)),
            mean(dc_gain(out(fem.modes2out::<OSSRotEncoderAngle>(), "OSSRotEncoderAngle")?)),
        ];
        // E⁺ = E^T (E E^T)^-1: [n_torques x 3]
        let mut eet = [[0f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                eet[i][j] = e[i].iter().zip(&e[j]).map(|(a, b)| a * b).sum();
            }
        }
        let eet_inv = inv3(&eet).ok_or_else(|| anyhow::anyhow!("singular encoders DC gain"))?;
        let e_pinv: Vec<[f64; 3]> = (0..n_t)
            .map(|t| [0, 1, 2].map(|k| (0..3).map(|i| e[i][t] * eet_inv[i][k]).sum()))
            .collect();
        // rigid body motions per radian of [azimuth, elevation, GIR]
        let kinematics = |phi: Vec<f64>| -> [Vec<f64>; 3] {
            let h = dc_gain(phi);
            [0, 1, 2].map(|k| {
                h.iter()
                    .map(|h| h.iter().zip(&e_pinv).map(|(h, p)| h * p[k]).sum())
                    .collect()
            })
        };
        Ok(Self {
            m1: kinematics(out(fem.modes2out::<OSSM1Lcl>(), "OSSM1Lcl")?),
            m2: kinematics(out(fem.modes2out::<MCM2Lcl6D>(), "MCM2Lcl6D")?),
            ..Self::from_geometry()
        })
    }
    /// Kinematics from the GMT geometry (geotrans)
    pub fn from_geometry() -> Self {
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
            angles: [0f64; 3],
            m1_rbm: Default::default(),
            m2_rbm: Default::default(),
        }
    }
    /// Largest difference between the kinematics of `self` and `other` per radian of
    /// [azimuth, elevation, GIR]
    pub fn max_difference(&self, other: &Self) -> [f64; 3] {
        [0, 1, 2].map(|k| {
            self.m1[k]
                .iter()
                .zip(&other.m1[k])
                .chain(self.m2[k].iter().zip(&other.m2[k]))
                .map(|(a, b)| (a - b).abs())
                .fold(0f64, f64::max)
        })
    }
    fn remove(&self, rbm: &[f64], s: &[Vec<f64>; 3]) -> Vec<f64> {
        rbm.iter()
            .enumerate()
            .map(|(i, x)| x - (0..3).map(|k| s[k][i] * self.angles[k]).sum::<f64>())
            .collect()
    }
}
impl Update for RigidBodyRemoval {}
impl Read<MountEncoders> for RigidBodyRemoval {
    fn read(&mut self, data: Data<MountEncoders>) {
        // mean of the 4 azimuth, of the 6 elevation and of the 4 GIR encoders
        self.angles = [
            data[..4].iter().sum::<f64>() / 4.,
            data[4..10].iter().sum::<f64>() / 6.,
            data[10..14].iter().sum::<f64>() / 4.,
        ];
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
        let r = RigidBodyRemoval::from_geometry();
        // M1 S7 vertex 3.9m above the OSS origin: Tx = 0.5 x 3.9 per radian of azimuth
        assert!((r.m1[0][36] - 1.95).abs() < 1e-12);
        // the frame rotation has a unit norm for all segments
        for s in r.m1[0].chunks(6).chain(r.m2[0].chunks(6)) {
            let n = s[3..].iter().map(|x| x * x).sum::<f64>().sqrt();
            assert!((n - 1.).abs() < 1e-12);
        }
    }
}
