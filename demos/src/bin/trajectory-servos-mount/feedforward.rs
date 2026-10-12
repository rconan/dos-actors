//! Mount drive torques feed-forward of the FEM "rigid body" springs
//!
//! In the FEM, the azimuth, elevation and GIR rigid body modes have small but non-zero
//! eigen frequencies (0.07-0.1Hz): holding the mount at an angle θ away from the FEM
//! pointing requires a static drive torque proportional to θ.
//! Over a long trajectory, that torque saturates the mount drives (800kN.m in azimuth).
//!
//! The feed-forward ([MountTorquesFeedForward]) is added by the mount controller to the drive
//! torques after the drives saturation, it is the torque that holds the mount at the
//! setpoint angles:
//! for each axis, the same torque τ = θ / Σ g on each drive, with g the DC gain from a drive
//! torque to the averaged encoders of the axis.

use gmt_dos_clients_fem::Model;
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::{OSSAzDriveTorque, OSSElDriveTorque, OSSRotDriveTorque},
        outputs::{OSSAzEncoderAngle, OSSElEncoderAngle, OSSRotEncoderAngle},
    },
    mount::{MountSetPoint, MountTorquesFeedForward},
};
use gmt_fem::FEM;
use interface::{Data, Read, Update, Write};

/// DC gain of all the FEM modes from the [azimuth (8), elevation (4), GIR (4)] drive torques
/// to the outputs whose modes to outputs matrix is `phi` ([n_outputs x n_modes] row-wise)
struct DriveDcGain {
    n_modes: usize,
    w2: Vec<f64>,
    b: Vec<Vec<f64>>,
}
impl DriveDcGain {
    pub fn new(fem: &FEM) -> anyhow::Result<Self> {
        let n = fem.n_modes();
        let w2: Vec<f64> = fem
            .eigen_frequencies_to_radians()
            .into_iter()
            .map(|w| w * w)
            .collect();
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
        Ok(Self { n_modes: n, w2, b })
    }
    pub fn n_torques(&self) -> usize {
        self.b[0].len()
    }
    /// DC gain [n_outputs x n_torques] row-wise
    pub fn gain(&self, phi: Vec<f64>) -> Vec<Vec<f64>> {
        let n_t = self.n_torques();
        phi.chunks(self.n_modes)
            .map(|row| {
                let mut h = vec![0f64; n_t];
                row.iter().zip(&self.w2).zip(&self.b).for_each(|((p, w2), b)| {
                    let c = p / w2;
                    h.iter_mut().zip(b).for_each(|(h, b)| *h += c * b);
                });
                h
            })
            .collect()
    }
    /// DC gain [3 x n_torques] of the averaged azimuth, elevation and GIR encoders
    pub fn encoders(&self, fem: &FEM) -> anyhow::Result<[Vec<f64>; 3]> {
        let out = |phi: Option<Vec<f64>>, name: &str| {
            phi.ok_or_else(|| anyhow::anyhow!("{name} is missing from the FEM outputs"))
        };
        let n_t = self.n_torques();
        let mean = |h: Vec<Vec<f64>>| -> Vec<f64> {
            let m = h.len() as f64;
            (0..n_t).map(|j| h.iter().map(|h| h[j]).sum::<f64>() / m).collect()
        };
        Ok([
            mean(self.gain(out(fem.modes2out::<OSSAzEncoderAngle>(), "OSSAzEncoderAngle")?)),
            mean(self.gain(out(fem.modes2out::<OSSElEncoderAngle>(), "OSSElEncoderAngle")?)),
            mean(self.gain(out(fem.modes2out::<OSSRotEncoderAngle>(), "OSSRotEncoderAngle")?)),
        ])
    }
}

/// Feed-forward of the mount drive torques from the mount setpoint
///
/// Input: [MountSetPoint], output: [MountTorquesFeedForward] for the [azimuth (8), elevation (4), GIR (4)] drives
pub struct SpringFeedForward {
    // torque per drive per radian of [azimuth, elevation, GIR]
    stiffness: [f64; 3],
    // # of drives of [azimuth, elevation, GIR]
    n_drives: [usize; 3],
    setpoint: [f64; 3],
}
impl SpringFeedForward {
    pub fn from_fem(fem: &FEM) -> anyhow::Result<Self> {
        let encoders = DriveDcGain::new(fem)?.encoders(fem)?;
        let n_drives = [8, 4, 4];
        anyhow::ensure!(
            encoders[0].len() == n_drives.iter().sum::<usize>(),
            "expected {} mount drives, found {}",
            n_drives.iter().sum::<usize>(),
            encoders[0].len()
        );
        let mut first = 0;
        let stiffness = [0, 1, 2].map(|k| {
            let drives = first..first + n_drives[k];
            first += n_drives[k];
            1. / encoders[k][drives].iter().sum::<f64>()
        });
        Ok(Self {
            stiffness,
            n_drives,
            setpoint: [0f64; 3],
        })
    }
    /// Turns the feed-forward off (zero torques)
    pub fn off(mut self) -> Self {
        self.stiffness = [0f64; 3];
        self
    }
    /// Torque per drive per radian of [azimuth, elevation, GIR]
    pub fn stiffness(&self) -> [f64; 3] {
        self.stiffness
    }
}
impl Update for SpringFeedForward {}
impl Read<MountSetPoint> for SpringFeedForward {
    fn read(&mut self, data: Data<MountSetPoint>) {
        self.setpoint.copy_from_slice(&data[..3]);
    }
}
impl Write<MountTorquesFeedForward> for SpringFeedForward {
    fn write(&mut self) -> Option<Data<MountTorquesFeedForward>> {
        Some(Data::new(
            (0..3)
                .flat_map(|k| vec![self.stiffness[k] * self.setpoint[k]; self.n_drives[k]])
                .collect(),
        ))
    }
}
