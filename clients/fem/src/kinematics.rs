//! Kinematic compensation of the mount rigid body motion
//!
//! The FEM outputs include the rigid body motion of the telescope as the mount moves.
//! The kinematic compensation removes from each output `y` the motion that follows the
//! mount encoders: `y - K [azimuth, elevation, GIR]` where azimuth, elevation and GIR are
//! the averaged encoder angles of each axis.
//!
//! The kinematic gain `K` is derived from the DC gain of the state space model from the mount
//! drive torques: with `H` the DC gain to all the outputs and `E` the DC gain to the
//! averaged encoders of each axis, `K = H E⁺`.
//! The rows of `K` corresponding to the mount encoders are set to zero, the mount encoders
//! are left untouched for the mount control system.
//! Other outputs can be excluded from the compensation,
//! see [DiscreteStateSpace::kinematic_exclusion](crate::DiscreteStateSpace::kinematic_exclusion).
//! Outputs fed back to control systems should be compensated too, e.g. the M1 hardpoints
//! forces: they include a kinematic component that the M1 actuators would otherwise offload.
//!
//! The columns of `K` for some outputs can be replaced by a calibrated gain, e.g. from the
//! closed-loop response to a mount trajectory.

use std::ops::Range;

/// Kinematic compensation of the FEM outputs
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default)]
pub struct KinematicCompensation {
    /// kinematic gain `[n_outputs x 3]` row-wise
    pub(crate) gain: Vec<f64>,
    /// ranges of the azimuth, elevation and GIR encoders in the output vector
    pub(crate) encoders: [Range<usize>; 3],
}
impl KinematicCompensation {
    /// Returns the kinematic gain `[n_outputs x 3]` row-wise
    pub fn gain(&self) -> &[f64] {
        &self.gain
    }
    /// Averaged azimuth, elevation and GIR encoder angles
    pub fn angles(&self, y: &[f64]) -> [f64; 3] {
        self.encoders.clone().map(|r| {
            let n = r.len() as f64;
            y[r].iter().sum::<f64>() / n
        })
    }
    /// Removes the kinematic motion from the outputs `y`
    pub fn apply(&self, y: &mut [f64]) {
        let [az, el, gir] = self.angles(y);
        y.iter_mut()
            .zip(self.gain.chunks(3))
            .for_each(|(y, k)| *y -= k[0] * az + k[1] * el + k[2] * gir);
    }
}

/// Mount axes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountAxis {
    Azimuth,
    Elevation,
    Gir,
}
impl MountAxis {
    pub(crate) fn index(self) -> usize {
        match self {
            Self::Azimuth => 0,
            Self::Elevation => 1,
            Self::Gir => 2,
        }
    }
}

/// An output excluded from the kinematic compensation
pub(crate) struct KinematicExclusion {
    pub(crate) is_output: Box<dyn Fn(&dyn std::any::Any) -> bool>,
    pub(crate) name: String,
}

/// A calibrated kinematic gain for one output and one mount axis
pub(crate) struct KinematicCalibration {
    pub(crate) is_output: Box<dyn Fn(&dyn std::any::Any) -> bool>,
    pub(crate) name: String,
    pub(crate) axis: usize,
    pub(crate) gain: Vec<f64>,
}
