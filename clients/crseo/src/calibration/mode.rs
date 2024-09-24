use std::ops::{Range, RangeInclusive};

use serde::{Deserialize, Serialize};

/// Selection of calibration modes per segment
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CalibrationMode {
    /// Rigid body motions as amplitudes of motion
    RBM([Option<f64>; 6]),
    /// Mirror shapes
    Modes {
        /// total number of modes
        n_mode: usize,
        /// mode amplitude
        stroke: f64,
        /// index of the 1st mode to calibrate
        start_idx: usize,
        /// last mode to calibrate
        end_id: Option<usize>,
    },
}
impl Default for CalibrationMode {
    fn default() -> Self {
        Self::RBM([None; 6])
    }
}
impl CalibrationMode {
    /// Sets the number of modes and the mode amplitude
    pub fn modes(n_mode: usize, stroke: f64) -> Self {
        Self::Modes {
            n_mode,
            stroke,
            start_idx: 0,
            end_id: None,
        }
    }
    /// Starts the calibration from the given mode
    pub fn start_from(self, id: usize) -> Self {
        if let Self::Modes {
            n_mode,
            stroke,
            end_id,
            ..
        } = self
        {
            Self::Modes {
                n_mode,
                stroke,
                start_idx: id - 1,
                end_id,
            }
        } else {
            self
        }
    }
    /// Ends the calibration at the given mode
    pub fn ends_at(self, id: usize) -> Self {
        if let Self::Modes {
            n_mode,
            stroke,
            start_idx,
            ..
        } = self
        {
            Self::Modes {
                n_mode,
                stroke,
                start_idx,
                end_id: Some(id),
            }
        } else {
            self
        }
    }
    /// Returns the number of modes
    pub fn n_mode(&self) -> usize {
        match self {
            Self::RBM(_) => 6,
            Self::Modes { n_mode, .. } => *n_mode,
        }
    }
    /// Returns the number of modes that are used for calibration
    pub fn calibration_n_mode(&self) -> usize {
        match self {
            Self::RBM(rbms) => rbms.iter().filter_map(|x| x.as_ref()).count(),
            Self::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => {
                let end = end_id.unwrap_or(*n_mode);
                end - start_idx
            }
        }
    }
    /// Returns the indices as the range of modes to calibrate
    pub fn range(&self) -> Range<usize> {
        match self {
            Self::RBM(_) => 0..6,
            Self::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => {
                let end = end_id.unwrap_or(*n_mode);
                *start_idx..end
            }
        }
    }
    /// Returns the mode number as the range of modes to calibrate
    pub fn mode_range(&self) -> RangeInclusive<usize> {
        match self {
            Self::RBM(_) => 1..=6,
            Self::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => {
                let start = *start_idx + 1;
                let end = end_id.unwrap_or(*n_mode);
                start..=end
            }
        }
    }
    /// Returns an iterator over the command vector
    pub fn command(&self) -> Box<dyn Iterator<Item = Vec<f64>> + '_> {
        match self {
            CalibrationMode::RBM(rbms) => {
                Box::new(rbms.iter().enumerate().filter_map(|(i, rbm)| {
                    rbm.map(|v| {
                        let mut cmd = vec![0.0; 6];
                        cmd[i] = v;
                        cmd
                    })
                }))
            }
            CalibrationMode::Modes { n_mode, stroke, .. } => Box::new(self.range().map(|i| {
                let mut cmd = vec![0.0; *n_mode];
                cmd[i] = *stroke;
                cmd
            })),
        }
    }
    /// Returns an iterator over both the stroke and the command vector
    pub fn stroke_command(&self) -> Box<dyn Iterator<Item = (f64, Vec<f64>)> + '_> {
        match self {
            CalibrationMode::RBM(rbms) => {
                Box::new(rbms.iter().enumerate().filter_map(|(i, rbm)| {
                    rbm.map(|v| {
                        let mut cmd = vec![0.0; 6];
                        cmd[i] = v;
                        (v, cmd)
                    })
                }))
            }
            CalibrationMode::Modes { n_mode, stroke, .. } => Box::new(self.range().map(|i| {
                let mut cmd = vec![0.0; *n_mode];
                cmd[i] = *stroke;
                (*stroke, cmd)
            })),
        }
    }
}
