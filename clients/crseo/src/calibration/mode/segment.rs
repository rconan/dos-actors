use std::{
    fmt::Display,
    ops::{Range, RangeInclusive},
};

use serde::{Deserialize, Serialize};

/// Selection of calibration modes per segment
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
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
    // Mirror([Option<Box<CalibrationMode>>; 7]),
}

impl Default for CalibrationMode {
    fn default() -> Self {
        Self::RBM([None; 6])
    }
}
impl Display for CalibrationMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CalibrationMode::RBM(rbms) => write!(
                f,
                "{:}",
                rbms.iter()
                    .zip(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"])
                    .filter_map(|(rbms, t)| rbms.and(Some(t)))
                    .collect::<Vec<_>>()
                    .join(",")
            )?,
            CalibrationMode::Modes { .. } => write!(f, "{:?}", self.mode_range())?,
            // CalibrationMode::Mirror(segments) => write!(
            //     f,
            //     "{:}",
            //     segments
            //         .iter()
            //         .enumerate()
            //         .filter_map(|(i, segment)| segment.as_ref().map(|s| format!(
            //             "S{}[{}]",
            //             i + 1,
            //             s
            //         )))
            //         .collect::<Vec<_>>()
            //         .join(",")
            // )?,
        };
        Ok(())
    }
}
impl CalibrationMode {
    /// Create a Tx & Ty [rigid body motions](CalibrationMode::RBM) calibration mode specifying the RBM amplitude
    pub fn t_xy(stroke: f64) -> Self {
        Self::RBM([
            (stroke.abs() > 0.).then_some(stroke),
            (stroke.abs() > 0.).then_some(stroke),
            None,
            None,
            None,
            None,
        ])
    }
    /// Create a Tz [rigid body motions](CalibrationMode::RBM) calibration mode specifying the RBM amplitude
    pub fn t_z(stroke: f64) -> Self {
        Self::RBM([
            None,
            None,
            (stroke.abs() > 0.).then_some(stroke),
            None,
            None,
            None,
        ])
    }
    /// Create a Rx & Ry [rigid body motions](CalibrationMode::RBM) calibration mode specifying the RBM amplitude
    pub fn r_xy(stroke: f64) -> Self {
        Self::RBM([
            None,
            None,
            None,
            (stroke.abs() > 0.).then_some(stroke),
            (stroke.abs() > 0.).then_some(stroke),
            None,
        ])
    }
    /// Create a Rz [rigid body motions](CalibrationMode::RBM) calibration mode specifying the RBM amplitude
    pub fn r_z(stroke: f64) -> Self {
        Self::RBM([
            None,
            None,
            None,
            None,
            None,
            (stroke.abs() > 0.).then_some(stroke),
        ])
    }
    /// Sets the number of modes and the mode amplitude
    pub fn modes(n_mode: usize, stroke: f64) -> Self {
        Self::Modes {
            n_mode,
            stroke,
            start_idx: (stroke.abs() > 0.).then_some(0).unwrap_or(n_mode),
            end_id: None,
        }
    }
    // /// Create a calibration mode for a GMT segment [mirror](CalibrationMode::Mirror)
    // pub fn mirror(mirror: Option<[Option<Box<CalibrationMode>>; 7]>) -> Self {
    //     if let Some(mirror) = mirror {
    //         Self::Mirror(mirror)
    //     } else {
    //         Self::Mirror([None, None, None, None, None, None, None])
    //     }
    // }
    // /// Update the mode of segment # `sid`
    // pub fn update(mut self, (sid, mode): (u8, CalibrationMode)) -> Self {
    //     match &mut self {
    //         Self::Mirror(segments) => segments[sid as usize - 1] = Some(Box::new(mode)),
    //         _ => (),
    //     };
    //     self
    // }
    /// Sets the amplitude of the given mode
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
            // &Self::Mirror(_) => todo!(),
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
            } // &Self::Mirror(_) => todo!(),
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
            } // &Self::Mirror(_) => todo!(),
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
            } // &Self::Mirror(_) => todo!(),
        }
    }
    /// Returns an iterator over the command vector
    pub fn command(&self) -> Box<dyn Iterator<Item = Vec<f64>> + '_> {
        match self {
            Self::RBM(rbms) => Box::new(rbms.iter().enumerate().filter_map(|(i, rbm)| {
                rbm.map(|v| {
                    let mut cmd = vec![0.0; 6];
                    cmd[i] = v;
                    cmd
                })
            })),
            Self::Modes { n_mode, stroke, .. } => Box::new(self.range().map(|i| {
                let mut cmd = vec![0.0; *n_mode];
                cmd[i] = *stroke;
                cmd
            })),
            // &Self::Mirror(_) => todo!(),
        }
    }
    /// Returns an iterator over both the stroke and the command vector
    pub fn stroke_command(&self) -> Box<dyn Iterator<Item = (f64, Vec<f64>)> + '_> {
        match self {
            Self::RBM(rbms) => Box::new(rbms.iter().enumerate().filter_map(|(i, rbm)| {
                rbm.map(|v| {
                    let mut cmd = vec![0.0; 6];
                    cmd[i] = v;
                    (v, cmd)
                })
            })),
            Self::Modes { n_mode, stroke, .. } => Box::new(self.range().map(|i| {
                let mut cmd = vec![0.0; *n_mode];
                cmd[i] = *stroke;
                (*stroke, cmd)
            })),
            // &Self::Mirror(_) => todo!(),
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::calibration::algebra::Modality;

    use super::*;

    #[test]
    fn rbm_zero() {
        let mode = CalibrationMode::RBM([None; 6]);
        let data = vec![];
        let filled = mode.fill(data.into_iter());
        assert_eq!(filled, [0.; 6]);
    }

    #[test]
    fn rbm_nofill() {
        let mode = CalibrationMode::RBM([Some(1.); 6]);
        let data = vec![1., 2., 3., 4., 5., 6.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(filled, [1., 2., 3., 4., 5., 6.]);
    }

    #[test]
    fn rbm_fill() {
        let mode = CalibrationMode::RBM([None, None, None, Some(1.), None, None]);
        let data = vec![4.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(filled, [0., 0., 0., 4., 0., 0.]);
    }

    #[test]
    fn modes_zero() {
        let mode = CalibrationMode::modes(6, 0.);
        let data = vec![];
        let filled = mode.fill(data.into_iter());
        assert_eq!(filled, [0.; 6]);
    }
    #[test]

    fn modes_nofill() {
        let mode = CalibrationMode::modes(6, 1.);
        let data = vec![1., 2., 3., 4., 5., 6.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(filled, [1., 2., 3., 4., 5., 6.]);
    }

    #[test]
    fn modes_fill() {
        let mode = CalibrationMode::modes(6, 1.).start_from(3).ends_at(5);
        let data = vec![4., 5., 6.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(filled, [0., 0., 4., 5., 6., 0.]);
    }

    #[test]
    fn rbm_mirror() {
        let mode = CalibrationMode::mirror(Some([
            Some(CalibrationMode::t_z(1.).into()),
            None,
            None,
            None,
            Some(CalibrationMode::t_z(1.).into()),
            None,
            Some(CalibrationMode::t_z(0.).into()),
        ]));
        let data = vec![1., 5.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(
            filled,
            [0., 0., 1., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        );
    }

    #[test]
    fn modes_mirror() {
        let mode = CalibrationMode::mirror(Some([
            Some(CalibrationMode::modes(3, 1.).into()),
            None,
            Some(CalibrationMode::modes(3, 0.).into()),
            None,
            Some(CalibrationMode::modes(3, 1.).into()),
            None,
            Some(CalibrationMode::modes(3, 0.).into()),
        ]));
        let data = vec![1., 2., 3., 4., 5., 6.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(
            filled,
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0,]
        );
    }
}