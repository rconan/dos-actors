use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use serde::{Deserialize, Serialize};

use crate::calibration::{CalibrationMode, Modality};

use super::MirrorMode;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MixedMode {
    Calibration(CalibrationMode),
    Mirror(MirrorMode),
}
impl Default for MixedMode {
    fn default() -> Self {
        Self::Calibration(Default::default())
    }
}
impl Display for MixedMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MixedMode::Calibration(calibration_mode) => write!(f, "{}", calibration_mode),
            MixedMode::Mirror(mirror_mode) => write!(f, "{}", mirror_mode),
        }
    }
}
impl Modality for MixedMode {
    fn n_cols(&self) -> usize {
        match self {
            MixedMode::Calibration(calibration_mode) => calibration_mode.n_cols(),
            MixedMode::Mirror(mirror_mode) => mirror_mode.n_cols(),
        }
    }

    fn fill(&self, iter: impl Iterator<Item = f64>) -> Vec<f64> {
        match self {
            MixedMode::Calibration(calibration_mode) => calibration_mode.fill(iter),
            MixedMode::Mirror(mirror_mode) => mirror_mode.fill(iter),
        }
    }
}

/// A set of [MirrorMode]s
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct MixedMirrorMode(Vec<MixedMode>);
impl Deref for MixedMirrorMode {
    type Target = [MixedMode];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for MixedMirrorMode {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Display for MixedMirrorMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:}",
            self.iter()
                .map(|x| x.to_string())
                .collect::<Vec<_>>()
                .join(",")
        )
    }
}

impl From<CalibrationMode> for MixedMirrorMode {
    fn from(value: CalibrationMode) -> Self {
        Self(vec![MixedMode::Calibration(value)])
    }
}
impl From<MirrorMode> for MixedMirrorMode {
    fn from(value: MirrorMode) -> Self {
        Self(vec![MixedMode::Mirror(value)])
    }
}
impl From<Vec<MirrorMode>> for MixedMirrorMode {
    fn from(value: Vec<MirrorMode>) -> Self {
        Self(value.into_iter().map(|m| MixedMode::Mirror(m)).collect())
    }
}
