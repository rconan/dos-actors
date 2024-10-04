use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use serde::{Deserialize, Serialize};

use super::CalibrationMode;

#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct MirrorMode([Option<CalibrationMode>; 7]);
impl Deref for MirrorMode {
    type Target = [Option<CalibrationMode>; 7];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for MirrorMode {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl MirrorMode {
    /// Create a calibration mode for a GMT [mirror](CalibrationMode::Mirror)
    pub fn new(mirror: [Option<CalibrationMode>; 7]) -> Self {
        Self(mirror)
    }
    /// Update the mode of segment # `sid`
    pub fn update(mut self, (sid, mode): (u8, CalibrationMode)) -> Self {
        assert!(
            sid > 0 && sid <= 7,
            "Segment id={sid} must be between 1 and 7"
        );
        self[sid as usize - 1] = Some(mode);
        self
    }
}

impl Display for MirrorMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:}",
            self.iter()
                .enumerate()
                .filter_map(|(i, segment)| segment.as_ref().map(|s| format!("S{}[{}]", i + 1, s)))
                .collect::<Vec<_>>()
                .join(",")
        )
    }
}

impl From<CalibrationMode> for MirrorMode {
    fn from(value: CalibrationMode) -> Self {
        Self([Some(value); 7])
    }
}
