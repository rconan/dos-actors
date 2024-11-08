use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use serde::{Deserialize, Serialize};

use super::CalibrationMode;

/// A full set of [segment mode](CalibrationMode)
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
    /// Create a calibration mode for a GMT mirror
    ///
    /// A missing segment has is entry set to [None]
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
    /// Remove segment # `sid`
    ///
    /// The segment is still there but the calibration mode
    /// associated with it is set to [None]
    pub fn remove(mut self, sid: u8) -> Self {
        assert!(
            sid > 0 && sid <= 7,
            "Segment id={sid} must be between 1 and 7"
        );
        self[sid as usize - 1] = None;
        self
    }
    pub fn iter(&self) -> impl Iterator<Item = &Option<CalibrationMode>> {
        self.0.iter()
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

impl<const N: usize> From<[CalibrationMode; N]> for MirrorMode {
    fn from(value: [CalibrationMode; N]) -> Self {
        let mut mirror = [None; 7];
        mirror
            .iter_mut()
            .zip(value.iter())
            .for_each(|(m, s)| *m = Some(*s));
        Self(mirror)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::Modality;

    #[test]
    fn rbm_mirror() {
        let mode = MirrorMode::new([
            Some(CalibrationMode::t_z(1.).into()),
            None,
            None,
            None,
            Some(CalibrationMode::t_z(1.).into()),
            None,
            Some(CalibrationMode::t_z(0.).into()),
        ]);
        let data = vec![1., 5.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(
            filled,
            [0., 0., 1., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        );
    }

    #[test]
    fn modes_mirror() {
        let mode = MirrorMode::new([
            Some(CalibrationMode::modes(3, 1.).into()),
            None,
            Some(CalibrationMode::modes(3, 0.).into()),
            None,
            Some(CalibrationMode::modes(3, 1.).into()),
            None,
            Some(CalibrationMode::modes(3, 0.).into()),
        ]);
        let data = vec![1., 2., 3., 4., 5., 6.];
        let filled = mode.fill(data.into_iter());
        assert_eq!(
            filled,
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0,]
        );
    }
}
