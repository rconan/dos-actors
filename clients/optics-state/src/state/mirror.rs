mod interface;
mod ops;

use std::sync::Arc;

use super::SegmentState;

/// GMT mirror optical state (rigid body motion and surface figure)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct MirrorState {
    pub(crate) segment: Vec<Option<SegmentState>>,
}
impl Default for MirrorState {
    fn default() -> Self {
        Self {
            segment: vec![None; 7],
        }
    }
}

impl FromIterator<SegmentState> for MirrorState {
    fn from_iter<T: IntoIterator<Item = SegmentState>>(iter: T) -> Self {
        Self {
            segment: iter.into_iter().map(|x| Some(x)).collect(),
        }
    }
}
impl FromIterator<Option<SegmentState>> for MirrorState {
    fn from_iter<T: IntoIterator<Item = Option<SegmentState>>>(iter: T) -> Self {
        Self {
            segment: iter.into_iter().map(|x| x).collect(),
        }
    }
}
impl MirrorState {
    /// Creates a new [MirrorState] from rigid body motion and segment figure modes
    pub fn new<U, V, R, M>(rbms: R, modes: M) -> Self
    where
        U: Into<Vec<f64>>,
        R: IntoIterator<Item = U>,
        V: Into<Vec<f64>>,
        M: IntoIterator<Item = V>,
    {
        rbms.into_iter()
            .zip(modes)
            .map(|(rbms, modes)| SegmentState::new(rbms, modes))
            .collect()
    }
    /// Creates a new [MirrorState] with all 42 rigid body motions zeroed
    /// and no segment modes
    pub fn rbms() -> Self {
        Self {
            segment: vec![Some(SegmentState::rbms(vec![0f64; 6])); 7],
        }
    }
    /// Creates a new [MirrorState] from the 42 rigid body motions
    pub fn from_rbms(rbms: &[f64]) -> Self {
        assert_eq!(rbms.len(), 42);
        Self {
            segment: rbms
                .chunks(6)
                .map(|rbms| SegmentState::rbms(rbms.to_vec()))
                .map(|x| Some(x))
                .collect(),
        }
    }
    pub fn set_segment_state(mut self, sid: u8, state: SegmentState) -> Self {
        self.segment[sid as usize - 1] = Some(state);
        self
    }
    /// Returns an iterator over each segment
    pub fn iter(&self) -> impl Iterator<Item = Option<&SegmentState>> {
        self.segment.iter().map(|segment| segment.as_ref())
    }
    pub fn into_iter(self) -> impl Iterator<Item = Option<SegmentState>> {
        self.segment.into_iter()
    }
    /// Returns a mutable iterator over each segment
    pub fn iter_mut(&mut self) -> impl Iterator<Item = Option<&mut self::SegmentState>> {
        self.segment.iter_mut().map(|segment| segment.as_mut())
    }
    /// Returns the mirror rigid body motions
    pub fn into_rbms(&self) -> Option<Vec<f64>> {
        let data: Vec<_> = self.rbms_into_iter().collect();
        if data.is_empty() { None } else { Some(data) }
    }
    /// Returns the mirror modes
    // pub fn into_modes(&self) -> Option<Vec<f64>> {
    //     let data: Vec<_> = self.modes_into_iter().collect();
    //     if data.is_empty() { None } else { Some(data) }
    // }
    /// Returns an iterator over the mirror rigid body motions
    pub fn rbms_into_iter(&self) -> impl Iterator<Item = f64> {
        self.segment.iter().flat_map(|segment| {
            if let Some(SegmentState {
                rbms: Some(values), ..
            }) = &segment
            {
                values.as_slice().to_vec()
            } else {
                vec![0f64; 6]
            }
        })
    }
    /// Returns an iterator over the mirror modes
    pub fn modes_into_iter(&self) -> impl Iterator<Item = Option<Arc<Vec<f64>>>> {
        self.segment.iter().map(|segment| {
            if let Some(SegmentState {
                modes: Some(values),
                ..
            }) = &segment
            {
                Some(values.clone())
            } else {
                None
            }
        })
    }
    /// Sets `n` mirror modes to 0
    pub fn zeros_modes(mut self, n: usize) -> Self {
        self.segment.iter_mut().for_each(|segment| {
            *segment = Some(SegmentState::modes(vec![0f64; n]));
        });
        self
    }
    /// Sets the mirror state zero point
    pub fn set_zero_point(self, zero_point: MirrorState) -> Self {
        Self {
            segment: zero_point
                .into_iter()
                .zip(self.segment.into_iter())
                .map(|(zp_s, s)| {
                    if let Some(zp_s) = zp_s {
                        Some(s.unwrap_or_default().set_zero_point(zp_s))
                    } else {
                        s
                    }
                })
                .collect(),
        }
    }
    /// Gets the mirror state zero point
    pub fn get_zero_point(&self) -> Option<MirrorState> {
        self.segment
            .iter()
            .map(|s| s.as_ref().map(|s| s.get_zero_point()))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn default() {
        let mirror = MirrorState::default();
        assert!(mirror.get_zero_point().is_none());
    }
}
