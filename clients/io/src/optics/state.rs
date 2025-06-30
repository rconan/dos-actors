use std::{ops::Add, sync::Arc};

/// M1 and M2 segment optical states
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Default, Debug, Clone, PartialEq)]
pub struct OpticalState {
    pub m1: Option<MirrorState>,
    pub m2: Option<MirrorState>,
}
impl OpticalState {
    /// Creates a new [OpticalState] from M1 and M2 [MirrorState]
    pub fn new(m1: MirrorState, m2: MirrorState) -> Self {
        Self {
            m1: Some(m1),
            m2: Some(m2),
        }
    }
    /// Creates a new [OpticalState] from M1 [MirrorState]
    pub fn m1(state: MirrorState) -> Self {
        Self {
            m1: Some(state),
            ..Default::default()
        }
    }
    /// Creates a new [OpticalState] from M2 [MirrorState]
    pub fn m2(state: MirrorState) -> Self {
        Self {
            m2: Some(state),
            ..Default::default()
        }
    }
    /// Returns a reference to M1 [MirrorState]
    pub fn m1_as_ref(&self) -> Option<&MirrorState> {
        self.m1.as_ref()
    }
    /// Returns a reference to M2 [MirrorState]
    pub fn m2_as_ref(&self) -> Option<&MirrorState> {
        self.m2.as_ref()
    }
    /// Returns a mutable reference to M1 [MirrorState]
    pub fn m1_as_mut(&mut self) -> Option<&mut MirrorState> {
        self.m1.as_mut()
    }
    /// Returns a mutable reference to M2 [MirrorState]
    pub fn m2_as_mut(&mut self) -> Option<&mut MirrorState> {
        self.m2.as_mut()
    }
}

mod mirror;
pub use mirror::MirrorState;

impl FromIterator<SegmentState> for MirrorState {
    fn from_iter<T: IntoIterator<Item = SegmentState>>(iter: T) -> Self {
        Self {
            segment: iter.into_iter().map(|x| Some(x)).collect(),
        }
    }
}

/// GMT mirror segment optical state (rigid body motion and surface figure)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Default, Clone, PartialEq)]
pub struct SegmentState {
    pub rbms: Option<Arc<Vec<f64>>>,
    pub modes: Option<Arc<Vec<f64>>>,
}
impl SegmentState {
    /// Creates a new GMT mirror [SegmentState] from
    /// the 6 rigid body motions and the surface figure
    /// modal or zonal coefficients
    pub fn new(rbms: impl Into<Vec<f64>>, modes: impl Into<Vec<f64>>) -> Self {
        Self {
            rbms: Some(Arc::new(rbms.into())),
            modes: Some(Arc::new(modes.into())),
        }
    }
    /// Creates a new GMT mirror [SegmentState] from
    /// the 6 rigid body motions
    pub fn modes(modes: impl Into<Vec<f64>>) -> Self {
        Self {
            modes: Some(Arc::new(modes.into())),
            ..Default::default()
        }
    }
    /// Creates a new GMT mirror [SegmentState] from
    /// the surface figure modal or zonal coefficients
    pub fn rbms(rbms: impl Into<Vec<f64>>) -> Self {
        Self {
            rbms: Some(Arc::new(rbms.into())),
            ..Default::default()
        }
    }
}
impl Add for SegmentState {
    type Output = SegmentState;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            rbms: self
                .rbms
                .zip(rhs.rbms)
                .map(|(rbms, rhs)| {
                    rbms.iter()
                        .zip(rhs.iter())
                        .map(|(&a, &b)| a + b)
                        .collect::<Vec<_>>()
                })
                .map(|rbms| Arc::new(rbms)),
            modes: self
                .modes
                .zip(rhs.modes)
                .map(|(modes, rhs)| {
                    modes
                        .iter()
                        .zip(rhs.iter())
                        .map(|(&a, &b)| a + b)
                        .collect::<Vec<_>>()
                })
                .map(|modes| Arc::new(modes)),
        }
    }
}
