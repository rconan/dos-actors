use std::ops::{Add, AddAssign};

use crate::{Data, Left, Read, Right, Update, Write};

use super::{M1State, M2State, OpticsState};

mod mirror;
mod segment;

pub use mirror::MirrorState;
pub use segment::SegmentState;

/// M1 and M2 segment optical states
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Default, Debug, Clone, PartialEq)]
pub struct OpticalState {
    pub m1: Option<MirrorState>,
    pub m2: Option<MirrorState>,
    pub zero_point: Option<Box<OpticalState>>,
}
impl OpticalState {
    /// Creates a new [OpticalState] from M1 and M2 [MirrorState]
    pub fn new(m1: MirrorState, m2: MirrorState) -> Self {
        Self {
            m1: Some(m1),
            m2: Some(m2),
            zero_point: None,
        }
    }
    /// Sets the optical state zero point
    pub fn zero_point(mut self, zp: OpticalState) -> Self {
        self.zero_point = Some(zp.into());
        self
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

impl Update for OpticalState {
    fn update(&mut self) {
        if let Some(zero_point) = self.zero_point.as_ref() {
            let OpticalState { m1, m2, .. } = zero_point.as_ref() + &self;
            self.m1 = m1;
            self.m2 = m2;
        }
    }
}

impl Read<M1State> for OpticalState {
    fn read(&mut self, data: Data<M1State>) {
        self.m1 = Some(data.into_arc().as_ref().clone());
    }
}
impl Read<M2State> for OpticalState {
    fn read(&mut self, data: Data<M2State>) {
        self.m2 = Some(data.into_arc().as_ref().clone());
    }
}
impl Write<Right<M1State>> for OpticalState {
    fn write(&mut self) -> Option<Data<Right<M1State>>> {
        <_ as Write<M1State>>::write(self).map(|x| x.transmute())
    }
}
impl Write<Right<M2State>> for OpticalState {
    fn write(&mut self) -> Option<Data<Right<M2State>>> {
        <_ as Write<M2State>>::write(self).map(|x| x.transmute())
    }
}
impl Write<Left<M1State>> for OpticalState {
    fn write(&mut self) -> Option<Data<Left<M1State>>> {
        <_ as Write<M1State>>::write(self).map(|x| x.transmute())
    }
}
impl Write<Left<M2State>> for OpticalState {
    fn write(&mut self) -> Option<Data<Left<M2State>>> {
        <_ as Write<M2State>>::write(self).map(|x| x.transmute())
    }
}
impl Write<M1State> for OpticalState {
    fn write(&mut self) -> Option<Data<M1State>> {
        self.m1.as_ref().map(|x| Data::new(x.to_owned()))
    }
}
impl Write<M2State> for OpticalState {
    fn write(&mut self) -> Option<Data<M2State>> {
        self.m2.as_ref().map(|x| Data::new(x.to_owned()))
    }
}

impl Write<OpticsState> for OpticalState {
    fn write(&mut self) -> Option<Data<OpticsState>> {
        Some(Data::new(self.clone()))
    }
}
impl Read<OpticsState> for OpticalState {
    fn read(&mut self, data: Data<OpticsState>) {
        let state = &*data;
        *self = state.clone();
    }
}
impl Write<Right<OpticsState>> for OpticalState {
    fn write(&mut self) -> Option<Data<Right<OpticsState>>> {
        Some(Data::new(self.clone()))
    }
}
impl Write<Left<OpticsState>> for OpticalState {
    fn write(&mut self) -> Option<Data<Left<OpticsState>>> {
        Some(Data::new(self.clone()))
    }
}

impl Add for OpticalState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            m1: match (self.m1, rhs.m1) {
                (None, None) => None,
                (None, Some(s1)) => Some(s1),
                (Some(s1), None) => Some(s1),
                (Some(s1), Some(rhs_s1)) => Some(s1 + rhs_s1),
            },
            m2: match (self.m2, rhs.m2) {
                (None, None) => None,
                (None, Some(s2)) => Some(s2),
                (Some(s2), None) => Some(s2),
                (Some(s2), Some(rhs_s2)) => Some(s2 + rhs_s2),
            },
            zero_point: None,
        }
    }
}
impl Add for &OpticalState {
    type Output = OpticalState;

    fn add(self, rhs: Self) -> Self::Output {
        OpticalState {
            m1: match (&self.m1, &rhs.m1) {
                (None, None) => None,
                (None, Some(s1)) => Some(s1.clone()),
                (Some(s1), None) => Some(s1.clone()),
                (Some(s1), Some(rhs_s1)) => Some(s1 + rhs_s1),
            },
            m2: match (&self.m2, &rhs.m2) {
                (None, None) => None,
                (None, Some(s2)) => Some(s2.clone()),
                (Some(s2), None) => Some(s2.clone()),
                (Some(s2), Some(rhs_s2)) => Some(s2 + rhs_s2),
            },
            zero_point: None,
        }
    }
}
