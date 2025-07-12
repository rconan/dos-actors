use crate::{Data, Left, Read, Right, Update, Write};

use super::{M1State, M2State};

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

impl Update for OpticalState {
    fn update(&mut self) {}
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
