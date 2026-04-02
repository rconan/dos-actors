use std::ops::Add;

use interface::{Data, Left, Read, Right, TimerMarker, Update, Write};

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
}
impl TimerMarker for OpticalState {}
impl OpticalState {
    /// Creates a new [OpticalState] from M1 and M2 [MirrorState]
    pub fn new(m1: impl Into<MirrorState>, m2: impl Into<MirrorState>) -> Self {
        Self {
            m1: Some(m1.into()),
            m2: Some(m2.into()),
        }
    }
    /// Sets the optical state zero point
    pub fn set_zero_point(self, zero_point: OpticalState) -> Self {
        Self {
            m1: if let Some(zero_point) = zero_point.m1 {
                Some(self.m1.unwrap_or_default().set_zero_point(zero_point))
            } else {
                self.m1
            },
            m2: if let Some(zero_point) = zero_point.m2 {
                Some(self.m2.unwrap_or_default().set_zero_point(zero_point))
            } else {
                self.m2
            },
        }
    }
    /// Gets the optical state zero point
    pub fn get_zero_point(&self) -> Option<Self> {
        let m1 = self.m1.as_ref().and_then(|m1| m1.get_zero_point());
        let m2 = self.m2.as_ref().and_then(|m2| m2.get_zero_point());
        if m1.is_none() && m2.is_none() {
            None
        } else {
            Some(Self { m1, m2 })
        }
    }
    /// Creates a new [OpticalState] from M1 [MirrorState]
    pub fn m1(state: impl Into<MirrorState>) -> Self {
        Self {
            m1: Some(state.into()),
            ..Default::default()
        }
    }
    /// Creates a new [OpticalState] from M2 [MirrorState]
    pub fn m2(state: impl Into<MirrorState>) -> Self {
        Self {
            m2: Some(state.into()),
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

impl Update for OpticalState {}

impl Read<M1State> for OpticalState {
    fn read(&mut self, data: Data<M1State>) {
        if let Some(zero_point) = self.get_zero_point()
            && let OpticalState { m1: Some(m1_0), .. } = zero_point
        {
            self.m1 = Some(data.into_arc().as_ref() + &m1_0);
        } else {
            self.m1 = Some(data.into_arc().as_ref().clone());
        }
    }
}
impl Read<M2State> for OpticalState {
    fn read(&mut self, data: Data<M2State>) {
        if let Some(zero_point) = self.get_zero_point()
            && let OpticalState { m2: Some(m2_0), .. } = zero_point
        {
            self.m2 = Some(data.into_arc().as_ref() + &m2_0);
        } else {
            self.m2 = Some(data.into_arc().as_ref().clone());
        }
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
        *self = if let Some(zero_point) = self.get_zero_point().take() {
            (&*data + &zero_point).set_zero_point(zero_point)
        } else {
            (&*data).clone()
        };
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
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add() {
        let state1 = OpticalState::m1(
            MirrorState::default()
                .zeros_modes(5)
                .set_segment_state(1, SegmentState::modes(vec![0f64; 6]).set_mode(2, -2.)),
        );
        let mut state = OpticalState::m1(MirrorState::default().zeros_modes(6)).set_zero_point(
            OpticalState::m1(
                MirrorState::default()
                    .zeros_modes(6)
                    .set_segment_state(1, SegmentState::modes(vec![0f64; 6]).set_mode(5, 1.)),
            ),
        );
        <_ as Read<OpticsState>>::read(&mut state, Data::new(state1));
        dbg!(&state);
    }

    #[test]
    fn interface() {
        let m1 = MirrorState::default();
        let m2 = MirrorState::default()
            .set_segment_state(1, SegmentState::rbms([1e-6, 0., 0., 0., 0., 0.]));
        let mut optical_state = OpticalState::m1(MirrorState::default().zeros_modes(3))
            .set_zero_point(OpticalState::new(&m1, &m2));

        <_ as Read<OpticsState>>::read(&mut optical_state, Data::new(Default::default()));
        let reference = OpticalState::m2(&m2).set_zero_point(OpticalState::m2(m2));
        assert_eq!(reference, optical_state);
    }
}
