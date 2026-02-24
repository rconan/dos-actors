use std::ops::{Add, Mul, Sub};

use crate::MirrorState;

impl Add for MirrorState {
    type Output = MirrorState;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            segment: self
                .segment
                .into_iter()
                .zip(rhs.segment.into_iter())
                .map(|(sl, sr)| match (sl, sr) {
                    (None, None) => None,
                    (None, Some(value)) => Some(value),
                    (Some(value), None) => Some(value),
                    (Some(sl), Some(sr)) => Some(sl + sr),
                })
                .collect(),
        }
    }
}
impl Add for &MirrorState {
    type Output = MirrorState;

    fn add(self, rhs: Self) -> Self::Output {
        MirrorState {
            segment: self
                .segment
                .iter()
                .zip(rhs.segment.iter())
                .map(|(sl, sr)| match (sl, sr) {
                    (None, None) => None,
                    (None, Some(value)) => Some(value.clone()),
                    (Some(value), None) => Some(value.clone()),
                    (Some(sl), Some(sr)) => Some(sl + sr),
                })
                .collect(),
        }
    }
}
impl Sub for MirrorState {
    type Output = MirrorState;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            segment: self
                .segment
                .into_iter()
                .zip(rhs.segment.into_iter())
                .map(|(sl, sr)| match (sl, sr) {
                    (None, None) => None,
                    (None, Some(value)) => Some(-value),
                    (Some(value), None) => Some(value),
                    (Some(sl), Some(sr)) => Some(sl - sr),
                })
                .collect(),
        }
    }
}
impl Sub for &MirrorState {
    type Output = MirrorState;

    fn sub(self, rhs: Self) -> Self::Output {
        MirrorState {
            segment: self
                .segment
                .iter()
                .zip(rhs.segment.iter())
                .map(|(sl, sr)| match (sl, sr) {
                    (None, None) => None,
                    (None, Some(value)) => Some(-value.clone()),
                    (Some(value), None) => Some(value.clone()),
                    (Some(sl), Some(sr)) => Some(sl - sr),
                })
                .collect(),
        }
    }
}
impl Mul<f64> for MirrorState {
    type Output = MirrorState;

    fn mul(self, rhs: f64) -> Self::Output {
        self.segment
            .into_iter()
            .map(|segment| segment.map(|segment| segment * rhs))
            .collect()
    }
}
impl Mul<f64> for &MirrorState {
    type Output = MirrorState;

    fn mul(self, rhs: f64) -> Self::Output {
        self.segment
            .iter()
            .map(|segment| segment.as_ref().map(|segment| segment * rhs))
            .collect()
    }
}
