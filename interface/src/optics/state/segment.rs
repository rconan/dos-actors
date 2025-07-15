use std::{
    ops::{Add, Mul, Neg, Sub},
    sync::Arc,
};

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
    /// Sets a particular mode
    pub fn set_mode(mut self, i: usize, value: f64) -> Self {
        let mut modes = self.modes.as_ref().unwrap().to_vec();
        modes[i] = value;
        self.modes = Some(Arc::new(modes));
        self
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
            rbms: match (self.rbms, rhs.rbms) {
                (None, None) => None,
                (None, Some(value)) => Some(value),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a + b)
                        .collect::<Vec<_>>(),
                )
                .map(|rbms| Arc::new(rbms)),
            },
            modes: match (self.modes, rhs.modes) {
                (None, None) => None,
                (None, Some(value)) => Some(value),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a + b)
                        .collect::<Vec<_>>(),
                )
                .map(|modes| Arc::new(modes)),
            },
        }
    }
}
impl Sub for SegmentState {
    type Output = SegmentState;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            rbms: match (self.rbms, rhs.rbms) {
                (None, None) => None,
                (None, Some(value)) => Some(value.iter().map(|x| *x * -1f64).collect::<Vec<_>>())
                    .map(|rbms| Arc::new(rbms)),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a - b)
                        .collect::<Vec<_>>(),
                )
                .map(|rbms| Arc::new(rbms)),
            },
            modes: match (self.modes, rhs.modes) {
                (None, None) => None,
                (None, Some(value)) => Some(value.iter().map(|x| *x * -1f64).collect::<Vec<_>>())
                    .map(|rbms| Arc::new(rbms)),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a - b)
                        .collect::<Vec<_>>(),
                )
                .map(|modes| Arc::new(modes)),
            },
        }
    }
}
impl Add for &SegmentState {
    type Output = SegmentState;

    fn add(self, rhs: Self) -> Self::Output {
        SegmentState {
            rbms: match (self.rbms.clone(), rhs.rbms.clone()) {
                (None, None) => None,
                (None, Some(value)) => Some(value),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a + b)
                        .collect::<Vec<_>>(),
                )
                .map(|rbms| Arc::new(rbms)),
            },
            modes: match (self.modes.clone(), rhs.modes.clone()) {
                (None, None) => None,
                (None, Some(value)) => Some(value),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a + b)
                        .collect::<Vec<_>>(),
                )
                .map(|modes| Arc::new(modes)),
            },
        }
    }
}
impl Sub for &SegmentState {
    type Output = SegmentState;

    fn sub(self, rhs: Self) -> Self::Output {
        SegmentState {
            rbms: match (self.rbms.clone(), rhs.rbms.clone()) {
                (None, None) => None,
                (None, Some(value)) => Some(value.iter().map(|x| *x * -1f64).collect::<Vec<_>>())
                    .map(|rbms| Arc::new(rbms)),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a - b)
                        .collect::<Vec<_>>(),
                )
                .map(|rbms| Arc::new(rbms)),
            },
            modes: match (self.modes.clone(), rhs.modes.clone()) {
                (None, None) => None,
                (None, Some(value)) => Some(value.iter().map(|x| *x * -1f64).collect::<Vec<_>>())
                    .map(|rbms| Arc::new(rbms)),
                (Some(value), None) => Some(value),
                (Some(l), Some(r)) => Some(
                    l.iter()
                        .zip(r.iter())
                        .map(|(&a, &b)| a - b)
                        .collect::<Vec<_>>(),
                )
                .map(|modes| Arc::new(modes)),
            },
        }
    }
}

impl Mul<f64> for SegmentState {
    type Output = SegmentState;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            rbms: self
                .rbms
                .map(|rbms| rbms.iter().map(|&a| a * rhs).collect::<Vec<_>>())
                .map(|rbms| Arc::new(rbms)),
            modes: self
                .modes
                .map(|modes| modes.iter().map(|&a| a * rhs).collect::<Vec<_>>())
                .map(|modes| Arc::new(modes)),
        }
    }
}

impl Neg for SegmentState {
    type Output = Self;

    fn neg(self) -> Self::Output {
        self * -1f64
    }
}
