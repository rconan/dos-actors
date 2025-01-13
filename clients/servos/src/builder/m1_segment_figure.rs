use gmt_dos_clients_fem::{DiscreteStateSpace, ExponentialMatrix};

use super::Include;

#[derive(Debug, Clone, Default)]
pub struct M1SegmentFigure;

impl M1SegmentFigure {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'a> Include<'a, M1SegmentFigure> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(
        self,
        component: Option<&'a mut M1SegmentFigure>,
    ) -> Result<Self, gmt_dos_clients_fem::StateSpaceError>
    where
        Self: 'a + Sized,
    {
        self.outs_by_name((1..=7).map(|i| format!("M2_segment_{i}_axial_d")).collect())
    }
}
