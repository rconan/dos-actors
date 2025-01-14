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
        m1_segment_figure: Option<&'a mut M1SegmentFigure>,
    ) -> Result<Self, gmt_dos_clients_fem::StateSpaceError>
    where
        Self: 'a + Sized,
    {
        if m1_segment_figure.is_none() {
            return Ok(self);
        }
        self.set_m1_figure_nodes()?
            .outs_by_name((1..=7).map(|i| format!("M1_segment_{i}_axial_d")).collect())
    }
}
