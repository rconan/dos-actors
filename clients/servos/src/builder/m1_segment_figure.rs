use gmt_dos_clients_fem::{solvers::ExponentialMatrix, DiscreteStateSpace};
use nalgebra as na;

use super::Include;

/**
M1 figure builder

The M1 figure builder adds the following outputs to the FEM:
* [`M1_segment_1_axial_d`](gmt_dos_clients_io::fem::M1_segment_1_axial_d)
* [`M1_segment_2_axial_d`](gmt_dos_clients_io::fem::M1_segment_2_axial_d)
* [`M1_segment_3_axial_d`](gmt_dos_clients_io::fem::M1_segment_3_axial_d)
* [`M1_segment_4_axial_d`](gmt_dos_clients_io::fem::M1_segment_4_axial_d)
* [`M1_segment_5_axial_d`](gmt_dos_clients_io::fem::M1_segment_5_axial_d)
* [`M1_segment_6_axial_d`](gmt_dos_clients_io::fem::M1_segment_6_axial_d)
* [`M1_segment_7_axial_d`](gmt_dos_clients_io::fem::M1_segment_7_axial_d)

Per default, the rigid body motions are removed from the mirror figures.
 **/
#[derive(Debug, Clone, Default)]
pub struct M1SegmentFigure {
    transforms: Option<Vec<na::DMatrix<f64>>>,
}

impl M1SegmentFigure {
    /// Creates a new [M1SegmentFigure] builder instance
    pub fn new() -> Self {
        Default::default()
    }
    /// Sets the matrices that will process the mirror figures
    ///
    /// The removal of the rigid body motion is disabled.
    pub fn transforms(mut self, transforms: Vec<na::DMatrix<f64>>) -> Self {
        self.transforms = Some(transforms);
        self
    }
    pub(crate) fn transforms_view<'a>(&'a mut self) -> Option<Vec<na::DMatrixView<'a, f64>>> {
        self.transforms
            .as_ref()
            .map(|transforms| transforms.iter().map(|t| t.as_view()).collect())
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
        let Some(m1_segment_figure) = m1_segment_figure else {
            return Ok(self);
        };
        if let Some(transforms) = m1_segment_figure.transforms_view() {
            self.outs_with_by_name(
                (1..=7).map(|i| format!("M1_segment_{i}_axial_d")).collect(),
                transforms,
            )
        } else {
            self.set_m1_figure_nodes()?
                .outs_by_name((1..=7).map(|i| format!("M1_segment_{i}_axial_d")).collect())
        }
    }
}
