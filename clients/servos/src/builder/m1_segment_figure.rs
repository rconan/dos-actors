use gmt_dos_clients_fem::{DiscreteStateSpace, solvers::ExponentialMatrix};
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
#[derive(Debug, Default, Clone)]
pub struct M1SegmentFigure {
    transforms: Option<Vec<na::DMatrix<f64>>>,
    keep_rbm: bool,
}

impl M1SegmentFigure {
    /// Creates a new [M1SegmentFigure] builder instance
    pub fn new() -> Self {
        Default::default()
    }
    /// Sets the matrices that will process the mirror figures
    ///
    /// The removal of the rigid body motion is applied beform the matrix transforms.
    pub fn transforms(mut self, transforms: Vec<na::DMatrix<f64>>) -> Self {
        self.transforms = Some(transforms);
        self
    }
    /// Disables the removal of the rigid body motions for the mirror figures
    pub fn keep_rigid_body_motions(mut self) -> Self {
        self.keep_rbm = true;
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
        // if let Some(transforms) = m1_segment_figure.transforms_view() {
        //     self.outs_with_by_name(
        //         (1..=7).map(|i| format!("M1_segment_{i}_axial_d")).collect(),
        //         transforms,
        //     )
        // } else {
        //     self.set_m1_figure_nodes()?
        //         .outs_by_name((1..=7).map(|i| format!("M1_segment_{i}_axial_d")).collect())
        // }
        let names: Vec<_> = (1..=7).map(|i| format!("M1_segment_{i}_axial_d")).collect();
        match (
            m1_segment_figure.keep_rbm,
            m1_segment_figure.transforms_view(),
        ) {
            (true, None) => self.outs_by_name(names),
            (true, Some(transforms)) => self.outs_with_by_name(names, transforms),
            (false, None) => self.set_m1_figure_nodes()?.outs_by_name(names),
            (false, Some(transforms)) => self
                .set_m1_figure_nodes()?
                .outs_with_by_name(names, transforms),
        }
    }
}
