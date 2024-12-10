//! M1 CONTROL

#[doc(hidden)]
pub use super::prelude;
use super::prelude::*;
use gmt_dos_clients_io::{
    gmt_m1::{segment::ModeShapes, M1EdgeSensors, M1ModeShapes, M1RigidBodyMotions},
    Assembly,
};

pub mod actuators;
pub mod assembly;
pub mod hardpoints;
pub mod rigid_body_motions;

/* impl<S> Get<M1ModeShapes> for DiscreteModalSolver<S>
where
    S: Solver + Default,
{
    fn get(&self) -> Option<Vec<f64>> {
        let mut encoders = <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self)?;
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self)?.as_slice(),
        );
        Some(encoders)
    }
} */
impl<S> Write<M1ModeShapes> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1ModeShapes>> {
        let data: Vec<_> = <M1ModeShapes as Assembly>::SIDS
            .into_iter()
            .filter_map(|sid| match sid {
                1 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self),
                2 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self),
                3 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self),
                4 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self),
                5 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self),
                6 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self),
                7 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self),
                _ => panic!("expected segment id with [1,7], found {:}", sid),
            })
            .collect();
        if self.m1_figure_nodes.is_some() {
            let rbms = <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self)
                .expect("failed to get rigid body motion from ASMS reference bodies");
            self.m1_figure_nodes.as_mut().map(|m1_figure| {
                m1_figure
                    .from_assembly(<M1ModeShapes as Assembly>::SIDS.into_iter(), &data, &rbms)
                    .expect("failed to remove RBM from ASM m1_figure")
            })
        } else {
            Some(data)
        }
        .map(|x| x.into_iter().flatten().collect::<Vec<_>>().into())
    }
}
impl<const ID: u8, S> Write<ModeShapes<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<ModeShapes<ID>>> {
        let figure = match ID {
            1 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self),
            2 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self),
            3 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self),
            4 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self),
            5 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self),
            6 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self),
            7 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self),
            _ => unreachable!(),
        }?;
        if self.m1_figure_nodes.is_some() {
            let rbms = <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self)
                .expect("failed to get rigid body motion from M1 segments");
            self.m1_figure_nodes.as_mut().map(|m1_figure| {
                m1_figure
                    .from_segment(ID, &figure, &rbms)
                    .expect("failed to remove RBM from M1 segment #{ID}")
                    .into()
            })
        } else {
            Some(figure.into())
        }
    }
}
//  * M1 rigid body motions
impl<S> Size<M1RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        42
    }
}
impl<S> Write<M1RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self).map(|data| Data::new(data))
    }
}
impl<S> Write<M1EdgeSensors> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1EdgeSensors>> {
        <DiscreteModalSolver<S> as Get<fem_io::OSSM1EdgeSensors>>::get(self)
            .map(|data| Data::new(data))
    }
}
