//! M2 rigid body motions

use super::prelude::*;
use gmt_dos_clients_io::{
    gmt_m2::{fsm::M2FSMPiezoForces, M2RigidBodyMotions},
    optics::{self, MirrorState},
    Assembly,
};

impl<S> Size<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        42
    }
}
#[cfg(all(fem, topend = "FSM"))]
impl<S> Write<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2Lcl6D>>::get(self).map(|data| Data::new(data))
    }
}
#[cfg(all(fem, topend = "FSM"))]
impl<S> Write<optics::M2State> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<optics::M2State>> {
        let data: Vec<_> = <M2FSMPiezoForces as Assembly>::SIDS
            .into_iter()
            .filter_map(|sid| match sid {
                1 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment1AxialD>>::get(self),
                2 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment2AxialD>>::get(self),
                3 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment3AxialD>>::get(self),
                4 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment4AxialD>>::get(self),
                5 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment5AxialD>>::get(self),
                6 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment6AxialD>>::get(self),
                7 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment7AxialD>>::get(self),
                _ => panic!("expected segment id with [1,7], found {:}", sid),
            })
            .collect();
        let rbms = <DiscreteModalSolver<S> as Get<fem_io::MCM2Lcl6D>>::get(self)
            .expect("failed to get rigid body motion from ASMS reference bodies");
        if data.is_empty() {
            return Some(Data::new(MirrorState::rbms(rbms)));
        }
        if self.facesheet_nodes.is_some() {
            self.facesheet_nodes.as_mut().map(|facesheet| {
                facesheet
                    .from_assembly(
                        <M2FSMPiezoForces as Assembly>::SIDS.into_iter(),
                        &data,
                        &rbms,
                    )
                    .expect("failed to remove RBM from ASM m1_figure")
            })
        } else {
            Some(data)
        }
        .map(|x| x.into_iter().flatten().collect::<Vec<_>>())
        .map(|modes| Data::new(MirrorState::new(rbms, modes)))
    }
}
#[cfg(all(fem, topend = "ASM"))]
impl<S> Write<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2RB6D>>::get(self).map(|data| Data::new(data))
    }
}
#[cfg(all(fem, m2_rbm = "MCM2Lcl"))]
impl<S> Write<M2RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2Lcl>>::get(self).map(|data| Data::new(data))
    }
}
