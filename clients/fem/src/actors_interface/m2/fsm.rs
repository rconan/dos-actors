//! M2 FSM Piezo-Stack Actuators

// use super::prelude::*;
use gmt_dos_clients_io::Assembly;
use gmt_dos_clients_io::gmt_m2::fsm::{M2FSMPiezoForces, M2FSMPiezoNodes};
use interface::{
    Data, Write,
    optics::{M2State, state::SegmentState},
};

use crate::{DiscreteModalSolver, Get};

use super::prelude::*;

impl<S> Write<M2State> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2State>> {
        // M2 RBMs (42)
        let rbms = <DiscreteModalSolver<S> as Get<fem_io::MCM2Lcl6D>>::get(self);
        // Segment RBMS (6x7)
        let mut segment_rbms = rbms.as_ref().map(|rbms| rbms.chunks(6));

        let mut segments = vec![];
        for sid in <M2FSMPiezoForces as Assembly>::SIDS.into_iter() {
            // segment # sid RBMS
            let rbms = segment_rbms.as_mut().map(|rbms| rbms.next());
            // segment # sid shape transforms
            // segment figure (aka shape)
            let figure = match sid {
                1 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment1AxialD>>::get(self),
                2 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment2AxialD>>::get(self),
                3 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment3AxialD>>::get(self),
                4 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment4AxialD>>::get(self),
                5 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment5AxialD>>::get(self),
                6 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment6AxialD>>::get(self),
                7 => <DiscreteModalSolver<S> as Get<fem_io::M2Segment7AxialD>>::get(self),
                _ => panic!("expected segment id with [1,7], found {:}", sid),
            };
            let segment = match (rbms, figure) {
                (None, Some(figure)) | (Some(None), Some(figure)) => SegmentState::modes(figure),
                (Some(Some(rbms)), None) => SegmentState::rbms(rbms),
                (Some(Some(rbms)), Some(mut figure)) => {
                    self.facesheet_nodes.as_mut().map(|facesheet| {
                        facesheet.rbms_removal(sid, &mut figure, rbms);
                    });
                    SegmentState::modes(figure)
                }
                _ => SegmentState::default(),
            };
            segments.push(segment);
        }
        Some(Data::new(segments.into_iter().collect()))
    }
}
/// forces
impl<S> Read<M2FSMPiezoForces> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<M2FSMPiezoForces>) {
        <DiscreteModalSolver<S> as Set<fem_io::MCM2PZTF>>::set(self, &data)
    }
}
/// nodes
impl<S> Write<M2FSMPiezoNodes> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M2FSMPiezoNodes>> {
        <DiscreteModalSolver<S> as Get<fem_io::MCM2PZTD>>::get(self).map(|data| Data::new(data))
    }
}
