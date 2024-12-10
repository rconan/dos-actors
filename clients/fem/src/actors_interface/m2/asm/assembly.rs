use crate::{actors_interface::fem_io, DiscreteModalSolver, Get, Set, Solver};
use gmt_dos_clients_io::{
    gmt_m2::asm::{
        M2ASMFaceSheetFigure, M2ASMFluidDampingForces, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
    },
    Assembly,
};
use interface::{Data, Read, Size, Write};
use std::sync::Arc;

impl<S> Read<M2ASMVoiceCoilsForces> for DiscreteModalSolver<S>
where
    S: Solver + Default,
    DiscreteModalSolver<S>: Iterator,
{
    fn read(&mut self, data: Data<M2ASMVoiceCoilsForces>) {
        let mut data_iter = data.iter();
        for sid in <M2ASMVoiceCoilsForces as Assembly>::SIDS {
            match sid {
                1 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S1VCDeltaF>>::set(
                    self,
                    &data_iter.next().unwrap(),
                ),
                2 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S2VCDeltaF>>::set(
                    self,
                    &data_iter.next().unwrap(),
                ),
                3 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S3VCDeltaF>>::set(
                    self,
                    &data_iter.next().unwrap(),
                ),
                4 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S4VCDeltaF>>::set(
                    self,
                    &data_iter.next().unwrap(),
                ),
                5 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S5VCDeltaF>>::set(
                    self,
                    &data_iter.next().unwrap(),
                ),
                6 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S6VCDeltaF>>::set(
                    self,
                    &data_iter.next().unwrap(),
                ),
                7 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S7VCDeltaF>>::set(
                    self,
                    &data_iter.next().unwrap(),
                ),
                _ => panic!("expected segment id with [1,7], found {:}", sid),
            }
        }
    }
}

impl<S> Size<M2ASMVoiceCoilsForces> for DiscreteModalSolver<S>
where
    S: Solver + Default,
    DiscreteModalSolver<S>: Iterator,
{
    fn len(&self) -> usize {
        675 * <M2ASMVoiceCoilsForces as Assembly>::N
    }
}

impl<S> Read<M2ASMFluidDampingForces> for DiscreteModalSolver<S>
where
    S: Solver + Default,
    DiscreteModalSolver<S>: Iterator,
{
    fn read(&mut self, data: Data<M2ASMFluidDampingForces>) {
        let mut data_iter = data.iter();
        for sid in <M2ASMVoiceCoilsForces as Assembly>::SIDS {
            match sid {
                1 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S1FluidDampingF>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                2 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S2FluidDampingF>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                3 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S3FluidDampingF>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                4 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S4FluidDampingF>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                5 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S5FluidDampingF>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                6 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S6FluidDampingF>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                7 => <DiscreteModalSolver<S> as Set<fem_io::MCM2S7FluidDampingF>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                _ => panic!("expected segment id with [1,7], found {:}", sid),
            }
        }
    }
}

impl<S> Size<M2ASMFluidDampingForces> for DiscreteModalSolver<S>
where
    S: Solver + Default,
    DiscreteModalSolver<S>: Iterator,
{
    fn len(&self) -> usize {
        675 * <M2ASMFluidDampingForces as Assembly>::N
    }
}
impl<S> Write<M2ASMVoiceCoilsMotion> for DiscreteModalSolver<S>
where
    S: Solver + Default,
    DiscreteModalSolver<S>: Iterator,
{
    fn write(&mut self) -> Option<Data<M2ASMVoiceCoilsMotion>> {
        let data: Vec<_> = <M2ASMVoiceCoilsForces as Assembly>::SIDS
            .into_iter()
            .filter_map(|sid| match sid {
                1 => <DiscreteModalSolver<S> as Get<fem_io::MCM2S1VCDeltaD>>::get(self),
                2 => <DiscreteModalSolver<S> as Get<fem_io::MCM2S2VCDeltaD>>::get(self),
                3 => <DiscreteModalSolver<S> as Get<fem_io::MCM2S3VCDeltaD>>::get(self),
                4 => <DiscreteModalSolver<S> as Get<fem_io::MCM2S4VCDeltaD>>::get(self),
                5 => <DiscreteModalSolver<S> as Get<fem_io::MCM2S5VCDeltaD>>::get(self),
                6 => <DiscreteModalSolver<S> as Get<fem_io::MCM2S6VCDeltaD>>::get(self),
                7 => <DiscreteModalSolver<S> as Get<fem_io::MCM2S7VCDeltaD>>::get(self),
                _ => panic!("expected segment id with [1,7], found {:}", sid),
            })
            .map(|data| Arc::new(data))
            .collect();
        Some(Data::new(data))
    }
}

impl<S> Size<M2ASMVoiceCoilsMotion> for DiscreteModalSolver<S>
where
    S: Solver + Default,
    DiscreteModalSolver<S>: Iterator,
{
    fn len(&self) -> usize {
        675 * <M2ASMVoiceCoilsMotion as Assembly>::N
    }
}

impl<S> Write<M2ASMFaceSheetFigure> for DiscreteModalSolver<S>
where
    S: Solver + Default,
    DiscreteModalSolver<S>: Iterator,
{
    fn write(&mut self) -> Option<Data<M2ASMFaceSheetFigure>> {
        let data: Vec<_> = <M2ASMVoiceCoilsForces as Assembly>::SIDS
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
        if self.facesheet_nodes.is_some() {
            let rbms = <DiscreteModalSolver<S> as Get<fem_io::MCM2RB6D>>::get(self)
                .expect("failed to get rigid body motion from ASMS reference bodies");
            self.facesheet_nodes.as_mut().map(|facesheet| {
                facesheet
                    .from_assembly(
                        <M2ASMVoiceCoilsForces as Assembly>::SIDS.into_iter(),
                        &data,
                        &rbms,
                    )
                    .expect("failed to remove RBM from ASM facesheet")
                    .into()
            })
        } else {
            Some(Data::new(data))
        }
    }
}
