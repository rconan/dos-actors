use crate::{DiscreteModalSolver, Get, Set, Solver, actors_interface::fem_io};
use gmt_dos_clients_io::{
    Assembly,
    gmt_m1::assembly::{M1ActuatorAppliedForces, M1HardpointsForces, M1HardpointsMotion},
};
use interface::{Data, Read, Write};
use std::sync::Arc;

impl<S> Read<M1HardpointsForces> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<M1HardpointsForces>) {
        let mut data_iter = data.iter();
        for id in <M1HardpointsForces as Assembly>::SIDS {
            let a: usize = (id * 6).into();
            <DiscreteModalSolver<S> as Set<fem_io::OSSHarpointDeltaF>>::set_slice(
                self,
                data_iter.next().unwrap(),
                a - 6..a,
            );
        }
    }
}

#[cfg(m1_hp_force_extension)]
impl<S> Write<M1HardpointsForces> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1HardpointsForces>> {
        let mut data = vec![];
        for id in <M1HardpointsMotion as Assembly>::SIDS {
            let a: usize = (id * 6).into();
            data.push(
                <DiscreteModalSolver<S> as Get<fem_io::OSSHardpointForce>>::get(self)
                    .as_ref()
                    .map(|data| data[a - 6..a].to_vec())
                    .map(|data| Arc::new(data))
                    .unwrap(),
            );
        }
        Some(Data::new(data))
    }
}

impl<S> Write<M1HardpointsMotion> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1HardpointsMotion>> {
        let mut data = vec![];
        for id in <M1HardpointsMotion as Assembly>::SIDS {
            let a: usize = (id * 12).into();
            data.push(
                <DiscreteModalSolver<S> as Get<fem_io::OSSHardpointD>>::get(self)
                    .as_ref()
                    .map(|data| data[a - 12..a].to_vec())
                    .map(|data| Arc::new(data))
                    .unwrap(),
            );
        }
        Some(Data::new(data))
    }
}

#[cfg(m1_hp_force_extension)]
impl<S> Read<M1HardpointsMotion> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<M1HardpointsMotion>) {
        let mut data_iter = data.iter();
        for id in <M1HardpointsForces as Assembly>::SIDS {
            let a: usize = (id * 6).into();
            <DiscreteModalSolver<S> as Set<fem_io::OSSHardpointExtension>>::set_slice(
                self,
                data_iter.next().unwrap(),
                a - 6..a,
            );
        }
    }
}

impl<S> Read<M1ActuatorAppliedForces> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<M1ActuatorAppliedForces>) {
        let mut data_iter = data.iter();
        for sid in <M1ActuatorAppliedForces as Assembly>::SIDS {
            match sid {
                1 => <DiscreteModalSolver<S> as Set<fem_io::M1ActuatorsSegment1>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                2 => <DiscreteModalSolver<S> as Set<fem_io::M1ActuatorsSegment2>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                3 => <DiscreteModalSolver<S> as Set<fem_io::M1ActuatorsSegment3>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                4 => <DiscreteModalSolver<S> as Set<fem_io::M1ActuatorsSegment4>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                5 => <DiscreteModalSolver<S> as Set<fem_io::M1ActuatorsSegment5>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                6 => <DiscreteModalSolver<S> as Set<fem_io::M1ActuatorsSegment6>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                7 => <DiscreteModalSolver<S> as Set<fem_io::M1ActuatorsSegment7>>::set(
                    self,
                    data_iter.next().unwrap(),
                ),
                _ => unreachable!(),
            }
        }
    }
}
