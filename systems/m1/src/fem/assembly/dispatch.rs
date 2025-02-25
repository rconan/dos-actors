use std::sync::Arc;

use gmt_dos_clients::operator;
use gmt_dos_clients_io::{
    gmt_m1::{
        assembly::{
            M1ActuatorAppliedForces, M1ActuatorCommandForces, M1HardpointsForces,
            M1HardpointsMotion, M1RigidBodyMotions,
        },
        segment::{
            ActuatorAppliedForces, ActuatorCommandForces, HardpointsForces, HardpointsMotion, RBM,
        },
    },
    Assembly,
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use serde::{Deserialize, Serialize};

const NA: [usize; 7] = [335, 335, 335, 335, 335, 335, 306];

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchIn
where
    Self: Assembly,
{
    m1_rigid_body_motions: Vec<Arc<Vec<f64>>>,
    m1_actuator_command_forces: Vec<Arc<Vec<f64>>>,
    m1_hardpoints_motion: Arc<Vec<Arc<Vec<f64>>>>,
    idx: Vec<usize>,
}
impl DispatchIn {
    pub fn new() -> Self {
        let m1_actuator_command_forces: Vec<_> = <Self as Assembly>::SIDS
            .into_iter()
            .map(|i| Arc::new(vec![0f64; NA[i as usize - 1]]))
            .collect();
        let m1_rigid_body_motions: Vec<_> = <Self as Assembly>::SIDS
            .into_iter()
            .map(|_| Arc::new(vec![0f64; 6]))
            .collect();
        let mut idx = vec![0; 7];
        <Self as Assembly>::SIDS
            .iter()
            .enumerate()
            .for_each(|(i, &id)| {
                idx[id as usize - 1] = i;
            });
        Self {
            m1_rigid_body_motions,
            m1_actuator_command_forces,
            m1_hardpoints_motion: Default::default(),
            idx,
        }
    }
}
impl Assembly for DispatchIn {}
impl Update for DispatchIn {}

impl Read<M1RigidBodyMotions> for DispatchIn {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        let data = data.into_arc();
        self.m1_rigid_body_motions.iter_mut().fold(0, |i, out| {
            let (slice, _) = data[i..].split_at(6);
            *out = Arc::new(slice.to_vec());
            i + 6
        });
    }
}

impl<U> Read<operator::Left<U>> for DispatchIn
where
    U: UniqueIdentifier,
    DispatchIn: Read<U>,
{
    fn read(&mut self, data: Data<operator::Left<U>>) {
        <Self as Read<U>>::read(self, data.transmute());
    }
}
impl<U> Read<operator::Right<U>> for DispatchIn
where
    U: UniqueIdentifier,
    DispatchIn: Read<U>,
{
    fn read(&mut self, data: Data<operator::Right<U>>) {
        <Self as Read<U>>::read(self, data.transmute());
    }
}

impl<const ID: u8> Write<RBM<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<RBM<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.m1_rigid_body_motions
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}

impl Read<M1ActuatorCommandForces> for DispatchIn {
    fn read(&mut self, data: Data<M1ActuatorCommandForces>) {
        let data = data.into_arc();
        NA.iter()
            .zip(self.m1_actuator_command_forces.iter_mut())
            .fold(0, |i, (&s, out)| {
                let (slice, _) = data[i..].split_at(s);
                *out = Arc::new(slice.to_vec());
                i + s
            });
    }
}
impl<const ID: u8> Read<ActuatorCommandForces<ID>> for DispatchIn {
    fn read(&mut self, data: Data<ActuatorCommandForces<ID>>) {
        self.m1_actuator_command_forces[ID as usize - 1] = data.into_arc();
    }
}
impl<const ID: u8> Write<ActuatorCommandForces<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<ActuatorCommandForces<ID>>> {
        Some(
            self.m1_actuator_command_forces[self.idx[ID as usize - 1]]
                .clone()
                .into(),
        )
    }
}

impl Read<M1HardpointsMotion> for DispatchIn {
    fn read(&mut self, data: Data<M1HardpointsMotion>) {
        self.m1_hardpoints_motion = data.into_arc();
    }
}
impl<const ID: u8> Write<HardpointsMotion<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<HardpointsMotion<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.m1_hardpoints_motion
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchOut
where
    Self: Assembly,
{
    m1_actuator_applied_forces: Vec<Arc<Vec<f64>>>,
    m1_hardpoints_forces: Vec<Arc<Vec<f64>>>,
}
impl DispatchOut {
    pub fn new() -> Self {
        Self {
            m1_actuator_applied_forces: vec![Default::default(); <Self as Assembly>::N],
            m1_hardpoints_forces: vec![Default::default(); <Self as Assembly>::N],
        }
    }
}
impl Assembly for DispatchOut {}
impl Update for DispatchOut {}

impl<const ID: u8> Read<ActuatorAppliedForces<ID>> for DispatchOut {
    fn read(&mut self, data: Data<ActuatorAppliedForces<ID>>) {
        if let Some(idx) = <Self as Assembly>::position::<ID>() {
            let forces = data.into_arc();
            self.m1_actuator_applied_forces[idx] = forces;
        }
    }
}
impl Write<M1ActuatorAppliedForces> for DispatchOut {
    fn write(&mut self) -> Option<Data<M1ActuatorAppliedForces>> {
        Some(Data::new(self.m1_actuator_applied_forces.clone()))
    }
}

impl<const ID: u8> Read<HardpointsForces<ID>> for DispatchOut {
    fn read(&mut self, data: Data<HardpointsForces<ID>>) {
        if let Some(idx) = <Self as Assembly>::position::<ID>() {
            let forces = data.into_arc();
            self.m1_hardpoints_forces[idx] = forces;
        }
    }
}
impl Write<M1HardpointsForces> for DispatchOut {
    fn write(&mut self) -> Option<Data<M1HardpointsForces>> {
        Some(Data::new(self.m1_hardpoints_forces.clone()))
    }
}
