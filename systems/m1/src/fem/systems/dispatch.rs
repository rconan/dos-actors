use std::sync::Arc;

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
use interface::{Data, Read, Update, Write};

#[derive(Debug, Default)]
pub struct DispatchIn
where
    Self: Assembly,
{
    m1_rigid_body_motions: Arc<Vec<Arc<Vec<f64>>>>,
    m1_actuator_command_forces: Arc<Vec<Arc<Vec<f64>>>>,
    m1_hardpoints_motion: Arc<Vec<Arc<Vec<f64>>>>,
}
impl DispatchIn {
    pub fn new() -> Self {
        Default::default()
    }
    pub fn position<const ID: u8>(&self) -> Option<usize> {
        <Self as Assembly>::SIDS
            .into_iter()
            .position(|sid| sid == ID)
    }
}
impl Assembly for DispatchIn {}
impl Update for DispatchIn {}

impl Read<M1RigidBodyMotions> for DispatchIn {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        self.m1_rigid_body_motions = data.into_arc();
    }
}
impl<const ID: u8> Write<RBM<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<RBM<ID>>> {
        self.position::<ID>().and_then(|idx| {
            self.m1_rigid_body_motions
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}

impl Read<M1ActuatorCommandForces> for DispatchIn {
    fn read(&mut self, data: Data<M1ActuatorCommandForces>) {
        self.m1_actuator_command_forces = data.into_arc();
    }
}
impl<const ID: u8> Write<ActuatorCommandForces<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<ActuatorCommandForces<ID>>> {
        self.position::<ID>().and_then(|idx| {
            self.m1_actuator_command_forces
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}

impl Read<M1HardpointsMotion> for DispatchIn {
    fn read(&mut self, data: Data<M1HardpointsMotion>) {
        self.m1_hardpoints_motion = data.into_arc();
    }
}
impl<const ID: u8> Write<HardpointsMotion<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<HardpointsMotion<ID>>> {
        self.position::<ID>().and_then(|idx| {
            self.m1_hardpoints_motion
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}

#[derive(Debug, Default)]
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
    pub fn position<const ID: u8>(&self) -> Option<usize> {
        <Self as Assembly>::SIDS
            .into_iter()
            .position(|sid| sid == ID)
    }
}
impl Assembly for DispatchOut {}
impl Update for DispatchOut {}

impl<const ID: u8> Read<ActuatorAppliedForces<ID>> for DispatchOut {
    fn read(&mut self, data: Data<ActuatorAppliedForces<ID>>) {
        if let Some(idx) = self.position::<ID>() {
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
        if let Some(idx) = self.position::<ID>() {
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
