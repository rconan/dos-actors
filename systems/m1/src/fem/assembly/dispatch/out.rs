use std::sync::Arc;

use gmt_dos_clients_io::{
    Assembly,
    gmt_m1::{
        assembly::{M1ActuatorAppliedForces, M1HardpointsForces},
        segment::{ActuatorAppliedForces, HardpointsForces},
    },
};
use interface::{Data, Read, Size, Update, Write, WriteFlatten};
use serde::{Deserialize, Serialize};

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

impl Size<M1HardpointsForces> for DispatchOut {
    fn len(&self) -> usize {
        42
    }
}

impl Size<M1ActuatorAppliedForces> for DispatchOut {
    fn len(&self) -> usize {
        335 * 6 + 306
    }
}

impl WriteFlatten for DispatchOut {}
