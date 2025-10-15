use std::sync::Arc;

#[cfg(not(m1_hp_force_extension))]
use gmt_dos_clients_io::gmt_m1::{assembly::M1HardpointsForces, segment::HardpointsForces};
#[cfg(m1_hp_force_extension)]
use gmt_dos_clients_io::gmt_m1::{assembly::M1HardpointsMotion, segment::HardpointsMotion};
use gmt_dos_clients_io::{
    Assembly,
    gmt_m1::{assembly::M1ActuatorAppliedForces, segment::ActuatorAppliedForces},
};
use interface::{Data, Read, Size, Update, Write, WriteFlatten};
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchOut
where
    Self: Assembly,
{
    m1_actuator_applied_forces: Vec<Arc<Vec<f64>>>,
    #[cfg(not(m1_hp_force_extension))]
    m1_hardpoints_forces: Vec<Arc<Vec<f64>>>,
    #[cfg(m1_hp_force_extension)]
    m1_hardpoints_motion: Vec<Arc<Vec<f64>>>,
}
impl DispatchOut {
    pub fn new() -> Self {
        Self {
            m1_actuator_applied_forces: vec![Default::default(); <Self as Assembly>::N],
            #[cfg(not(m1_hp_force_extension))]
            m1_hardpoints_forces: vec![Default::default(); <Self as Assembly>::N],
            #[cfg(m1_hp_force_extension)]
            m1_hardpoints_motion: vec![Default::default(); <Self as Assembly>::N],
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

#[cfg(not(m1_hp_force_extension))]
impl<const ID: u8> Read<HardpointsForces<ID>> for DispatchOut {
    fn read(&mut self, data: Data<HardpointsForces<ID>>) {
        if let Some(idx) = <Self as Assembly>::position::<ID>() {
            let forces = data.into_arc();
            self.m1_hardpoints_forces[idx] = forces;
        }
    }
}
#[cfg(not(m1_hp_force_extension))]
impl Write<M1HardpointsForces> for DispatchOut {
    fn write(&mut self) -> Option<Data<M1HardpointsForces>> {
        Some(Data::new(self.m1_hardpoints_forces.clone()))
    }
}

#[cfg(m1_hp_force_extension)]
impl<const ID: u8> Read<HardpointsMotion<ID>> for DispatchOut {
    fn read(&mut self, data: Data<HardpointsMotion<ID>>) {
        if let Some(idx) = <Self as Assembly>::position::<ID>() {
            let forces = data.into_arc();
            self.m1_hardpoints_motion[idx] = forces;
        }
    }
}
#[cfg(m1_hp_force_extension)]
impl Write<M1HardpointsMotion> for DispatchOut {
    fn write(&mut self) -> Option<Data<M1HardpointsMotion>> {
        Some(Data::new(self.m1_hardpoints_motion.clone()))
    }
}

#[cfg(not(m1_hp_force_extension))]
impl Size<M1HardpointsForces> for DispatchOut {
    fn len(&self) -> usize {
        42
    }
}

#[cfg(m1_hp_force_extension)]
impl Size<M1HardpointsMotion> for DispatchOut {
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
