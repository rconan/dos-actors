use gmt_dos_clients_io::gmt_m1::segment::{HardpointsForces, RBM};
#[cfg(all(m1_hp_force_extension, not(feature = "explicit-loadcells")))]
use hardpoints_dynamics::HardpointsDynamics;
#[cfg(any(not(m1_hp_force_extension), feature = "explicit-loadcells"))]
use hardpoints_dynamics_as_design::HardpointsDynamics;
use interface::{Data, Read, Size, Update, Write};

mod loadcell;
pub use loadcell::LoadCells;
use serde::{Deserialize, Serialize};

type M = nalgebra::Matrix6<f64>;
type V = nalgebra::Vector6<f64>;

/// [gmt_dos_actors](https://docs.rs/gmt_dos-actors) client interface for [HardpointsDynamics]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Hardpoints<const ID: u8> {
    dynamics: HardpointsDynamics,
    // FEM M1 RBM to FEM HP force input
    rbm_2_hp: M,
    // hardpoints stiffness (from FEM static gain)
    m1_hpk: f64,
}
impl<const ID: u8> From<&Calibration> for Hardpoints<ID> {
    fn from(c: &Calibration) -> Self {
        Self {
            dynamics: HardpointsDynamics::new(),
            rbm_2_hp: c.rbm_2_hp[ID as usize - 1],
            m1_hpk: c.stiffness,
        }
    }
}
impl<const ID: u8> Hardpoints<ID> {
    /// Creates a new hardpoints client
    ///
    /// The hardpoints stiffness and the matrix transformation
    /// from rigid body motion to hardpoint motion are provided.
    pub fn new(stiffness: f64, rbm_2_hp: M) -> Self {
        Self {
            dynamics: HardpointsDynamics::new(),
            rbm_2_hp,
            m1_hpk: stiffness,
        }
    }
}

impl<const ID: u8> Size<RBM<ID>> for Hardpoints<ID> {
    fn len(&self) -> usize {
        6
    }
}

impl<const ID: u8> Size<HardpointsForces<ID>> for Hardpoints<ID> {
    fn len(&self) -> usize {
        6
    }
}

impl<const ID: u8> Read<RBM<ID>> for Hardpoints<ID> {
    fn read(&mut self, data: Data<RBM<ID>>) {
        let hp = self.rbm_2_hp * V::from_column_slice(&data);
        self.dynamics.inputs.In1 = hp
            .as_slice()
            .try_into()
            .expect("failed to import `RBM` in `HardpointsDynamics` input");
    }
}

impl<const ID: u8> Update for Hardpoints<ID> {
    fn update(&mut self) {
        self.dynamics.step();
    }
}

impl<const ID: u8> Write<HardpointsForces<ID>> for Hardpoints<ID> {
    fn write(&mut self) -> Option<Data<HardpointsForces<ID>>> {
        let data: Vec<f64> = self
            .dynamics
            .outputs
            .Out1
            .iter()
            .map(|d| *d * self.m1_hpk)
            .collect();
        Some(Data::new(data))
    }
}
#[cfg(m1_hp_force_extension)]
use gmt_dos_clients_io::gmt_m1::segment::HardpointsMotion;

use crate::Calibration;
#[cfg(m1_hp_force_extension)]
impl<const ID: u8> Write<HardpointsMotion<ID>> for Hardpoints<ID> {
    fn write(&mut self) -> Option<Data<HardpointsMotion<ID>>> {
        let data: Vec<f64> = self.dynamics.outputs.Out1.to_vec();
        Some(Data::new(data))
    }
}
#[cfg(m1_hp_force_extension)]
impl<const ID: u8> Size<HardpointsMotion<ID>> for Hardpoints<ID> {
    fn len(&self) -> usize {
        6
    }
}
