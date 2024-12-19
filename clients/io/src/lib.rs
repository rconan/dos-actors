//! # GMT DOS Clients IO
//!
//! Definitions of the types for the inputs and the ouputs of [gmt_dos-actors](https://crates.io/crates/gmt_dos-actors)
//! clients used with the GMT Integrated Model

use std::any::type_name;

use interface::{UniqueIdentifier, UID};

pub mod assembly;
pub mod gmt_fem;
pub mod gmt_m1;
pub mod gmt_m2;

pub use assembly::Assembly;

/// M1 and M2 Rigid Body Motions
#[derive(UID)]
#[uid(port = 56_012)]
pub enum M12RigidBodyMotions {}

#[derive(UID)]
#[uid(port = 56_666)]
pub enum Estimate {}

/// Mount
pub mod mount {
    use super::UID;
    /// Mount Encoders
    #[derive(UID)]
    #[uid(port = 52_001)]
    pub enum MountEncoders {}
    #[derive(UID)]
    #[uid(port = 52_011)]
    pub enum AverageMountEncoders<const E: i32 = 0> {}
    /// Mount Torques
    #[derive(UID)]
    #[uid(port = 52_002)]
    pub enum MountTorques {}
    /// Mount set point
    #[derive(UID)]
    #[uid(port = 52_003)]
    pub enum MountSetPoint {}
}
/// CFD wind loads
pub mod cfd_wind_loads {
    use super::UID;
    /// CFD Mount Wind Loads
    #[derive(UID)]
    #[uid(port = 53_001)]
    pub enum CFDMountWindLoads {}
    /// CFD M1 Loads
    #[derive(UID)]
    #[uid(port = 53_002)]
    pub enum CFDM1WindLoads {}
    /// CFD M2 Wind Loads
    #[derive(UID)]
    #[uid(port = 53_003)]
    pub enum CFDM2WindLoads {}
}

pub mod optics;

/// Dome seeing
pub mod domeseeing {
    use super::UID;
    /// Dome seeing optical path difference in GMT exit pupil
    #[derive(UID)]
    #[uid(port = 54_001)]
    pub enum DomeSeeingOpd {}
}

/// Returns the port #
pub fn get_port<U: UniqueIdentifier>() -> u16 {
    <U as UniqueIdentifier>::PORT
}
/// Returns the data type
pub fn get_datatype<U: UniqueIdentifier>() -> &'static str {
    type_name::<<U as UniqueIdentifier>::DataType>()
}
