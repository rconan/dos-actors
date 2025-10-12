use std::marker::PhantomData;

use gmt_dos_clients_io::gmt_m1::segment;
use interface::{Data, Read, Size, Update, Write};
use serde::{Deserialize, Serialize};

use crate::Calibration;

type M = nalgebra::Matrix6<f64>;
type V = nalgebra::Vector6<f64>;

pub enum New {}
pub enum LC2CG {}

#[derive(Debug, Clone)]
pub struct LoadCellsBuilder<T> {
    lc_2_cg: M,
    m1_hpk: Option<f64>,
    checks: PhantomData<T>,
}
impl Default for LoadCellsBuilder<New> {
    fn default() -> Self {
        Self {
            lc_2_cg: Default::default(),
            m1_hpk: None,
            checks: PhantomData,
        }
    }
}
impl LoadCellsBuilder<New> {
    pub fn hardpoints_barycentric_transform(self, lc_2_cg: M) -> LoadCellsBuilder<LC2CG> {
        LoadCellsBuilder {
            lc_2_cg,
            m1_hpk: None,
            checks: PhantomData,
        }
    }
}
impl LoadCellsBuilder<LC2CG> {
    pub fn hardpoints_stiffness(mut self, m1_hk: f64) -> Self {
        self.m1_hpk = Some(m1_hk);
        self
    }
    pub fn build<const ID: u8>(self) -> LoadCells<ID> {
        LoadCells {
            m1_hpk: self.m1_hpk,
            hp_f_cmd: vec![0f64; 6],
            hp_d_cell: vec![0f64; 6],
            hp_d_face: vec![0f64; 6],
            hp_f_meas: vec![0f64; 6],
            lc_2_cg: self.lc_2_cg,
        }
    }
}

/// [gmt_dos_actors](https://docs.rs/gmt_dos-actors) client interface for hardpoints loadcells
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadCells<const ID: u8> {
    pub(super) hp_f_cmd: Vec<f64>,
    pub(super) hp_d_cell: Vec<f64>,
    pub(super) hp_d_face: Vec<f64>,
    hp_f_meas: Vec<f64>,
    m1_hpk: Option<f64>,
    lc_2_cg: M,
}
impl<const ID: u8> From<&Calibration> for LoadCells<ID> {
    fn from(c: &Calibration) -> Self {
        if cfg!(any(
            not(m1_hp_force_extension),
            feature = "explicit-loadcells"
        )) {
            Self {
                m1_hpk: Some(c.stiffness),
                hp_f_cmd: vec![0f64; 6],
                hp_d_cell: vec![0f64; 6],
                hp_d_face: vec![0f64; 6],
                hp_f_meas: vec![0f64; 6],
                lc_2_cg: c.lc_2_cg[ID as usize - 1],
            }
        } else {
            Self {
                m1_hpk: None,
                hp_f_cmd: vec![0f64; 6],
                hp_d_cell: vec![0f64; 6],
                hp_d_face: vec![0f64; 6],
                hp_f_meas: vec![0f64; 6],
                lc_2_cg: c.lc_2_cg[ID as usize - 1],
            }
        }
    }
}
impl<const ID: u8> LoadCells<ID> {
    /// Creates a new loadcells client
    ///
    /// The hardpoints stiffness and the matrix transformation
    /// from local to center of gravity coordinates are provided.
    #[deprecated = "replaced with builder"]
    pub fn new(m1_hpk: f64, lc_2_cg: M) -> Self {
        Self {
            m1_hpk: Some(m1_hpk),
            hp_f_cmd: vec![0f64; 6],
            hp_d_cell: vec![0f64; 6],
            hp_d_face: vec![0f64; 6],
            hp_f_meas: vec![0f64; 6],
            lc_2_cg,
        }
    }
    pub fn builder() -> LoadCellsBuilder<New> {
        Default::default()
    }
}

impl<const ID: u8> Size<segment::HardpointsMotion<ID>> for LoadCells<ID> {
    fn len(&self) -> usize {
        12
    }
}

impl<const ID: u8> Size<segment::BarycentricForce<ID>> for LoadCells<ID> {
    fn len(&self) -> usize {
        6
    }
}

impl<const ID: u8> Update for LoadCells<ID> {
    fn update(&mut self) {
        if let Some(m1_hpk) = self.m1_hpk {
            self.hp_d_cell
                .iter()
                .zip(self.hp_d_face.iter())
                .map(|(hp_d_cell, hp_d_face)| hp_d_face - hp_d_cell)
                .map(|hp_relative_displacements| hp_relative_displacements * m1_hpk)
                .zip(self.hp_f_cmd.iter())
                .map(|(hp_relative_force, hp_f_cmd)| hp_relative_force - hp_f_cmd)
                .zip(&mut self.hp_f_meas)
                .for_each(|(hp_f_diff_force, hp_f_meas)| *hp_f_meas = hp_f_diff_force);
        } else {
            self.hp_f_meas.copy_from_slice(&self.hp_f_cmd);
        }
    }
}

impl<const ID: u8> Read<segment::HardpointsForces<ID>> for LoadCells<ID> {
    fn read(&mut self, data: Data<segment::HardpointsForces<ID>>) {
        self.hp_f_cmd = (**data).to_vec();
    }
}

impl<const ID: u8> Read<segment::HardpointsMotion<ID>> for LoadCells<ID> {
    fn read(&mut self, data: Data<segment::HardpointsMotion<ID>>) {
        let (cell, face) = (&data).split_at(6);
        self.hp_d_cell.copy_from_slice(cell);
        self.hp_d_face.copy_from_slice(face);
    }
}

impl<const ID: u8> Write<segment::BarycentricForce<ID>> for LoadCells<ID> {
    fn write(&mut self) -> Option<Data<segment::BarycentricForce<ID>>> {
        let cg = self.lc_2_cg * V::from_column_slice(self.hp_f_meas.as_slice());
        Some(Data::new(cg.as_slice().to_vec()))
    }
}
