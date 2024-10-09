use std::fmt::Display;

use faer::{Mat, MatRef};
use serde::{Deserialize, Serialize};

use super::{Calib, CalibPinv, CalibProps, CalibrationMode};
use crate::calibration::{algebra::Reconstructor, mode::Modality};

/// Closed-loop calibration matrix
///
/// The generic parameter indicates if the matrix correspond to a single segment ([CalibrationMode])
/// or to a full mirror ([MirrorMode](crate::calibration::MirrorMode),[MixedMirrorMode](crate::calibration::MixedMirrorMode)).
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct ClosedLoopCalib<M = CalibrationMode>
where
    M: Modality,
{
    pub(crate) m1_to_closed_loop_sensor: Reconstructor,
    pub(crate) m2_to_closed_loop_sensor: Reconstructor,
    pub(crate) m1_to_m2: Mat<f64>,
    pub(crate) m1_to_sensor: Option<Reconstructor>,
    pub(crate) m2_to_sensor: Option<Reconstructor>,
    pub(crate) m1_closed_loop_to_sensor: Calib<M>,
}

impl ClosedLoopCalib {
    pub fn m1_to_m2(&self) -> MatRef<'_, f64> {
        self.m1_to_m2.as_ref()
    }
}
impl<M: Modality + Display> CalibProps<M> for ClosedLoopCalib<M> {
    fn pseudoinverse(&self) -> CalibPinv<f64, M> {
        self.m1_closed_loop_to_sensor.pseudoinverse()
    }

    fn truncated_pseudoinverse(&self, n: usize) -> CalibPinv<f64, M> {
        self.m1_closed_loop_to_sensor.truncated_pseudoinverse(n)
    }

    fn area(&self) -> usize {
        self.m1_closed_loop_to_sensor.area()
    }

    fn match_areas(&mut self, other: &mut ClosedLoopCalib<M>) {
        self.m1_closed_loop_to_sensor
            .match_areas(&mut other.m1_closed_loop_to_sensor);
    }

    fn mask_as_slice(&self) -> &[bool] {
        self.m1_closed_loop_to_sensor.mask_as_slice()
    }

    fn mask_as_mut_slice(&mut self) -> &mut [bool] {
        self.m1_closed_loop_to_sensor.mask_as_mut_slice()
    }

    fn mask(&self, data: &[f64]) -> Vec<f64> {
        self.m1_closed_loop_to_sensor.mask(data)
    }

    fn n_cols(&self) -> usize {
        self.m1_closed_loop_to_sensor.n_cols()
    }

    fn n_rows(&self) -> usize {
        self.m1_closed_loop_to_sensor.n_rows()
    }

    fn mat_ref(&self) -> MatRef<'_, f64> {
        self.m1_closed_loop_to_sensor.mat_ref()
    }

    fn n_mode(&self) -> usize {
        self.m1_closed_loop_to_sensor.n_mode()
    }
    fn mode(&self) -> M {
        self.m1_closed_loop_to_sensor.mode()
    }

    fn sid(&self) -> u8 {
        self.m1_closed_loop_to_sensor.sid()
    }

    fn normalize(&mut self) -> f64 {
        self.m1_closed_loop_to_sensor.normalize()
    }

    fn norm_l2(&mut self) -> f64 {
        self.m1_closed_loop_to_sensor.norm_l2()
    }

    fn as_slice(&self) -> &[f64] {
        self.m1_closed_loop_to_sensor.as_slice()
    }

    fn mode_as_mut(&mut self) -> &mut M {
        self.m1_closed_loop_to_sensor.mode_as_mut()
    }

    fn as_mut_slice(&mut self) -> &mut [f64] {
        self.m1_closed_loop_to_sensor.as_mut_slice()
    }

    fn as_mut(&mut self) -> &mut Vec<f64> {
        &mut self.m1_closed_loop_to_sensor.c
    }
}

impl<M: Modality + Display> Display for ClosedLoopCalib<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // writeln!(f, "\nClosed-loop calibration matrix:")?;
        // write!(
        //     f,
        //     " * M1 -> closed-loop sensor: {:}",
        //     self.m1_to_closed_loop_sensor
        // )?;
        // write!(
        //     f,
        //     " * M2 -> closed-loop sensor: {:}",
        //     self.m2_to_closed_loop_sensor
        // )?;
        // writeln!(f, " * M1 -> M2: {:?}", self.m1_to_m2.shape())?;
        // if let Some(m1_to_sensor) = &self.m1_to_sensor {
        //     write!(f, " * M1 -> sensor: {:}", m1_to_sensor)?;
        // }
        // if let Some(m2_to_sensor) = &self.m2_to_sensor {
        //     write!(f, " * M2 -> sensor: {:}", m2_to_sensor)?;
        // }
        self.m1_closed_loop_to_sensor.fmt(f)
    }
}
