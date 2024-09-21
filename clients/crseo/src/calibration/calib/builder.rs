use crate::calibration::CalibrationMode;
use serde::{Deserialize, Serialize};

use super::Calib;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct CalibBuilder {
    pub(crate) sid: u8,
    pub(crate) n_mode: usize,
    pub(crate) c: Vec<f64>,
    pub(crate) mask: Vec<bool>,
    pub(crate) mode: CalibrationMode,
    pub(crate) n_cols: Option<usize>,
}

impl CalibBuilder {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn c(mut self, c: Vec<f64>) -> Self {
        self.c = c;
        self
    }

    pub fn mask(mut self, mask: Vec<bool>) -> Self {
        self.mask = mask;
        self
    }

    pub fn mode(mut self, mode: CalibrationMode) -> Self {
        self.mode = mode;
        self
    }

    pub fn n_mode(mut self, n_mode: usize) -> Self {
        self.n_mode = n_mode;
        self
    }

    pub fn n_cols(mut self, n_cols: usize) -> Self {
        self.n_cols = Some(n_cols);
        self
    }

    pub fn build(self) -> Calib {
        let Self {
            sid,
            n_mode,
            c,
            mask,
            mode,
            n_cols,
        } = self;
        Calib {
            sid,
            n_mode,
            c,
            mask,
            mode,
            runtime: Default::default(),
            n_cols,
        }
    }
}
