use anyhow::{Context, Result};
use gmt_dos_clients_io::gmt_m2::{asm::M2ASMReferenceBodyNodes, M2EdgeSensors};
use interface::{Data, Read, Update, Write};
use matio_rs::MatFile;
use std::{env, path::Path, sync::Arc};

use nalgebra as na;

use io::M2EdgeSensorsAsRbms;

/// ASMS actuators to reference bodies off-load
#[derive(Debug, Clone)]
pub struct M2EdgeSensorsToRbm {
    // 36x36
    // rbm_2_mode: na::DMatrix<f64>,
    // 36x6
    r7_2_es: na::DMatrix<f64>,
    // 36x48
    es_2_r: na::DMatrix<f64>,
    // 42
    rbms: Arc<Vec<f64>>,
    // 42
    edge_sensors: Arc<Vec<f64>>,
    // 42
    data: Arc<Vec<f64>>,
}
impl M2EdgeSensorsToRbm {
    pub fn new() -> Result<Self> {
        let data_repo = env::var("EDGE_SENSORS_DATA").context("`EDGE_SENSORS_DATA` is not set")?;
        //  * M2S7 RIGID-BODY MOTIONS TO EDGE SENSORS
        let r7_2_es: na::DMatrix<f64> = MatFile::load(Path::new(&data_repo).join("m2_r7_es.mat"))
            .context("Failed to read from m2_r7_es.mat")?
            .var("m2_r7_es")?;
        //  * EDGE SENSORS TO M2 RBMS
        let es_2_r = MatFile::load(Path::new(&data_repo).join("m2_r_es.mat"))
            .context("Failed to read from m12_r_es.mat")?
            .var("m2_r_es")?;
        Ok(Self {
            r7_2_es,
            es_2_r,
            rbms: Default::default(),
            edge_sensors: Default::default(),
            data: Default::default(),
        })
    }
}
impl Update for M2EdgeSensorsToRbm {
    fn update(&mut self) {
        // RBM of M2S7
        let r7 = &self.rbms[36..];
        // Transform r7 into edge sensors
        let es_from_r7 = &self.r7_2_es * na::DVector::from_column_slice(r7);
        // let rbm_2_mode =
        //     &self.rbm_2_mode * na::DVector::from_column_slice(&self.edge_sensors[..36]);
        // Offset `es_from_r7` from edge_sensors
        let data: Vec<_> = self
            .edge_sensors
            .iter()
            .zip(es_from_r7.into_iter())
            .map(|(x, y)| x - y)
            .collect();
        // Computes outer segment RBM from offset'd edge sensors
        let rbm = &self.es_2_r * na::DVector::from_column_slice(&data);
        // Concatenate `rbm` with `r7`
        let data: Vec<_> = rbm
            .into_iter()
            .map(|x| *x)
            .chain(r7.into_iter().map(|x| *x))
            .collect::<Vec<_>>();
        self.data = Arc::new(data);
    }
}
impl Read<M2ASMReferenceBodyNodes> for M2EdgeSensorsToRbm {
    fn read(&mut self, data: Data<M2ASMReferenceBodyNodes>) {
        self.rbms = data.into_arc();
    }
}
impl Read<M2EdgeSensors> for M2EdgeSensorsToRbm {
    fn read(&mut self, data: Data<M2EdgeSensors>) {
        self.edge_sensors = data.into_arc();
    }
}
impl Write<M2EdgeSensorsAsRbms> for M2EdgeSensorsToRbm {
    fn write(&mut self) -> Option<Data<M2EdgeSensorsAsRbms>> {
        Some(self.data.clone().into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_m2_es_to_r() {
        let Ok(mut m2_es_to_r) = M2EdgeSensorsToRbm::new() else {
            return;
        };
    }
}
