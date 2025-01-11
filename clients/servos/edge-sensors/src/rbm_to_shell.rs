use anyhow::{Context, Result};
use gmt_dos_clients_io::gmt_m1::M1EdgeSensors;
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use io::M2EdgeSensorsAsRbms;
use matio_rs::MatFile;
use na::{DMatrix, DVector};
use std::{env, mem, path::Path, sync::Arc};

use gmt_lom::{Loader, LoaderTrait, OpticalSensitivities, OpticalSensitivity};
use nalgebra as na;

use crate::N_ACTUATOR;

/// Rigid body motions to facesheet displacements
#[derive(Debug, Clone)]
pub struct RbmToShell {
    data: Arc<Vec<f64>>,
    rbm_2_shell: Vec<DMatrix<f64>>,
    y: Vec<f64>,
    tzrxry_m1tom2: Vec<DMatrix<f64>>,
    m1_rbm: Option<Arc<Vec<f64>>>,
}

const N: usize = 84;

impl RbmToShell {
    pub fn new() -> Result<Self> {
        let data_repo = env::var("EDGE_SENSORS_DATA").context("`EDGE_SENSORS_DATA` is not set")?;
        let mat_file = MatFile::load(Path::new(&data_repo).join("rbm_2_faceheet.mat"))?;
        let mut rbm_2_shell = Vec::<DMatrix<f64>>::new();
        for i in 1..=7 {
            rbm_2_shell.push(mat_file.var(format!("m2_s{i}_rbm_2_shell"))?);
        }
        let lom = Loader::<OpticalSensitivities<N>>::default().load()?;

        // M1 RBM to M2 RBM through LOM
        //  * M1/2 RBM  to segment piston [7x84]
        let segment_piston = if let OpticalSensitivity::SegmentPiston(sens) =
            &lom[OpticalSensitivity::SegmentPiston(vec![])]
        {
            na::DMatrix::from_column_slice(7, N, sens)
        } else {
            panic!("failed to build segment piston sensitivity matrix")
        };
        //  * M1/2 RBM to segment tip-tilt [14x84]
        let segment_tiptilt = if let OpticalSensitivity::SegmentTipTilt(sens) =
            &lom[OpticalSensitivity::SegmentTipTilt(vec![])]
        {
            na::DMatrix::from_column_slice(14, N, sens)
        } else {
            panic!("failed to build segment tip-tilt sensitivity matrix")
        };
        // M1 [Tz,Rx,Ry] to M2 [Tz,Rx,Ry]
        let mut tzrxry_m1tom2 = vec![];
        for i in 0..7 {
            // M1
            let mut first_col = 6 * i + 2;
            let p = segment_piston.columns(first_col, 3);
            let t = segment_tiptilt.columns(first_col, 3);
            let l1 = DMatrix::<f64>::from_rows(
                &p.row_iter()
                    .skip(i)
                    .take(1)
                    .chain(t.row_iter().skip(i).step_by(7).take(2))
                    .collect::<Vec<_>>(),
            );
            // M2
            first_col += 42;
            let p = segment_piston.columns(first_col, 3);
            let t = segment_tiptilt.columns(first_col, 3);
            let l2 = DMatrix::<f64>::from_rows(
                &p.row_iter()
                    .skip(i)
                    .take(1)
                    .chain(t.row_iter().skip(i).step_by(7).take(2))
                    .collect::<Vec<_>>(),
            );
            let l = l2.try_inverse().unwrap() * l1;
            tzrxry_m1tom2.push(l);
        }

        Ok(Self {
            data: Default::default(),
            rbm_2_shell,
            y: vec![0f64; N_ACTUATOR * 7],
            tzrxry_m1tom2,
            m1_rbm: None,
        })
    }
}

impl Update for RbmToShell {
    fn update(&mut self) {
        let m12_rbm = self.m1_rbm.as_ref().map(|m1_rbm| {
            m1_rbm
                .chunks(6)
                .zip(&self.tzrxry_m1tom2)
                .map(|(r, t)| t * na::DVector::from_column_slice(&r[2..5]))
                .map(|x| x.as_slice().to_vec())
        });
        if let Some(m12_rbm) = m12_rbm {
            let mut m2_rbm = self.data.iter().as_slice().to_vec();
            m2_rbm.chunks_mut(6).zip(m12_rbm).for_each(|(m2, m12)| {
                m2.iter_mut()
                    .skip(2)
                    .zip(m12)
                    .for_each(|(m2, m12)| *m2 += m12)
            });
            let _ = mem::replace(
                &mut self.y,
                m2_rbm
                    .chunks(6)
                    .zip(&self.rbm_2_shell)
                    .map(|(data, rbm_2_shell)| rbm_2_shell * DVector::from_column_slice(data))
                    .flat_map(|x| x.as_slice().to_vec())
                    .collect::<Vec<_>>(),
            );
        } else {
            let _ = mem::replace(
                &mut self.y,
                self.data
                    .chunks(6)
                    .zip(&self.rbm_2_shell)
                    .map(|(data, rbm_2_shell)| rbm_2_shell * DVector::from_column_slice(data))
                    .flat_map(|x| x.as_slice().to_vec())
                    .collect::<Vec<_>>(),
            );
        }
    }
}

impl Read<M2EdgeSensorsAsRbms> for RbmToShell {
    fn read(&mut self, data: Data<M2EdgeSensorsAsRbms>) {
        self.data = data.into_arc();
    }
}

impl Read<M1EdgeSensors> for RbmToShell {
    fn read(&mut self, data: Data<M1EdgeSensors>) {
        self.m1_rbm = Some(data.into_arc());
    }
}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for RbmToShell {
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.y.clone().into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_rbm_2_shell() {
        let Ok(mut rbm_to_shell) = RbmToShell::new() else {
            return;
        };
        for i in 0..7 {
            println!("#{i} : {:.3}", rbm_to_shell.tzrxry_m1tom2[i]);
        }
    }
}
