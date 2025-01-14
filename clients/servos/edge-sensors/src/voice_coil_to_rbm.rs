use anyhow::{Context, Result};
use gmt_dos_clients_io::gmt_m2::asm::M2ASMVoiceCoilsMotion;
use interface::{filing::Codec, Data, Read, UniqueIdentifier, Update, Write};
use matio_rs::MatFile;
use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};
use std::{env, mem, path::Path, sync::Arc};

/// Voice coils displacements to rigid body motions
#[derive(Debug, Serialize, Deserialize)]
pub struct VoiceCoilToRbm {
    data: Arc<Vec<Arc<Vec<f64>>>>,
    vc_2_rbm: Vec<DMatrix<f64>>,
    y: Vec<f64>,
}

impl VoiceCoilToRbm {
    pub fn new() -> Result<Self> {
        let data_repo = env::var("EDGE_SENSORS_DATA").context("`EDGE_SENSORS_DATA` is not set")?;
        let mat_file = MatFile::load(Path::new(&data_repo).join("m2_vc_r.mat"))?;
        let mut vc_2_rbm = Vec::<DMatrix<f64>>::new();
        for i in 1..=7 {
            vc_2_rbm.push(mat_file.var(format!("m2_s{i}_vc_r"))?);
        }
        // let mut vc_2_rbm = vec![];
        // for i in 1..=7 {
        //     let t = Transform::new(
        //         IO::new("MC_M2_lcl_6D").rows((i - 1) * 6, 6),
        //         format!("MC_M2_S{i}_VC_delta_D"),
        //         format!("MC_M2_S{i}_VC_delta_F"),
        //     )
        //     .build(fem)?;
        //     vc_2_rbm.push(t);
        // }
        Ok(Self {
            data: Default::default(),
            vc_2_rbm,
            y: vec![0f64; 42],
        })
    }
}

// impl TryFrom<&mut FEM> for VoiceCoilToRbm {
//     type Error = anyhow::Error;

//     fn try_from(fem: &mut FEM) -> Result<Self> {
//         Self::new(fem)
//     }
// }

impl Codec for VoiceCoilToRbm {}

impl Update for VoiceCoilToRbm {
    fn update(&mut self) {
        let _ = mem::replace(
            &mut self.y,
            self.data
                .iter()
                .zip(&self.vc_2_rbm)
                .map(|(data, vc_2_rbm)| -vc_2_rbm * DVector::from_column_slice(data.as_slice()))
                // .map(|(data, vc_2_rbm)| {
                //     -vc_2_rbm * from_column_major_slice::<f64>(data.as_slice(), data.len(), 1)
                // })
                .flat_map(|x| x.as_slice().to_vec())
                .collect::<Vec<_>>(),
        );
    }
}

impl Read<M2ASMVoiceCoilsMotion> for VoiceCoilToRbm {
    fn read(&mut self, data: Data<M2ASMVoiceCoilsMotion>) {
        self.data = data.into_arc();
    }
}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for VoiceCoilToRbm {
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.y.clone().into())
    }
}
