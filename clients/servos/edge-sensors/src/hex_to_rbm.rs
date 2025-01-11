use anyhow::{Context, Result};
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use matio_rs::MatFile;
use nalgebra::{DMatrix, DVector};
use std::{env, mem, path::Path, sync::Arc};

#[derive(Debug)]
pub struct HexToRbm {
    data: Arc<Vec<f64>>,
    d2r: DMatrix<f64>,
    y: Vec<f64>,
}

impl HexToRbm {
    pub fn new() -> Result<Self> {
        let data_repo = env::var("EDGE_SENSORS_DATA").context("`EDGE_SENSORS_DATA` is not set")?;
        let mat_file = MatFile::load(Path::new(&data_repo).join("m2_hex_d2r.mat"))?;
        Ok(Self {
            data: Arc::new(vec![0f64; 42]),
            d2r: mat_file.var("d2r")?,
            y: vec![0f64; 427],
        })
    }
}

impl Update for HexToRbm {
    fn update(&mut self) {
        let hex_d: Vec<_> = self.data.chunks(2).map(|x| x[1] - x[0]).collect();
        let _ = mem::replace(
            &mut self.y,
            (&self.d2r * DVector::from_column_slice(&hex_d))
                .as_slice()
                .to_vec(),
        );
    }
}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for HexToRbm {
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for HexToRbm {
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.y.clone().into())
    }
}
