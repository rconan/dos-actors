use serde::{Deserialize, Serialize};
use std::{
    ops::{Deref, DerefMut},
    path::Path,
};

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Nodes {
    pub sid: u8,
    pub xyz: Vec<Vec<f64>>,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct AsmsNodes(Vec<Nodes>);
impl Deref for AsmsNodes {
    type Target = [Nodes];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for AsmsNodes {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, thiserror::Error)]
pub enum NodesError {
    #[error("failed to create file the nodes file")]
    IO(#[from] std::io::Error),
    #[error("failed to encode nodes")]
    Encode(#[from] bincode::error::EncodeError),
    #[cfg(feature = "polars")]
    #[error("failed to write nodes to parquet")]
    Parquet(#[from] polars::error::PolarsError),
}
type Result<T> = std::result::Result<T, NodesError>;

impl AsmsNodes {
    pub fn push(&mut self, nodes: Nodes) {
        self.0.push(nodes)
    }
    pub fn into_bin(&self, path: impl AsRef<Path>) -> Result<()> {
        let mut file = std::fs::File::create(path.as_ref())?;
        bincode::serde::encode_into_std_write(self, &mut file, bincode::config::standard())?;
        Ok(())
    }
    #[cfg(feature = "polars")]
    pub fn into_parquet(&self, path: impl AsRef<Path>) -> Result<()> {
        use polars::prelude::*;
        let mut series = vec![];
        for (i, nodes) in self.0.iter().enumerate() {
            let s: Vec<_> = nodes
                .xyz
                .iter()
                .map(|xyz| {
                    let s: Series = xyz.iter().collect();
                    s
                })
                .collect();
            series.push(Series::new(&format!("S{}", i + 1), s));
        }

        let mut df = DataFrame::new(series)?;

        let mut file = std::fs::File::create(path.as_ref())?;
        ParquetWriter::new(&mut file).finish(&mut df)?;
        Ok(())
    }
}
