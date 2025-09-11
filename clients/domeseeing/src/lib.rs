#[cfg(feature = "object_store")]
use futures::StreamExt;
use glob::{GlobError, PatternError};
use gmt_dos_clients_io::domeseeing::DomeSeeingOpd;
use interface::{Data, Size, Update, Write};
#[cfg(feature = "object_store")]
use object_store::ObjectStore;
use serde::{Deserialize, Serialize};
#[cfg(feature = "npyz")]
use std::io;
#[cfg(not(feature = "object_store"))]
use std::path::{Path, PathBuf};
use std::{num::ParseFloatError, ops::Index, sync::Arc};

#[derive(Debug, thiserror::Error)]
pub enum DomeSeeingError {
    #[error("failed to load dome seeing data {1}")]
    Load(#[source] std::io::Error, String),
    #[error("failed to get dome seeing data path")]
    Glob(#[from] GlobError),
    #[error("failed to find dome seeing file pattern")]
    Pattern(#[from] PatternError),
    #[cfg(feature = "bincode")]
    #[error("failed to read dome seeing file")]
    Bincode(#[from] bincode::Error),
    #[error("dome seeing index {0} is out-of-bounds")]
    OutOfBounds(usize),
    #[error("failed to parse CFD optvol files timestamp: {1}")]
    TimeStamp(#[source] ParseFloatError, String),
    #[error("the remote store is missing")]
    MissingStore,
    #[cfg(feature = "object_store")]
    #[error("failed to access remote data")]
    Store(#[from] object_store::Error),
}

mod builder;
pub use builder::DomeSeeingBuilder;

type Result<T> = std::result::Result<T, DomeSeeingError>;

//const CFD_SAMPLING_FREQUENCY: f64 = 5f64; // Hz

/// Dome seeing OPD
///
/// The OPD `values` are given only inside the `mask` (i.e. where the mask is `true`)
#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct Opd {
    pub mean: f64,
    pub values: Vec<f64>,
    pub mask: Vec<bool>,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct DomeSeeingData {
    pub time_stamp: f64,
    #[cfg(not(feature = "object_store"))]
    pub file: PathBuf,
    #[cfg(feature = "object_store")]
    pub file: object_store::path::Path,
}

impl PartialOrd for DomeSeeingData {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.time_stamp.partial_cmp(&other.time_stamp)
    }
}

type Counter = Box<dyn Iterator<Item = usize> + Send>;

/// Dome seeing time series
pub struct DomeSeeing {
    upsampling: usize,
    data: Vec<DomeSeeingData>,
    counter: Counter,
    i: usize,
    y1: Opd,
    y2: Opd,
    mapping: OpdMapping,
    opd: Option<Arc<Vec<f64>>>,
    #[cfg(feature = "object_store")]
    store: Option<Arc<dyn ObjectStore>>,
}
impl std::fmt::Debug for DomeSeeing {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DomeSeeing")
            .field("upsampling", &self.upsampling)
            .field("data", &self.data)
            // .field("counter", ())
            .field("i", &self.i)
            .field("y1", &self.y1)
            .field("y2", &self.y2)
            .field("mapping", &self.mapping)
            .finish()
    }
}

unsafe impl Send for DomeSeeing {}
unsafe impl Sync for DomeSeeing {}

#[derive(Debug, Default, Clone)]
enum OpdMapping {
    #[default]
    Whole,
    Masked,
}

impl DomeSeeing {
    #[cfg(not(feature = "object_store"))]
    /// Creates a [DomeSeeingBuilder] instance given the `path` to the CFD case
    pub fn builder(path: impl AsRef<Path>) -> DomeSeeingBuilder {
        DomeSeeingBuilder {
            cfd_case_path: path.as_ref().to_path_buf(),
            ..Default::default()
        }
    }
    #[cfg(feature = "object_store")]
    /// Creates a [DomeSeeingBuilder] instance given the `path` to the CFD case
    pub fn builder(path: impl Into<object_store::path::Path>) -> DomeSeeingBuilder {
        DomeSeeingBuilder {
            cfd_case_path: path.into(),
            ..Default::default()
        }
    }
    pub fn masked(mut self) -> Self {
        self.mapping = OpdMapping::Masked;
        self
    }
    pub fn len(&self) -> usize {
        self.data.len()
    }
    #[cfg(feature = "bincode")]
    pub fn get(&self, idx: usize) -> Result<Opd> {
        let path = &self
            .data
            .get(idx)
            .ok_or(DomeSeeingError::OutOfBounds(idx))?
            .file;
        let file = std::fs::File::open(&path)
            .map_err(|e| DomeSeeingError::Load(e, path.display().to_string()))?;
        Ok(bincode::deserialize_from(&file)?)
    }
    #[cfg(all(
        feature = "npyz",
        not(feature = "bincode"),
        not(feature = "object_store")
    ))]
    pub fn get(&self, idx: usize) -> Result<Opd> {
        let path = &self
            .data
            .get(idx)
            .ok_or(DomeSeeingError::OutOfBounds(idx))?
            .file;
        let mut archive = npyz::npz::NpzArchive::open(path)
            .map_err(|e| DomeSeeingError::Load(e, path.display().to_string()))?;
        Self::get_npyz(path.display(), &mut archive)
        // let mut values = vec![];
        // let mut mask = vec![];
        // for opd in archive
        //     .by_name("opd")
        //     .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?
        //     .unwrap()
        //     .into_vec::<f64>()
        //     .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?
        //     .into_iter()
        // {
        //     if opd.is_nan() {
        //         mask.push(false);
        //     } else {
        //         values.push(opd);
        //         mask.push(true);
        //     }
        // }
        // // let opd: Vec<_> = opd.into_iter().filter(|x| !f64::is_nan(x)).collect();
        // let n = values.len();
        // let mean = values.iter().sum::<f64>() / n as f64;
        // Ok(Opd { mean, values, mask })
    }
    #[cfg(all(feature = "npyz", not(feature = "bincode")))]
    pub fn get_npyz<R: io::Read + io::Seek>(
        path_to_archive: impl ToString,
        archive: &mut npyz::npz::NpzArchive<R>,
    ) -> Result<Opd> {
        let mut values = vec![];
        let mut mask = vec![];
        for opd in archive
            .by_name("opd")
            .map_err(|e| DomeSeeingError::Load(e, path_to_archive.to_string()))?
            .unwrap()
            .into_vec::<f64>()
            .map_err(|e| DomeSeeingError::Load(e, path_to_archive.to_string()))?
            .into_iter()
        {
            if opd.is_nan() {
                mask.push(false);
            } else {
                values.push(opd);
                mask.push(true);
            }
        }
        // let opd: Vec<_> = opd.into_iter().filter(|x| !f64::is_nan(x)).collect();
        let n = values.len();
        let mean = values.iter().sum::<f64>() / n as f64;
        Ok(Opd { mean, values, mask })
    }
    #[cfg(all(feature = "object_store", not(feature = "bincode")))]
    pub async fn get_npyz_from_store(
        store: &Arc<dyn ObjectStore>,
        path: &object_store::path::Path,
    ) -> Result<Opd> {
        use std::io::Cursor;
        let mut data = Vec::new();
        let mut reader = store.get(&path).await?.into_stream();
        while let Some(chunk) = reader.next().await {
            let chunk = chunk?;
            data.extend_from_slice(&chunk);
        }

        let cursor = Cursor::new(data);

        let mut archive = npyz::npz::NpzArchive::new(cursor)
            .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?;
        Self::get_npyz(path, &mut archive)
    }
    #[cfg(all(feature = "object_store", not(feature = "bincode")))]
    pub async fn get(&self, idx: usize) -> Result<Opd> {
        let path = &self
            .data
            .get(idx)
            .ok_or(DomeSeeingError::OutOfBounds(idx))?
            .file;

        // let mut data = Vec::new();
        // let mut reader = self
        //     .store
        //     .as_ref()
        //     .ok_or(DomeSeeingError::MissingStore)?
        //     .get(&path)
        //     .await?
        //     .into_stream();
        // while let Some(chunk) = reader.next().await {
        //     let chunk = chunk?;
        //     data.extend_from_slice(&chunk);
        // }

        // let cursor = Cursor::new(data);

        // let mut archive = npyz::npz::NpzArchive::new(cursor)
        //     .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?;
        Self::get_npyz_from_store(
            self.store.as_ref().ok_or(DomeSeeingError::MissingStore)?,
            path,
        )
        .await
        // let mut values = vec![];
        // let mut mask = vec![];
        // for opd in archive
        //     .by_name("opd")
        //     .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?
        //     .unwrap()
        //     .into_vec::<f64>()
        //     .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?
        //     .into_iter()
        // {
        //     if opd.is_nan() {
        //         mask.push(false);
        //     } else {
        //         values.push(opd);
        //         mask.push(true);
        //     }
        // }
        // // let opd: Vec<_> = opd.into_iter().filter(|x| !f64::is_nan(x)).collect();
        // let n = values.len();
        // let mean = values.iter().sum::<f64>() / n as f64;
        // Ok(Opd { mean, values, mask })
    }
}

impl Index<usize> for DomeSeeing {
    type Output = DomeSeeingData;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}
impl Iterator for DomeSeeing {
    type Item = Vec<f64>;
    fn next(&mut self) -> Option<Self::Item> {
        let i_cfd = self.i / self.upsampling;
        if self.i % self.upsampling == 0 {
            std::mem::swap(&mut self.y1, &mut self.y2);
            if let Some(idx) = self.counter.next() {
                #[cfg(not(feature = "object_store"))]
                {
                    self.y2 = self.get(idx).expect("failed to load dome seeing data file");
                }
                #[cfg(feature = "object_store")]
                {
                    let path = &self
                        .data
                        .get(idx)?
                        // .ok_or(DomeSeeingError::OutOfBounds(idx))?.ok()?
                        .file;
                    let store = self
                        .store
                        .as_ref()?
                        // .ok_or(DomeSeeingError::MissingStore)?
                        .clone();
                    self.y2 = tokio::task::block_in_place(move || {
                        {
                            tokio::runtime::Handle::current().block_on(async move {
                                Self::get_npyz_from_store(&store, path)
                                    .await
                                    .expect("failed to load dome seeing data file")
                            })
                        }
                    });
                }
            } else {
                return None;
            }
        };
        let alpha = (self.i - i_cfd * self.upsampling) as f64 / self.upsampling as f64;
        let y1 = &self.y1;
        let y2 = &self.y2;
        let opd_i = y1
            .values
            .iter()
            .zip(&y2.values)
            .map(|(y1, y2)| y1 + (y2 - y1) * alpha);
        match self.mapping {
            OpdMapping::Masked => Some(opd_i.collect()),
            OpdMapping::Whole => {
                let mut opd = vec![0f64; y1.mask.len()];
                opd.iter_mut()
                    .zip(&y1.mask)
                    .filter(|(_, mask)| **mask)
                    .zip(opd_i)
                    .for_each(|((opd, _), opd_i)| *opd = opd_i);
                self.i += 1;
                Some(opd)
            }
        }
    }
}

impl Update for DomeSeeing {
    fn update(&mut self) {
        self.opd = self.next().map(|opd| opd.into());
    }
}

impl Size<DomeSeeingOpd> for DomeSeeing {
    fn len(&self) -> usize {
        self.y2.mask.len()
    }
}
impl Write<DomeSeeingOpd> for DomeSeeing {
    fn write(&mut self) -> Option<Data<DomeSeeingOpd>> {
        self.opd.clone().map(|opd| opd.into())
    }
}

#[cfg(test)]
mod tests {
    use std::env;

    use super::*;

    #[cfg(not(feature = "object_store"))]
    #[test]
    fn load() -> std::result::Result<(), Box<dyn std::error::Error>> {
        if let Ok(path) = env::var("CFD_PATH") {
            let dome_seeing = DomeSeeing::builder(path).build()?;
            assert!(
                dome_seeing.len() > 1,
                "expected dome seeing len > 1, found dome seeing len = {}",
                dome_seeing.len()
            );
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }

        Ok(())
    }

    #[cfg(feature = "object_store")]
    #[tokio::test]
    async fn load() -> std::result::Result<(), Box<dyn std::error::Error>> {
        use std::sync::Arc;
        let s3: Arc<dyn object_store::ObjectStore> = Arc::new(
            object_store::aws::AmazonS3Builder::from_env()
                .with_region("sa-east-1")
                .with_bucket_name("maua.cfd.2025")
                .build()?,
        );
        if let Ok(path) = env::var("CFD_PATH") {
            let dome_seeing = DomeSeeing::builder(path)
                .store(s3.clone())
                .build()
                .await?;
            assert!(
                dome_seeing.len() > 1,
                "expected dome seeing len > 1, found dome seeing len = {}",
                dome_seeing.len()
            );
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }

        Ok(())
    }

    #[cfg(not(feature = "object_store"))]
    #[test]
    fn time() -> std::result::Result<(), Box<dyn std::error::Error>> {
        if let Ok(path) = env::var("CFD_PATH") {
            let dome_seeing = DomeSeeing::builder(path).build()?;
            assert!(dome_seeing[dome_seeing.len() - 1].time_stamp > dome_seeing[0].time_stamp);
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }
        Ok(())
    }

    #[cfg(feature = "object_store")]
    #[tokio::test]
    async fn time() -> std::result::Result<(), Box<dyn std::error::Error>> {
        use std::sync::Arc;
        let s3: Arc<dyn object_store::ObjectStore> = Arc::new(
            object_store::aws::AmazonS3Builder::from_env()
                .with_region("sa-east-1")
                .with_bucket_name("maua.cfd.2025")
                .build()?,
        );
        if let Ok(path) = env::var("CFD_PATH") {
            let dome_seeing = DomeSeeing::builder(path)
                .store(s3.clone())
                .build()
                .await?;
            assert!(dome_seeing[dome_seeing.len() - 1].time_stamp > dome_seeing[0].time_stamp);
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }

        Ok(())
    }

    #[cfg(not(feature = "object_store"))]
    #[test]
    fn next() -> std::result::Result<(), Box<dyn std::error::Error>> {
        if let Ok(path) = env::var("CFD_PATH") {
            let mut dome_seeing = DomeSeeing::builder(path).build()?;
            let mut opd = dome_seeing.next().unwrap();
            dbg!(opd.len());
            opd.sort_by(|a, b| a.partial_cmp(b).unwrap());
            dbg!((opd[0], opd.last().unwrap()));
            // assert!(dome_seeing[dome_seeing.len() - 1].time_stamp > dome_seeing[0].time_stamp);
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }
        Ok(())
    }

    #[cfg(feature = "object_store")]
    #[tokio::test(flavor = "multi_thread")]
    async fn next() -> std::result::Result<(), Box<dyn std::error::Error>> {
        use std::sync::Arc;
        let s3: Arc<dyn object_store::ObjectStore> = Arc::new(
            object_store::aws::AmazonS3Builder::from_env()
                .with_region("sa-east-1")
                .with_bucket_name("maua.cfd.2025")
                .build()?,
        );
        if let Ok(path) = env::var("CFD_PATH") {
            let mut dome_seeing = DomeSeeing::builder(path)
                .store(s3.clone())
                .build()
                .await?;
            let mut opd = dome_seeing.next().unwrap();
            dbg!(opd.len());
            opd.sort_by(|a, b| a.partial_cmp(b).unwrap());
            dbg!((opd[0], opd.last().unwrap()));
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }

        Ok(())
    }
}
