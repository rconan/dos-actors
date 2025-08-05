use glob::{GlobError, PatternError};
use gmt_dos_clients_io::domeseeing::DomeSeeingOpd;
use interface::{Data, Size, Update, Write};
use serde::{Deserialize, Serialize};
use std::{
    num::ParseFloatError,
    ops::Index,
    path::{Path, PathBuf},
    sync::Arc,
};

#[derive(Debug, thiserror::Error)]
pub enum DomeSeeingError {
    #[error("failed to load dome seeing data {1}")]
    Load(#[source] std::io::Error, PathBuf),
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
    pub file: PathBuf,
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

#[derive(Debug)]
enum OpdMapping {
    Whole,
    Masked,
}

impl DomeSeeing {
    /// Creates a [DomeSeeingBuilder] instance given the `path` to the CFD case
    pub fn builder(path: impl AsRef<Path>) -> DomeSeeingBuilder {
        DomeSeeingBuilder {
            cfd_case_path: path.as_ref().to_path_buf(),
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
        let file =
            std::fs::File::open(&path).map_err(|e| DomeSeeingError::Load(e, path.to_path_buf()))?;
        Ok(bincode::deserialize_from(&file)?)
    }
    #[cfg(all(feature = "npyz", not(feature = "bincode")))]
    pub fn get(&self, idx: usize) -> Result<Opd> {
        let path = &self
            .data
            .get(idx)
            .ok_or(DomeSeeingError::OutOfBounds(idx))?
            .file;
        let mut archive = npyz::npz::NpzArchive::open(path)
            .map_err(|e| DomeSeeingError::Load(e, path.to_path_buf()))?;
        let mut values = vec![];
        let mut mask = vec![];
        for opd in archive
            .by_name("opd")
            .map_err(|e| DomeSeeingError::Load(e, path.to_path_buf()))?
            .unwrap()
            .into_vec::<f64>()
            .map_err(|e| DomeSeeingError::Load(e, path.to_path_buf()))?
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
                self.y2 = self.get(idx).expect("failed to load dome seeing data file");
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
    use super::*;
    #[test]
    fn load() -> std::result::Result<(), Box<dyn std::error::Error>> {
        let _dome_seeing =
            DomeSeeing::builder("/home/ubuntu/mnt/CASES/zen30az000_CD_12ms").build()?;
        Ok(())
    }
    #[test]
    fn time() -> std::result::Result<(), Box<dyn std::error::Error>> {
        let dome_seeing =
            DomeSeeing::builder("/home/ubuntu/mnt/CASES/zen30az000_OS_2ms").build()?;
        assert!(dome_seeing[dome_seeing.len() - 1] > dome_seeing[0]);
        Ok(())
    }
}
