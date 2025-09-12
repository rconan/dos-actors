use glob::{GlobError, PatternError};
use serde::{Deserialize, Serialize};
use std::num::ParseFloatError;
#[cfg(not(feature = "object_store"))]
use std::path::PathBuf;

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
    #[cfg(feature = "object_store")]
    #[error("no dome seeing data found in {0}")]
    MissingData(String),
}

mod builder;
mod dome_seeing;
pub use builder::DomeSeeingBuilder;
pub use dome_seeing::DomeSeeing;


type Counter = Box<dyn Iterator<Item = usize> + Send>;

//const CFD_SAMPLING_FREQUENCY: f64 = 5f64; // Hz

#[derive(Debug, Default, Clone)]
pub enum OpdMapping {
    #[default]
    Whole,
    Masked,
}

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
