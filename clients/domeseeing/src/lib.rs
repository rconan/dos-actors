use glob::{GlobError, PatternError, glob};
use gmt_dos_clients_io::domeseeing::DomeSeeingOpd;
use interface::{Data, Size, Update, Write};
use serde::{Deserialize, Serialize};
use std::{
    num::ParseFloatError,
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

pub type Result<T> = std::result::Result<T, DomeSeeingError>;

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

#[derive(Debug, Default, Clone)]
struct DomeSeeingData {
    time_stamp: f64,
    file: PathBuf,
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

#[derive(Default, Debug, Clone)]
pub struct DomeSeeingBuilder {
    cfd_case_path: PathBuf,
    upsampling: Option<usize>,
    take: Option<usize>,
    cycle: bool,
}
impl DomeSeeingBuilder {
    /// Sets the ratio (>=1) between the desired OPD sampling frequency and the CFD sampling frequency (usually 5Hz)
    pub fn upsampling_ratio(mut self, upsampling_ratio: usize) -> Self {
        self.upsampling = Some(upsampling_ratio);
        self
    }
    /// Sets the size of the dome seeing sample
    pub fn sample_size(mut self, size: usize) -> Self {
        self.take = Some(size);
        self
    }
    /// Cycles though the dome seeing OPD back and forth
    pub fn cycle(mut self) -> Self {
        self.cycle = true;
        self
    }
    /// Creates a new [DomeSeeing] instance
    pub fn build(self) -> Result<DomeSeeing> {
        let mut data: Vec<DomeSeeingData> = Vec::with_capacity(2005);
        for entry in glob(
            self.cfd_case_path
                .join("optvol")
                .join(if cfg!(feature = "bincode") {
                    "optvol_optvol_*.bin"
                } else {
                    "optvol_optvol_*.npz"
                })
                .as_os_str()
                .to_str()
                .unwrap(),
        )? {
            let file = entry?;
            let time_stamp = file
                .with_extension("")
                // .as_ref()
                .file_stem()
                // .and_then(|x| file_stem())
                .and_then(|x| x.to_str())
                .and_then(|x| x.split("_").last())
                .and_then(|x| {
                    Some(
                        x.parse::<f64>()
                            .map_err(|e| DomeSeeingError::TimeStamp(e, x.into())),
                    )
                })
                .transpose()?
                .unwrap();
            // .expect("failed to parse dome seeing time stamp");
            data.push(DomeSeeingData { time_stamp, file });
        }
        data.sort_by(|a, b| a.time_stamp.partial_cmp(&b.time_stamp).unwrap());
        let counter = match (self.take, self.cycle) {
            (None, true) => Box::new(
                (0..data.len())
                    .chain((0..data.len()).skip(1).rev().skip(1))
                    .cycle(),
            ) as Counter,
            (None, false) => Box::new(0..data.len()) as Counter,
            (Some(take), true) => Box::new(
                (0..data.len())
                    .chain((0..data.len()).skip(1).rev().skip(1))
                    .cycle()
                    .take(take),
            ) as Counter,
            (Some(take), false) => Box::new((0..data.len()).take(take)) as Counter,
        };
        // let counter = if let Some(take) = self.take {
        //     Box::new(
        //         (0..data.len())
        //             .chain((0..data.len()).skip(1).rev().skip(1))
        //             .cycle()
        //             .take(take),
        //     ) as Counter
        // } else {
        //     Box::new(
        //         (0..data.len())
        //             .chain((0..data.len()).skip(1).rev().skip(1))
        //             .cycle(),
        //     ) as Counter
        // };
        let mut this = DomeSeeing {
            upsampling: self.upsampling.unwrap_or(1),
            data,
            counter,
            i: 0,
            y1: Default::default(),
            y2: Default::default(),
            mapping: OpdMapping::Whole,
            opd: None,
        };
        if let Some(c) = this.counter.next() {
            // let y2: Opd = bincode::deserialize_from(&File::open(&data[c].file)?)?;
            //dbg!(y2.values.len());
            //dbg!(y2.mask.len());
            this.y2 = this.get(c)?;
        };
        Ok(this)
    }
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

/* #[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn load() {
        let n = 3;
        let dome_seeing: DomeSeeing =
            DomeSeeing::new("/fsx/CASES/zen30az000_OS7/", 1, Some(n)).unwrap();
    }

    #[test]
    fn next() {
        let n = 4;
        const N: usize = 5;
        let mut dome_seeing: DomeSeeing =
            DomeSeeing::new("/fsx/CASES/zen30az000_OS7/", N, Some(n)).unwrap();
        let mut i = 0;
        while let Some(opd) = dome_seeing.next() {
            let val = 1e9 * opd[123456];
            if i % N == 0 {
                println!("{:9.3} *", val);
            } else {
                println!("{:9.3}", val);
            }
            i += 1;
        }
        //assert_eq!(vals[0], *vals.last().unwrap());
    }
} */
