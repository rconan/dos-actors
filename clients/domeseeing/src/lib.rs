use glob::{GlobError, PatternError, glob};
use gmt_dos_clients_io::domeseeing::DomeSeeingOpd;
use interface::{Data, Size, Update, Write};
use serde::{Deserialize, Serialize};
use std::{
    fs::File,
    path::{Path, PathBuf},
};

#[derive(Debug, thiserror::Error)]
pub enum DomeSeeingError {
    #[error("failed to load dome seeing data")]
    Load(#[from] std::io::Error),
    #[error("failed to get dome seeing data path")]
    Glob(#[from] GlobError),
    #[error("failed to find dome seeing file pattern")]
    Pattern(#[from] PatternError),
    #[error("failed to read dome seeing file")]
    Bincode(#[from] bincode::Error),
    #[error("dome seeing index {0} is out-of-bounds")]
    OutOfBounds(usize),
}

pub type Result<T> = std::result::Result<T, DomeSeeingError>;

//const CFD_SAMPLING_FREQUENCY: f64 = 5f64; // Hz

/// Dome seeing OPD
///
/// The OPD `values` are given only inside the `mask` (i.e. where the mask is `true`)
#[derive(Serialize, Deserialize, Debug, Default)]
pub struct Opd {
    pub mean: f64,
    pub values: Vec<f64>,
    pub mask: Vec<bool>,
}

#[derive(Debug)]
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
    /// Creates a new dome seeing time series object
    ///
    /// The arguments are the `path` to the CFD dome seeing OPD and the `upsampling` factor
    /// i.e. the ratio between the desired OPD sampling frequency and the CFD sampling frequency (usually 5Hz)
    pub fn new<P: AsRef<str> + std::fmt::Display>(
        path: P,
        upsampling: usize,
        take: Option<usize>,
    ) -> Result<Self> {
        let mut data: Vec<DomeSeeingData> = Vec::with_capacity(2005);
        for entry in glob(&format!("{}/optvol/optvol_optvol_*", path))? {
            let time_stamp = entry
                .as_ref()
                .ok()
                .and_then(|x| x.file_name())
                .and_then(|x| Path::new(x).file_stem())
                .and_then(|x| x.to_str())
                .and_then(|x| x.split("_").last())
                .and_then(|x| x.parse::<f64>().ok())
                .expect("failed to parse dome seeing time stamp");
            data.push(DomeSeeingData {
                time_stamp,
                file: entry?,
            });
        }
        data.sort_by(|a, b| a.time_stamp.partial_cmp(&b.time_stamp).unwrap());
        let mut counter = if let Some(take) = take {
            Box::new(
                (0..data.len())
                    .chain((0..data.len()).skip(1).rev().skip(1))
                    .cycle()
                    .take(take),
            ) as Counter
        } else {
            Box::new(
                (0..data.len())
                    .chain((0..data.len()).skip(1).rev().skip(1))
                    .cycle(),
            ) as Counter
        };
        if let Some(c) = counter.next() {
            let y2: Opd = bincode::deserialize_from(&File::open(&data[c].file)?)?;
            //dbg!(y2.values.len());
            //dbg!(y2.mask.len());
            Ok(Self {
                upsampling,
                data,
                counter,
                i: 0,
                y1: Default::default(),
                y2,
                mapping: OpdMapping::Whole,
            })
        } else {
            Ok(Self {
                upsampling,
                data,
                counter,
                i: 0,
                y1: Default::default(),
                y2: Default::default(),
                mapping: OpdMapping::Whole,
            })
        }
    }
    pub fn masked(mut self) -> Self {
        self.mapping = OpdMapping::Masked;
        self
    }
    pub fn len(&self) -> usize {
        self.data.len()
    }
    pub fn get(&self, idx: usize) -> Result<Opd> {
        let file = File::open(
            &self
                .data
                .get(idx)
                .ok_or(DomeSeeingError::OutOfBounds(idx))?
                .file,
        )?;
        Ok(bincode::deserialize_from(&file)?)
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

impl Update for DomeSeeing {}

impl Size<DomeSeeingOpd> for DomeSeeing {
    fn len(&self) -> usize {
        self.y2.mask.len()
    }
}
impl Write<DomeSeeingOpd> for DomeSeeing {
    fn write(&mut self) -> Option<Data<DomeSeeingOpd>> {
        self.next().map(|x| Data::new(x))
    }
}

#[cfg(test)]
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
}
