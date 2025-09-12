use gmt_dos_clients_io::domeseeing::DomeSeeingOpd;
use interface::{Data, Size, Update, Write};
#[cfg(feature = "npyz")]
use std::io;
#[cfg(not(feature = "object_store"))]
use std::path::Path;
use std::{ops::Index, sync::Arc};

#[cfg(not(feature = "object_store"))]
use crate::DomeSeeingBuilder;
use crate::{Counter, DomeSeeingData, DomeSeeingError, Opd, OpdMapping};

type Result<T> = std::result::Result<T, DomeSeeingError>;

/// Dome seeing time series
pub struct DomeSeeing {
    pub(crate) upsampling: usize,
    pub(crate) data: Vec<DomeSeeingData>,
    pub(crate) counter: Counter,
    pub(crate) i: usize,
    pub(crate) y1: Opd,
    pub(crate) y2: Opd,
    pub(crate) mapping: OpdMapping,
    pub(crate) opd: Option<Arc<Vec<f64>>>,
    #[cfg(feature = "object_store")]
    pub(crate) store: Option<Arc<dyn object_store::ObjectStore>>,
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

#[cfg(all(feature = "object_store", not(feature = "bincode")))]
mod store;

impl DomeSeeing {
    #[cfg(not(feature = "object_store"))]
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
    pub fn step(&mut self) -> Vec<f64> {
        let i_cfd = self.i / self.upsampling;
        let alpha = (self.i - i_cfd * self.upsampling) as f64 / self.upsampling as f64;
        let y1 = &self.y1;
        let y2 = &self.y2;
        let opd_i = y1
            .values
            .iter()
            .zip(&y2.values)
            .map(|(y1, y2)| y1 + (y2 - y1) * alpha);
        match self.mapping {
            OpdMapping::Masked => opd_i.collect(),
            OpdMapping::Whole => {
                let mut opd = vec![0f64; y1.mask.len()];
                opd.iter_mut()
                    .zip(&y1.mask)
                    .filter(|(_, mask)| **mask)
                    .zip(opd_i)
                    .for_each(|((opd, _), opd_i)| *opd = opd_i);
                self.i += 1;
                opd
            }
        }
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
        if self.i % self.upsampling == 0 {
            std::mem::swap(&mut self.y1, &mut self.y2);
            if let Some(idx) = self.counter.next() {
                #[cfg(not(feature = "object_store"))]
                {
                    self.y2 = self.get(idx).expect("failed to load dome seeing data file");
                }
                #[cfg(feature = "object_store")]
                {
                    // requires tokio feature rt-multi-thread
                    // or use the Stream implementation with the object_store feature
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
        Some(self.step())
    }
}

impl Update for DomeSeeing {
    fn update(&mut self) {
        self.opd = Iterator::next(self).map(|opd| opd.into());
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

#[cfg(not(feature = "object_store"))]
#[cfg(test)]
mod tests {
    use std::env;

    use super::*;

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

    #[test]
    fn next() -> std::result::Result<(), Box<dyn std::error::Error>> {
        if let Ok(path) = env::var("CFD_PATH") {
            let dome_seeing = DomeSeeing::builder(path).build()?;
            for (i, mut opd) in dome_seeing.enumerate().take(5) {
                opd.sort_by(|a, b| a.partial_cmp(b).unwrap());
                println!(
                    "#{i:02}: [{:+6.0},{:+6.0}]nm",
                    opd[0] * 1e9,
                    1e9 * opd.last().unwrap()
                );
            }
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }
        Ok(())
    }
}
