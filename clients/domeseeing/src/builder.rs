#[cfg(not(feature = "object_store"))]
use std::path::PathBuf;
#[cfg(feature = "object_store")]
use std::sync::Arc;

use crate::{Counter, DomeSeeing, DomeSeeingData, DomeSeeingError, OpdMapping};
#[cfg(feature = "object_store")]
use futures::TryStreamExt;
#[cfg(feature = "object_store")]
use object_store::{ObjectStore, path::Path};

type Result<T> = std::result::Result<T, DomeSeeingError>;

#[derive(Default, Debug)]
pub struct DomeSeeingBuilder {
    #[cfg(not(feature = "object_store"))]
    pub(crate) cfd_case_path: PathBuf,
    #[cfg(feature = "object_store")]
    pub(crate) cfd_case_path: Path,
    pub(crate) upsampling: Option<usize>,
    pub(crate) take: Option<usize>,
    pub(crate) cycle: bool,
    #[cfg(feature = "object_store")]
    pub(crate) store: Option<Arc<dyn ObjectStore>>,
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
    #[cfg(feature = "object_store")]
    /// Sets the remote storage where the dome seeing data is saved
    pub fn store(mut self, store: impl ObjectStore) -> Self {
        self.store = Some(Arc::new(store));
        self
    }
    #[cfg(not(feature = "object_store"))]
    /// Creates a new [DomeSeeing] instance
    pub fn build(self) -> Result<DomeSeeing> {
        use glob::glob;
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

        // .file_stem()
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
            #[cfg(feature = "object_store")]
            store: self.store,
        };
        if let Some(c) = this.counter.next() {
            // let y2: Opd = bincode::deserialize_from(&File::open(&data[c].file)?)?;
            //dbg!(y2.values.len());
            //dbg!(y2.mask.len());
            this.y2 = this.get(c)?;
        };
        Ok(this)
    }
    #[cfg(feature = "object_store")]
    /// Creates a new [DomeSeeing] instance
    pub async fn build(self) -> Result<DomeSeeing> {
        let mut data: Vec<DomeSeeingData> = Vec::with_capacity(2005);

        let path = object_store::path::Path::from(format!("{}/optvol", self.cfd_case_path));
        let mut objects = self
            .store
            .as_ref()
            .ok_or(DomeSeeingError::MissingStore)?
            .list(Some(&path));
        while let Some(object) = objects.try_next().await.unwrap() {
            let path = object.location;
            if path.to_string().ends_with(".npz") {
                let time_stamp = path
                    .filename()
                    .and_then(|x| x.strip_suffix(".npz"))
                    .and_then(|x| x.split("_").last())
                    .and_then(|x| {
                        Some(
                            x.parse::<f64>()
                                .map_err(|e| DomeSeeingError::TimeStamp(e, x.into())),
                        )
                    })
                    .transpose()?
                    .unwrap();
                data.push(DomeSeeingData {
                    time_stamp,
                    file: path,
                });
            }
        }
        if data.is_empty() {
            return Err(DomeSeeingError::MissingData(path.into()));
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
            #[cfg(feature = "object_store")]
            store: self.store,
        };
        if let Some(c) = this.counter.next() {
            // let y2: Opd = bincode::deserialize_from(&File::open(&data[c].file)?)?;
            //dbg!(y2.values.len());
            //dbg!(y2.mask.len());
            this.y2 = this.get(c).await?;
        };
        Ok(this)
    }
}
