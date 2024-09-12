use std::{
    env,
    fs::{DirBuilder, File},
    path::Path,
    sync::Arc,
};

use apache_arrow::{
    array::Array,
    compute::concat_batches,
    datatypes::{DataType, Field, Schema},
    record_batch::{RecordBatch, RecordBatchReader},
};
use interface::print_info;
use parquet::{
    arrow::{arrow_reader::ParquetRecordBatchReaderBuilder, ArrowWriter},
    file::properties::WriterProperties,
};

use crate::{Arrow, ArrowError, DropOption, FileFormat, Result};

impl Arrow {
    /// Writes the record to file
    pub fn save(&mut self) -> &mut Self {
        match self.drop_option {
            DropOption::Save(ref filename) => {
                let file_name = filename
                    .as_ref()
                    .cloned()
                    .unwrap_or_else(|| "data".to_string());
                match self.file_format {
                    FileFormat::Parquet => {
                        if let Err(e) = self.to_parquet(file_name) {
                            print_info("Arrow error", Some(&e));
                        }
                    }
                    #[cfg(feature = "matio-rs")]
                    FileFormat::Matlab(_) => {
                        if let Err(e) = self.to_mat(file_name) {
                            print_info("Arrow error", Some(&e));
                        }
                    }
                }
            }
            DropOption::NoSave => {
                log::info!("no saving option set");
            }
        }
        self
    }
    /// Moves the record into the record batch
    pub fn batch(&mut self) -> Result<&Vec<RecordBatch>> {
        self.record()?;
        if let Some(record) = self.record.take() {
            self.batch.get_or_insert(vec![]).push(record);
        }
        self.batch.as_ref().ok_or(ArrowError::NoRecord)
    }
    /// Concatenates batch of records together into a single record batch
    pub fn concat_batches(&mut self) -> Result<RecordBatch> {
        self.batch().and_then(|batches| {
            let schema = batches[0].schema();
            let record = concat_batches(&schema, batches)?;
            Ok(record)
        })
    }
    /// Returns the data record
    pub fn record(&mut self) -> Result<&RecordBatch> {
        if self.record.is_none() {
            let mut lists: Vec<Arc<dyn Array>> = vec![];
            for ((buffer, buffer_data_type), n) in self.buffers.iter_mut().zip(&self.capacities) {
                let list = buffer.into_list(
                    self.batch_size.unwrap_or(self.count / self.n_entry),
                    *n,
                    buffer_data_type.clone(),
                )?;
                lists.push(Arc::new(list));
            }

            let fields: Vec<_> = self
                .buffers
                .iter()
                .map(|(buffer, data_type)| {
                    Field::new(
                        &buffer.who(),
                        DataType::List(Box::new(Field::new("values", data_type.clone(), false))),
                        false,
                    )
                })
                .collect();
            let schema = Arc::new(if let Some(metadata) = self.metadata.as_ref() {
                Schema::new_with_metadata(fields, metadata.clone())
            } else {
                Schema::new(fields)
            });

            self.record = Some(RecordBatch::try_new(Arc::clone(&schema), lists)?);
        }
        self.record.as_ref().ok_or(ArrowError::NoRecord)
    }
    /// Saves the data to a [Parquet](https://docs.rs/parquet) data file
    ///
    /// The [Parquet](https://docs.rs/parquet) data file is saved in the current directory
    /// unless the environment variable `DATA_REPO` is set to another directory.
    /// We will try to create the directory if does not exist.
    pub fn to_parquet<P: AsRef<Path> + std::fmt::Debug>(&mut self, path: P) -> Result<()> {
        // let batch = self.record()?;
        let batch = self.concat_batches()?;
        let root_env = env::var("DATA_REPO").unwrap_or_else(|_| ".".to_string());
        let root = Path::new(&root_env).join(&path).with_extension("parquet");
        if let Some(path) = root.parent() {
            if !path.is_dir() {
                DirBuilder::new().recursive(true).create(&path)?;
            }
        };
        let file = File::create(&root)?;
        let props = WriterProperties::builder().build();
        let mut writer = ArrowWriter::try_new(file, Arc::clone(&batch.schema()), Some(props))?;
        writer.write(&batch)?;
        writer.close()?;
        log::info!("Arrow data saved to {root:?}");
        Ok(())
    }
    /// Loads data from a [Parquet](https://docs.rs/parquet) data file
    ///
    /// The [Parquet](https://docs.rs/parquet) data file is loaded from the current
    /// directory unless the environment variable `DATA_REPO` is set to another directory.
    /// We will try to create the directory if does not exist.
    pub fn from_parquet<P>(path: P) -> Result<Self>
    where
        P: AsRef<Path>,
    {
        let root_env = env::var("DATA_REPO").unwrap_or_else(|_| ".".to_string());
        let root = Path::new(&root_env);
        let filename = root.join(&path).with_extension("parquet");
        if let Some(path) = filename.parent() {
            if !path.is_dir() {
                DirBuilder::new().recursive(true).create(&path)?;
            }
        };
        let file = File::open(&filename)?;
        log::info!("Loading {:?}", filename);
        let parquet_reader = ParquetRecordBatchReaderBuilder::try_new(file)?
            .with_batch_size(2048)
            .build()?;
        let schema = parquet_reader.schema();
        let records: std::result::Result<Vec<_>, apache_arrow::error::ArrowError> =
            parquet_reader.collect();
        let record = concat_batches(&schema, records?.as_slice())?;
        Ok(Arrow {
            n_step: 0,
            capacities: Vec::new(),
            buffers: Vec::new(),
            metadata: None,
            step: 0,
            n_entry: 0,
            record: Some(record),
            batch: None,
            drop_option: DropOption::NoSave,
            decimation: 1,
            count: 0,
            file_format: FileFormat::Parquet,
            batch_size: None,
        })
    }
    #[cfg(feature = "matio-rs")]
    /// Saves the data to a Matlab "mat" file
    ///
    /// All data must be of the type `Vec<f64>`
    pub fn to_mat<P: AsRef<Path>>(&mut self, path: P) -> Result<()> {
        use matio_rs::MatFile;
        let batch = self.concat_batches()?;
        let root_env = env::var("DATA_REPO").unwrap_or_else(|_| ".".to_string());
        let root = Path::new(&root_env).join(&path).with_extension("mat");
        let mat_file = MatFile::save(&root)?;
        let mut n_sample = 0;
        for field in batch.schema().fields() {
            let name = field.name();
            let data: Vec<Vec<f64>> = self.iter(name)?.collect();
            n_sample = data.len();
            let n_data = data[0].len();
            mat_file.array(
                name,
                data.into_iter()
                    .flatten()
                    .collect::<Vec<f64>>()
                    .as_mut_slice(),
                vec![n_data as u64, n_sample as u64],
            )?;
        }
        if let FileFormat::Matlab(crate::MatFormat::TimeBased(sampling_frequency)) =
            self.file_format
        {
            let tau = sampling_frequency.recip();
            let time: Vec<f64> = (0..n_sample).map(|i| i as f64 * tau).collect();
            mat_file.var("time", time.as_slice())?;
        }
        log::info!("Arrow data saved to {root:?}");
        Ok(())
    }
}
