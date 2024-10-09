use std::{collections::HashMap, fmt::Display, mem::size_of};

use apache_arrow::{
    array::BufferBuilder,
    datatypes::{ArrowNativeType, DataType},
    record_batch::RecordBatch,
};
use interface::{print_info, Entry, UniqueIdentifier};

use crate::{
    ArrowBuffer, BufferDataType, BufferObject, DropOption, FileFormat, LogData, MAX_CAPACITY_BYTE,
};

mod arrow;
mod builder;
// mod get;
mod iter;
pub use builder::ArrowBuilder;

/// Apache [Arrow](https://docs.rs/arrow) client
pub struct Arrow {
    n_step: usize,
    capacities: Vec<usize>,
    buffers: Vec<(Box<dyn BufferObject>, DataType)>,
    metadata: Option<HashMap<String, String>>,
    pub(crate) step: usize,
    pub(crate) n_entry: usize,
    record: Option<RecordBatch>,
    batch: Option<Vec<RecordBatch>>,
    drop_option: DropOption,
    pub(crate) decimation: usize,
    pub(crate) count: usize,
    file_format: FileFormat,
    pub(crate) batch_size: Option<usize>,
}
impl Default for Arrow {
    fn default() -> Self {
        Arrow {
            n_step: 0,
            capacities: Vec::new(),
            buffers: Vec::new(),
            metadata: None,
            step: 0,
            n_entry: 0,
            record: None,
            batch: None,
            drop_option: DropOption::NoSave,
            decimation: 1,
            count: 0,
            file_format: Default::default(),
            batch_size: None,
        }
    }
}
impl Arrow {
    /// Creates a new Apache [Arrow](https://docs.rs/arrow) data logger
    ///
    ///  - `n_step`: the number of time step
    pub fn builder(n_step: usize) -> ArrowBuilder {
        ArrowBuilder::new(n_step)
    }
    pub(crate) fn data<T, U>(&mut self) -> Option<&mut LogData<ArrowBuffer<U>>>
    where
        T: 'static + ArrowNativeType,
        U: 'static + UniqueIdentifier<DataType = Vec<T>>,
    {
        self.buffers
            .iter_mut()
            .find_map(|(b, _)| b.as_mut_any().downcast_mut::<LogData<ArrowBuffer<U>>>())
    }
    pub fn pct_complete(&self) -> usize {
        self.step / self.n_step / self.n_entry
    }
    pub fn size(&self) -> usize {
        self.step / self.n_entry
    }
}

impl<T, U> Entry<U> for Arrow
where
    T: 'static + BufferDataType + ArrowNativeType + Send + Sync,
    U: 'static + Send + Sync + UniqueIdentifier<DataType = Vec<T>>,
{
    fn entry(&mut self, size: usize) {
        let mut capacity = size * (1 + self.n_step / self.decimation);
        //log::info!("Buffer capacity: {}", capacity);
        if capacity * size_of::<T>() > MAX_CAPACITY_BYTE {
            capacity = MAX_CAPACITY_BYTE / size_of::<T>();
            log::info!("Capacity limit of 1GB exceeded, reduced to : {}", capacity);
        }
        let buffer: LogData<ArrowBuffer<U>> = LogData::new(BufferBuilder::<T>::new(capacity));
        self.buffers.push((Box::new(buffer), T::buffer_data_type()));
        self.capacities.push(size);
        self.n_entry += 1;
    }
}
/*
impl<T, U> Entry<Vec<T>, U> for Arrow
where
    T: 'static + BufferDataType + ArrowNativeType + Send + Sync,
    U: 'static + Send + Sync,
{
    fn entry(&mut self, size: usize) {
        <Arrow as Entry<T, U>>::entry(self, size);
    }
}
 */
impl Display for Arrow {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.n_entry > 0 {
            writeln!(f, "Arrow logger:")?;
            writeln!(f, " - data:")?;
            for ((buffer, _), capacity) in self.buffers.iter().zip(self.capacities.iter()) {
                writeln!(f, "   - {:>8}:{:>4}", buffer.who(), capacity)?;
            }
            write!(
                f,
                " - steps #: {}/{}/{}",
                self.n_step,
                self.step / self.n_entry,
                self.count / self.n_entry
            )?;
            return Ok(());
        }
        if let Some(record) = &self.record {
            write!(
                f,
                "Arrow logger {:?}:\n{:}",
                (record.num_rows(), record.num_columns()),
                record
                    .schema()
                    .all_fields()
                    .iter()
                    .step_by(2)
                    .map(|field| format!(" - {}", field.name()))
                    .collect::<Vec<_>>()
                    .join("\n")
            )?;
            return Ok(());
        }
        Ok(())
    }
}

impl Drop for Arrow {
    fn drop(&mut self) {
        log::info!("{self}");
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
                log::info!("Dropping Arrow logger without saving.");
            }
        }
    }
}
