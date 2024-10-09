/*!
# Actor client for Apache Arrow

A simulation data logger that records the data in the [Apache Arrow] format and
automatically saves the data into a [Parquet] file (`data.parquet`) at the end of a simulation.

[Apache Arrow]: https://docs.rs/arrow
[Parquet]: https://docs.rs/parquet

# Example

An Arrow logger setup for 1000 time steps
```
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_actors::prelude::*;
let logging = Arrow::builder(1000).build();
```
setting the name of the Parquet file
```
# use gmt_dos_clients_arrow::Arrow;
# use gmt_dos_actors::prelude::*;

let logging = Arrow::builder(1000)
                       .filename("my_data.parquet")
                       .build();
```
opting out of saving the data to the Parquet file
```
# use gmt_dos_clients_arrow::Arrow;
# use gmt_dos_actors::prelude::*;

let logging = Arrow::builder(1000)
                       .no_save()
                       .build();
```
Logging an output into an [Arrow] logger:
```
# tokio_test::block_on(async {
use gmt_dos_actors::prelude::*;
use gmt_dos_clients::Signals;
use gmt_dos_clients_arrow::Arrow;
use interface::UID;

let logging = Arrow::builder(1000).build().into_arcx();
let mut sink = Terminator::<_>::new(logging);
let mut source: Initiator<_> = Signals::new(1, 100).into();
#[derive(UID)]
enum Source {};
source.add_output().build::<Source>().logn(&mut sink, 42).await;
# Ok::<(), gmt_dos_actors::model::ModelError>(())
# });
```
or if `Signals` implements the trait: `Size<Source>`
```
# tokio_test::block_on(async {
use gmt_dos_actors::prelude::*;
use gmt_dos_clients::Signals;
use gmt_dos_clients_arrow::Arrow;
use interface::{Size, UID};

let logging = Arrow::builder(1000).build().into_arcx();
let mut sink = Terminator::<_>::new(logging);
let mut source: Initiator<_> = Signals::new(1, 100).into();
#[derive(UID)]
enum Source {};
impl Size<Source> for Signals {
    fn len(&self) -> usize {
        42
    }
}
source.add_output().build::<Source>().log(&mut sink).await;
# Ok::<(), gmt_dos_actors::model::ModelError>(())
# });
```
*/

use apache_arrow::datatypes::ArrowNativeType;
use apache_arrow::{
    array::{ArrayData, BufferBuilder, ListArray},
    buffer::Buffer,
    datatypes::{DataType, Field, ToByteSlice},
};
use interface::{Data, Read, UniqueIdentifier, Update};
use std::{
    any::{type_name, Any},
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

#[derive(Debug, thiserror::Error)]
pub enum ArrowError {
    #[error("cannot open a parquet file")]
    ArrowToFile(#[from] std::io::Error),
    #[error("cannot build Arrow data")]
    ArrowError(#[from] apache_arrow::error::ArrowError),
    #[error("cannot save data to Parquet")]
    ParquetError(#[from] parquet::errors::ParquetError),
    #[error("no record available")]
    NoRecord,
    #[error("Field {0} not found")]
    FieldNotFound(String),
    #[error("Parsing field {0} failed")]
    ParseField(String),
    #[cfg(feature = "matio-rs")]
    #[error("failed to save data to mat file")]
    MatFile(#[from] matio_rs::MatioError),
}

type Result<T> = std::result::Result<T, ArrowError>;

// Buffers 1GB max capacity
const MAX_CAPACITY_BYTE: usize = 2 << 29;

/// Format to write data to file
///
/// Use parquet as the default file format
pub enum FileFormat {
    Parquet,
    #[cfg(feature = "matio-rs")]
    Matlab(MatFormat),
}
impl Default for FileFormat {
    fn default() -> Self {
        Self::Parquet
    }
}
/// Matlab data format
///
/// The Matlab data format is either `SampleBased` and does not include the time vector
/// or is `TimeBased` and does include a time vector.
/// The default format is `SampledBased`
pub enum MatFormat {
    SampleBased,
    TimeBased(f64),
}
impl Default for MatFormat {
    fn default() -> Self {
        Self::SampleBased
    }
}

/// Buffers generic interface
trait BufferObject: Send + Sync {
    fn who(&self) -> String;
    #[allow(dead_code)]
    fn as_any(&self) -> &dyn Any;
    fn as_mut_any(&mut self) -> &mut dyn Any;
    fn into_list(&mut self, n_step: usize, n: usize, data_type: DataType) -> Result<ListArray>;
}

/// Arrow buffer type match to a dos-actors Data type
struct ArrowBuffer<U: UniqueIdentifier>(PhantomData<U>);
impl<T: ArrowNativeType, U: UniqueIdentifier<DataType = Vec<T>>> UniqueIdentifier
    for ArrowBuffer<U>
{
    type DataType = BufferBuilder<T>;
}
struct LogData<U: UniqueIdentifier>(<U as UniqueIdentifier>::DataType, PhantomData<U>);
impl<U: UniqueIdentifier> Deref for LogData<U> {
    type Target = <U as UniqueIdentifier>::DataType;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<U: UniqueIdentifier> DerefMut for LogData<U> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl<T, U: UniqueIdentifier<DataType = T>> LogData<U> {
    pub fn new(data: T) -> Self {
        Self(data, PhantomData)
    }
}
impl<T, U> BufferObject for LogData<ArrowBuffer<U>>
where
    T: ArrowNativeType,
    U: 'static + Send + Sync + UniqueIdentifier<DataType = Vec<T>>,
{
    fn who(&self) -> String {
        type_name::<U>()
            .split("<")
            .map(|x| format!("{}", x.split("::").last().unwrap()))
            .collect::<Vec<_>>()
            .join("<")
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_mut_any(&mut self) -> &mut dyn Any {
        self
    }
    fn into_list(&mut self, n_step: usize, n: usize, data_type: DataType) -> Result<ListArray> {
        let buffer = &mut *self;
        let data = ArrayData::builder(data_type.clone())
            .len(buffer.len())
            .add_buffer(buffer.finish())
            .build()?;
        let offsets = (0..).step_by(n).take(n_step + 1).collect::<Vec<i32>>();
        let list = ArrayData::builder(DataType::List(Box::new(Field::new(
            "values", data_type, false,
        ))))
        .len(n_step)
        .add_buffer(Buffer::from(&offsets.to_byte_slice()))
        .add_child_data(data)
        .build()?;
        Ok(ListArray::from(list))
    }
}

#[doc(hidden)]
pub trait BufferDataType {
    type ArrayType;
    fn buffer_data_type() -> DataType;
}
use paste::paste;
macro_rules! impl_buffer_types {
    ( $( ($rs:ty,$arw:expr) ),+ ) => {
	    $(
        paste! {
impl BufferDataType for $rs {
    type ArrayType = apache_arrow::datatypes::[<$arw Type>];
    fn buffer_data_type() -> DataType {
        apache_arrow::datatypes::DataType::$arw
    }
}
        }
		)+
    };
}

impl_buffer_types! {
(f64,Float64),
(f32,Float32),
(i64,Int64),
(i32,Int32),
(i16,Int16),
(i8 ,Int8),
(u64,UInt64),
(u32,UInt32),
(u16,UInt16),
(u8 ,UInt8)
}

enum DropOption {
    Save(Option<String>),
    NoSave,
}

mod arrow;
pub use arrow::{Arrow, ArrowBuilder};

impl Update for Arrow {}
impl<T, U> Read<U> for Arrow
where
    T: ArrowNativeType,
    U: 'static + UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        let r = 1 + (self.step as f64 / self.n_entry as f64).floor() as usize;
        self.step += 1;
        if r % self.decimation > 0 {
            return;
        }
        if let Some(buffer) = self.data::<T, U>() {
            buffer.append_slice(&data);
            self.count += 1;
            match self.batch_size {
                Some(batch_size) if self.count % (self.n_entry * batch_size) == 0 => {
                    self.save();
                }
                _ => (),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use apache_arrow::datatypes::Schema;
    use interface::{Data, Entry, UID};

    use super::*;

    #[test]
    fn who() {
        let exp = "gmt_dos_clients_io::optics::dispersed_fringe_sensor::DfsFftFrame<gmt_dos_clients_io::optics::Host<a::b::c>>";
        let q = exp
            .split("<")
            .map(|x| format!("{}", x.split("::").last().unwrap()))
            .collect::<Vec<_>>()
            .join("<");
        dbg!(q);
    }

    #[test]
    fn arrow() {
        let mut arw = Arrow::builder(10).build();
        #[derive(UID)]
        pub enum Data {}
        <Arrow as Entry<Data>>::entry(&mut arw, 1);

        let field = Field::new(
            "Data",
            DataType::List(Box::new(Field::new("values", DataType::Float64, false))),
            false,
        );
        let schema = Arc::new(Schema::new(vec![field]));
        assert_eq!(arw.record().unwrap().schema(), schema);
    }

    #[test]
    fn batch() {
        //env_logger::init();
        let n_step = 8;
        let mut arw = Arrow::builder(n_step).batch_size(n_step / 2).build();
        #[derive(UID)]
        pub enum U {}
        <Arrow as Entry<U>>::entry(&mut arw, 1);
        for i in 0..n_step {
            arw.read(Data::<U>::new(vec![i as f64]));
        }
    }

    #[test]
    fn batch2() {
        //env_logger::init();
        let n_step = 24;
        let mut arw = Arrow::builder(n_step).batch_size(4).build();
        #[derive(UID)]
        pub enum U {}
        <Arrow as Entry<U>>::entry(&mut arw, 1);
        #[derive(UID)]
        pub enum V {}
        <Arrow as Entry<V>>::entry(&mut arw, 3);
        for i in 0..n_step {
            arw.read(Data::<U>::new(vec![i as f64]));
            arw.read(Data::<V>::new(vec![(10 * i) as f64; 3]));
        }
    }
}
