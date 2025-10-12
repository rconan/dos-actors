use std::collections::VecDeque;

use apache_arrow::{
    array::{ListArray, PrimitiveArray},
    datatypes::ArrowPrimitiveType,
};

use crate::{Arrow, ArrowError, BufferDataType, DropOption, Result};

pub struct ArrowIter<T>(VecDeque<Vec<T>>)
where
    T: BufferDataType,
    <T as BufferDataType>::ArrayType: ArrowPrimitiveType,
    Vec<T>: FromIterator<<<T as BufferDataType>::ArrayType as ArrowPrimitiveType>::Native>;

impl<T> Iterator for ArrowIter<T>
where
    T: BufferDataType,
    <T as BufferDataType>::ArrayType: ArrowPrimitiveType,
    Vec<T>: FromIterator<<<T as BufferDataType>::ArrayType as ArrowPrimitiveType>::Native>,
{
    type Item = Vec<T>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.pop_front()
    }
}
impl<T> DoubleEndedIterator for ArrowIter<T>
where
    T: BufferDataType,
    <T as BufferDataType>::ArrayType: ArrowPrimitiveType,
    Vec<T>: FromIterator<<<T as BufferDataType>::ArrayType as ArrowPrimitiveType>::Native>,
{
    fn next_back(&mut self) -> Option<Self::Item> {
        self.0.pop_back()
    }
}
impl<T> ExactSizeIterator for ArrowIter<T>
where
    T: BufferDataType,
    <T as BufferDataType>::ArrayType: ArrowPrimitiveType,
    Vec<T>: FromIterator<<<T as BufferDataType>::ArrayType as ArrowPrimitiveType>::Native>,
{
    fn len(&self) -> usize {
        self.0.len()
    }
}

impl Arrow {
    /// Returns an iterator over the data in the specified field
    pub fn iter<S, T>(&mut self, field_name: S) -> Result<ArrowIter<T>>
    where
        S: AsRef<str>,
        String: From<S>,
        T: BufferDataType,
        <T as BufferDataType>::ArrayType: ArrowPrimitiveType,
        Vec<T>: FromIterator<<<T as BufferDataType>::ArrayType as ArrowPrimitiveType>::Native>,
    {
        match self.record() {
            Ok(record) => match record.schema().column_with_name(field_name.as_ref()) {
                Some((idx, _)) => record
                    .column(idx)
                    .as_any()
                    .downcast_ref::<ListArray>()
                    .map(|data| {
                        data.iter()
                            .map(|data| {
                                data.map(|data| {
                                    data.as_any()
                                        .downcast_ref::<PrimitiveArray<<T as BufferDataType>::ArrayType>>()
                                        .and_then(|data| data.iter().collect::<Option<Vec<T>>>())
                                })
                                .flatten()
                            })
                            .collect::<Option<VecDeque<Vec<T>>>>()
                    })
                    .flatten()
                    .ok_or_else(|| ArrowError::ParseField(field_name.into())),
                None => Err(ArrowError::FieldNotFound(field_name.into())),
            },
            Err(e) => Err(e),
        }.map(|data| ArrowIter(data))
    }
    /// Returns an iterator over the data in the specified field consuming [Self]
    ///
    /// [Self] won't be written to a file regarless of the original settings
    pub fn into_iter<S, T>(mut self, field_name: S) -> Result<ArrowIter<T>>
    where
        S: AsRef<str>,
        String: From<S>,
        T: BufferDataType,
        <T as BufferDataType>::ArrayType: ArrowPrimitiveType,
        Vec<T>: FromIterator<<<T as BufferDataType>::ArrayType as ArrowPrimitiveType>::Native>,
    {
        self.drop_option = DropOption::NoSave;
        self.iter(field_name)
    }
}
