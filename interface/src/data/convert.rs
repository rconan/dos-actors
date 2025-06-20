use std::{marker::PhantomData, sync::Arc};

use crate::UniqueIdentifier;

use super::Data;

impl<T, U> From<Data<U>> for Vec<T>
where
    T: Clone,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn from(data: Data<U>) -> Self {
        (*data.0).clone()
    }
}
impl<T, U: UniqueIdentifier<DataType = Vec<T>>> From<&Data<U>> for Vec<T>
where
    T: Clone,
{
    fn from(data: &Data<U>) -> Self {
        data.to_vec()
    }
}
impl<'a, T, U: UniqueIdentifier<DataType = Vec<T>>> From<&'a Data<U>> for &'a [T] {
    fn from(data: &'a Data<U>) -> Self {
        data
    }
}
impl<T, U: UniqueIdentifier<DataType = Vec<T>>> From<Vec<T>> for Data<U> {
    fn from(u: Vec<T>) -> Self {
        Data(Arc::new(u), PhantomData)
    }
}
impl<T, U: UniqueIdentifier<DataType = [T; N]>, const N: usize> From<[T; N]> for Data<U> {
    fn from(u: [T; N]) -> Self {
        Data(Arc::new(u), PhantomData)
    }
}
impl<'a, T: Clone, U: UniqueIdentifier<DataType = Vec<T>>> From<&'a [T]> for Data<U> {
    fn from(u: &'a [T]) -> Self {
        Data(Arc::new(u.to_vec()), PhantomData)
    }
}
impl<T, U: UniqueIdentifier<DataType = T>> From<Arc<T>> for Data<U> {
    fn from(u: Arc<T>) -> Self {
        Data(u, PhantomData)
    }
}
impl<T, U: UniqueIdentifier<DataType = T>> From<&Arc<T>> for Data<U> {
    /// Makes a clone of the `Arc` pointer, returning `Data<U>` with the cloned [Arc] within
    fn from(u: &Arc<T>) -> Self {
        Data(Arc::clone(u), PhantomData)
    }
}
impl<T, U, V> From<&Data<V>> for Data<U>
where
    U: UniqueIdentifier<DataType = T>,
    V: UniqueIdentifier<DataType = T>,
{
    /// Makes a clone of `Data<V>` inner `Arc` pointer, returning `Data<U>` with the cloned [Arc] within
    fn from(data: &Data<V>) -> Self {
        Data(Arc::clone(&data.0), PhantomData)
    }
}
