pub mod convert;
mod ops;

use std::{
    fmt::{self, Debug},
    marker::PhantomData,
    ops::Deref,
    sync::Arc,
};

use super::{UniqueIdentifier, Who};

/// Actors I/O data wrapper
///
/// `U` is the data unique identifier (UID).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Data<U: UniqueIdentifier>(
    #[cfg_attr(
        feature = "serde",
        serde(bound(serialize = "<U as UniqueIdentifier>::DataType: serde::Serialize"))
    )]
    #[cfg_attr(
        feature = "serde",
        serde(bound(deserialize = "<U as UniqueIdentifier>::DataType: serde::Deserialize<'de>"))
    )]
    Arc<<U as UniqueIdentifier>::DataType>,
    PhantomData<U>,
);
impl<T, U: UniqueIdentifier<DataType = T>> Deref for Data<U> {
    type Target = T;
    /// Returns a reference to the data
    fn deref(&self) -> &Self::Target {
        &*self.0
    }
}

unsafe impl<T: Send, U: UniqueIdentifier<DataType = T>> Send for Data<U> {}
unsafe impl<T: Sync, U: UniqueIdentifier<DataType = T>> Sync for Data<U> {}

impl<T, U: UniqueIdentifier<DataType = T>> Clone for Data<U> {
    /// Makes a clone of the inner `Arc` pointer, returning a new instance of `Data<U>` with the cloned [Arc] within
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0), PhantomData)
    }
}

impl<T, U: UniqueIdentifier<DataType = T>> Data<U> {
    /// Moves `data` into an `Arc` pointer and places into `Data<U>`
    pub fn new(data: T) -> Self {
        Data(Arc::new(data), PhantomData)
    }
    /// Consumes `Data<U>`, returning `Data<V>` with the wrapped value within
    #[inline]
    pub fn transmute<V: UniqueIdentifier<DataType = T>>(self) -> Data<V> {
        Data(self.0, PhantomData)
    }
    /// Consumes `Data<U>`, returning the inner [Arc] pointer
    #[inline]
    pub fn into_arc(self) -> Arc<T> {
        self.0
    }
    /// Returns a clone of the inner [Arc] pointer
    #[inline]
    pub fn as_arc(&self) -> Arc<T> {
        Arc::clone(&self.0)
    }
}

impl<U: UniqueIdentifier> Who<U> for Data<U> {}
impl<T, U> fmt::Debug for Data<U>
where
    T: fmt::Debug,
    U: UniqueIdentifier<DataType = T>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Data").field(&self.0).field(&self.1).finish()
    }
}

impl<T, U> fmt::Display for Data<U>
where
    T: Debug,
    U: UniqueIdentifier<DataType = T>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.0)
    }
}

impl<T: Default, U: UniqueIdentifier<DataType = T>> Default for Data<U> {
    fn default() -> Self {
        Self(Default::default(), Default::default())
    }
}
impl<T, U> PartialEq for Data<U>
where
    T: PartialEq,
    U: UniqueIdentifier<DataType = T>,
{
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0 && self.1 == other.1
    }
}
use std::hash::Hash;
impl<T: Hash, U: UniqueIdentifier<DataType = T>> Hash for Data<U> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.0.hash(state);
        self.1.hash(state);
    }
}

// impl<T, U> Mul<T> for Data<U>
// where
//     T: Copy + Mul<T, Output = T>,
//     U: UniqueIdentifier<DataType = T>,
// {
//     type Output = Data<U>;
//
//     fn mul(self, rhs: T) -> Self::Output {
//         Data::new(rhs * *self)
//     }
// }


#[cfg(test)]
mod tests {
    use std::hash::Hasher;

    use super::*;

    /*
        [clients/src/interface/data.rs:152] hasher.finish() = 7081654805104250600
    [clients/src/interface/data.rs:155] hasher.finish() = 5099540947345535221
    [clients/src/interface/data.rs:162] hasher.finish() = 6858907488843767161
     */

    #[test]
    fn hash() {
        pub enum TestData {}
        impl UniqueIdentifier for TestData {
            type DataType = Vec<u32>;
        }
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        let data = Data::<TestData>::new(vec![1u32; 10]);
        data.hash(&mut hasher);
        dbg!(hasher.finish());
        let data = Data::<TestData>::new(vec![3u32; 10]);
        data.hash(&mut hasher);
        dbg!(hasher.finish());
        pub enum NewData {}
        impl UniqueIdentifier for NewData {
            type DataType = Vec<u64>;
        }
        let data = Data::<NewData>::new(vec![3u64; 10]);
        data.hash(&mut hasher);
        dbg!(hasher.finish());
    }
}
