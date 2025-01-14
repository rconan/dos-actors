/*!
# A client that splits vectors in 2 parts

[LeftRight] is a client that splits a vector in 2 parts: `data = [left,right]`

## Examples

Splitting the vector `vec![1,2,3,4,5]` into `vec![1,2]` and `vec![3,4,5]`
and reassembling it

```
use gmt_dos_clients::leftright::{self, LeftRight, Split, Merge, Left, Right};
use interface::{UniqueIdentifier, Data, Read, Write};

enum S {}
impl UniqueIdentifier for S {
    type DataType = Vec<usize>;
}
enum M {}
impl UniqueIdentifier for M {
    type DataType = Vec<usize>;
}
let data: Vec<_> = (1..=5).into_iter().collect();
let (mut split, mut merge) = leftright::split_merge_at::<S, M>(2);

// SPLITTING
<LeftRight<S, Split, M> as Read<S>>::read(&mut split, data.clone().into());
let left = <LeftRight<S, Split, M> as Write<Left<S>>>::write(&mut split).unwrap();
assert_eq!(*left,vec![1,2]);
let right = <LeftRight<S, Split, M> as Write<Right<S>>>::write(&mut split).unwrap();
assert_eq!(*right,vec![3,4,5]);

// MERGING
<LeftRight<S, Merge, M> as Read<Left<S>>>::read(&mut merge, left);
<LeftRight<S, Merge, M> as Read<Right<S>>>::read(&mut merge, right);
let merged_data = <LeftRight<S, Merge, M> as Write<M>>::write(&mut merge).unwrap();
assert_eq!(*merged_data,data);
```

Splitting the vector `vec![1,2,3,1,2,3,1,2,3,1,2,3]` into `vec![1,1,1,1]` and `vec![2,3,2,3,2,3,2,3]`
and reassembling it

```
use gmt_dos_clients::leftright::{self, LeftRight, Split, Merge, Left, Right};
use interface::{UniqueIdentifier, Data, Read, Write};

enum S {}
impl UniqueIdentifier for S {
    type DataType = Vec<usize>;
}
enum M {}
impl UniqueIdentifier for M {
    type DataType = Vec<usize>;
}
let data: Vec<_> = (1..=3).into_iter().collect::<Vec<_>>().repeat(4);
let (mut split, mut merge) = leftright::split_merge_chunks_at::<S, M>(3, 1);

// SPLITTING
<LeftRight<S, Split, M> as Read<S>>::read(&mut split, data.clone().into());
let left = <LeftRight<S, Split, M> as Write<Left<S>>>::write(&mut split).unwrap();
assert_eq!(*left,vec![1,1,1,1]);
let right = <LeftRight<S, Split, M> as Write<Right<S>>>::write(&mut split).unwrap();
assert_eq!(*right,vec![2,3,2,3,2,3,2,3]);

// MERGING
<LeftRight<S, Merge, M> as Read<Left<S>>>::read(&mut merge, left);
<LeftRight<S, Merge, M> as Read<Right<S>>>::read(&mut merge, right);
let merged_data = <LeftRight<S, Merge, M> as Write<M>>::write(&mut merge).unwrap();
assert_eq!(*merged_data,data);
```
*/

use std::{marker::PhantomData, ops::Deref, sync::Arc};

use interface::{Data, Read, UniqueIdentifier, Update, Write};

/// Splitting state for [LeftRight] client
pub enum Split {}
/// Merging state for [LeftRight] client
pub enum Merge {}

/// LeftRight client
pub struct LeftRight<U, S, V = U>
where
    U: UniqueIdentifier,
    V: UniqueIdentifier,
{
    i: usize,
    n: Option<usize>,
    data: Arc<U::DataType>,
    left: Arc<U::DataType>,
    right: Arc<U::DataType>,
    state: PhantomData<S>,
    u: PhantomData<U>,
    v: PhantomData<V>,
}

impl<T, U, V> LeftRight<U, Split, V>
where
    T: Copy,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    /// Creates a [LeftRight] client
    ///
    /// The data is split at index `i`
    /// meaning that the left part will be the first `i` elements
    pub fn split_at(i: usize) -> Self {
        Self {
            i,
            n: Default::default(),
            data: Default::default(),
            left: Default::default(),
            right: Default::default(),
            state: PhantomData,
            u: PhantomData,
            v: PhantomData,
        }
    }
    /// Creates a [LeftRight] client
    ///
    /// The data is first split in chunks of size `n`,
    /// then each chunk is split at index `i`
    /// meaning that the left part will be the first `i` elements
    /// of each chunk
    pub fn split_chunks_at(n: usize, i: usize) -> Self {
        Self {
            i,
            n: Some(n),
            data: Default::default(),
            left: Default::default(),
            right: Default::default(),
            state: PhantomData,
            u: PhantomData,
            v: PhantomData,
        }
    }
}

/// Creates a merging [LeftRight] client from a splitting [LeftRight] client
impl<T, U, V> From<&LeftRight<U, Split>> for LeftRight<U, Merge, V>
where
    T: Copy,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    fn from(split: &LeftRight<U, Split>) -> Self {
        Self {
            i: split.i,
            n: split.n,
            data: Default::default(),
            left: Default::default(),
            right: Default::default(),
            state: PhantomData,
            u: PhantomData,
            v: PhantomData,
        }
    }
}

/// Creates both a splittng and a merging [LeftRight] client
///
/// The data is split at index `i`
/// meaning that the left part will be the first `i` elements
pub fn split_merge_at<U, V>(i: usize) -> (LeftRight<U, Split, V>, LeftRight<U, Merge, V>)
where
    U: UniqueIdentifier,
    V: UniqueIdentifier,
    <U as UniqueIdentifier>::DataType: Default,
    <V as UniqueIdentifier>::DataType: Default,
{
    (
        LeftRight::<U, Split, V> {
            i,
            n: Default::default(),
            data: Default::default(),
            left: Default::default(),
            right: Default::default(),
            state: PhantomData,
            u: PhantomData,
            v: PhantomData,
        },
        LeftRight::<U, Merge, V> {
            i,
            n: Default::default(),
            data: Default::default(),
            left: Default::default(),
            right: Default::default(),
            state: PhantomData,
            u: PhantomData,
            v: PhantomData,
        },
    )
}

/// Creates both a splittng and a merging [LeftRight] client
///
/// The data is first split in chunks of size `n`,
/// then each chunk is split at index `i`
/// meaning that the left part will be the first `i` elements
/// of each chunk
pub fn split_merge_chunks_at<U, V>(
    n: usize,
    i: usize,
) -> (LeftRight<U, Split, V>, LeftRight<U, Merge, V>)
where
    U: UniqueIdentifier,
    V: UniqueIdentifier,
    <U as UniqueIdentifier>::DataType: Default,
    <V as UniqueIdentifier>::DataType: Default,
{
    (
        LeftRight::<U, Split, V> {
            i,
            n: Some(n),
            data: Default::default(),
            left: Default::default(),
            right: Default::default(),
            state: PhantomData,
            u: PhantomData,
            v: PhantomData,
        },
        LeftRight::<U, Merge, V> {
            i,
            n: Some(n),
            data: Default::default(),
            left: Default::default(),
            right: Default::default(),
            state: PhantomData,
            u: PhantomData,
            v: PhantomData,
        },
    )
}

impl<T, U, S, V> Update for LeftRight<U, S, V>
where
    T: Copy + Send + Sync,
    S: Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
}

impl<T, U, V> Read<U> for LeftRight<U, Split, V>
where
    T: Copy + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        self.data = data.as_arc();
    }
}

/// Identifier for the left part of the splitted data
pub struct Left<U: UniqueIdentifier>(PhantomData<U>);
impl<U: UniqueIdentifier> UniqueIdentifier for Left<U> {
    type DataType = U::DataType;
}

/// Identifier for the right part of the splitted data
pub struct Right<U: UniqueIdentifier>(PhantomData<U>);
impl<U: UniqueIdentifier> UniqueIdentifier for Right<U> {
    type DataType = U::DataType;
}

impl<T, U, V> Write<Left<U>> for LeftRight<U, Split, V>
where
    T: Copy + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<Left<U>>> {
        Some(
            if let Some(n) = self.n {
                self.data
                    .chunks(n)
                    .flat_map(|data| data[..self.i].to_vec())
                    .collect::<Vec<_>>()
            } else {
                self.data[..self.i].to_vec()
            }
            .into(),
        )
    }
}

impl<T, U, V> Write<Right<U>> for LeftRight<U, Split, V>
where
    T: Copy + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<Right<U>>> {
        Some(
            if let Some(n) = self.n {
                self.data
                    .chunks(n)
                    .flat_map(|data| data[self.i..].to_vec())
                    .collect::<Vec<_>>()
            } else {
                self.data[self.i..].to_vec()
            }
            .into(),
        )
    }
}

impl<T, U, V> Read<Left<U>> for LeftRight<U, Merge, V>
where
    T: Copy + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<Left<U>>) {
        self.left = data.as_arc();
    }
}

impl<T, U, V> Read<Right<U>> for LeftRight<U, Merge, V>
where
    T: Copy + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<Right<U>>) {
        self.right = data.as_arc();
    }
}

impl<T, U, V> Write<V> for LeftRight<U, Merge, V>
where
    T: Copy + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
    V: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<V>> {
        Some(
            if let Some(n) = self.n {
                self.left
                    .chunks(self.i)
                    .zip(self.right.chunks(n - self.i))
                    .flat_map(|(left, right)| {
                        let mut data = left.to_vec();
                        data.extend_from_slice(right);
                        data
                    })
                    .collect::<Vec<T>>()
            } else {
                let mut data = self.left.deref().to_vec();
                data.extend_from_slice(&self.right);
                data
            }
            .into(),
        )
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crate::{print::Print, Signals};
    use interface::UID;

    use super::*;

    enum U {}
    impl UniqueIdentifier for U {
        type DataType = Vec<usize>;
    }
    #[derive(UID)]
    pub enum S {}
    #[derive(UID)]
    pub enum M {}

    #[test]
    fn split_at() {
        let data: Vec<_> = (1..=5).into_iter().collect();
        let mut split = LeftRight::<U, Split>::split_at(2);
        <LeftRight<U, Split> as Read<U>>::read(&mut split, data.into());
        let left = <LeftRight<U, Split> as Write<Left<U>>>::write(&mut split);
        dbg!(&left);
        let right = <LeftRight<U, Split> as Write<Right<U>>>::write(&mut split);
        dbg!(&right);

        let mut merge: LeftRight<U, Merge> = (&split).into();
        <LeftRight<U, Merge> as Read<Left<U>>>::read(&mut merge, left.unwrap());
        <LeftRight<U, Merge> as Read<Right<U>>>::read(&mut merge, right.unwrap());
        let merged_data = <LeftRight<U, Merge> as Write<U>>::write(&mut merge);
        dbg!(merged_data);
    }

    #[test]
    fn split_chunks_at() {
        let data: Vec<_> = (1..=5).into_iter().collect::<Vec<_>>().repeat(3);
        let mut split = LeftRight::<U, Split>::split_chunks_at(5, 2);
        <LeftRight<U, Split> as Read<U>>::read(&mut split, data.into());
        let left = <LeftRight<U, Split> as Write<Left<U>>>::write(&mut split);
        dbg!(&left);
        let right = <LeftRight<U, Split> as Write<Right<U>>>::write(&mut split);
        dbg!(&right);

        let mut merge: LeftRight<U, Merge> = (&split).into();
        <LeftRight<U, Merge> as Read<Left<U>>>::read(&mut merge, left.unwrap());
        <LeftRight<U, Merge> as Read<Right<U>>>::read(&mut merge, right.unwrap());
        let merged_data = <LeftRight<U, Merge> as Write<U>>::write(&mut merge);
        dbg!(merged_data);
    }

    /* #[tokio::test]
    async fn model() -> Result<(), Box<dyn Error>> {
        use gmt_dos_actors::prelude::*;

        let mut signal: Initiator<_> = Signals::new(10, 10).into();
        let (split, merge) = split_merge_at::<S, M>(5);
        let mut split_actor: Actor<_> = split.into();
        let mut left: Terminator<_> = Print::<Vec<f64>>::default().into();
        let mut right: Terminator<_> = Print::<Vec<f64>>::default().into();

        let _ = signal
            .add_output()
            .build::<S>()
            .into_input(&mut split_actor)?;
        let _ = split_actor
            .add_output()
            .build::<Left<S>>()
            .into_input(&mut left)?;
        let _ = split_actor
            .add_output()
            .build::<Right<S>>()
            .into_input(&mut right)?;

        model!(signal, split_actor, left, right)
            .check()?
            .flowchart();

        Ok(())
    } */
}
