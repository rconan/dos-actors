use std::{marker::PhantomData, ops::Deref, sync::Arc};

use crate::{Data, Read, UniqueIdentifier, Update, Write};

pub trait UidTuple {}

/// A unique identifier for a pair of unique identifiers
///
/// [Doublet] can be used with any client that implements the trait [UidTuple]
/// and the traits [Read] and/or [Write] for the pair of UIDs
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Doublet<U1, U2, const ID: usize = 2>(PhantomData<U1>, PhantomData<U2>)
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier;
type UID2<U1, U2> = (
    Arc<<U1 as UniqueIdentifier>::DataType>,
    Arc<<U2 as UniqueIdentifier>::DataType>,
);

// UID
impl<U1: UniqueIdentifier, U2: UniqueIdentifier> UidTuple for Doublet<U1, U2> {}
impl<U1, U2> UniqueIdentifier for Doublet<U1, U2, 0>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
    type DataType = <U1 as UniqueIdentifier>::DataType;
    const PORT: u16 = <U1 as UniqueIdentifier>::PORT;
}
impl<U1, U2> UniqueIdentifier for Doublet<U1, U2, 1>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
    type DataType = <U2 as UniqueIdentifier>::DataType;
    const PORT: u16 = <U2 as UniqueIdentifier>::PORT;
}
impl<U1, U2> UniqueIdentifier for Doublet<U1, U2>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
    type DataType = UID2<U1, U2>;
    const PORT: u16 = (U1::PORT - 50_000) + (U2::PORT - 50_000) + 20_000;
}

// WRITE
impl<C, U1, U2> Write<Doublet<U1, U2, 0>> for C
where
    C: Update + Write<U1> + Write<U2> + UidTuple,
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
    Doublet<U1, U2>: UniqueIdentifier<DataType = <U1 as UniqueIdentifier>::DataType>,
{
    fn write(&mut self) -> Option<Data<Doublet<U1, U2, 0>>> {
        <_ as Write<U1>>::write(self).map(|x| x.transmute())
    }
}
impl<C, U1, U2> Write<Doublet<U1, U2, 1>> for C
where
    C: Update + Write<U1> + Write<U2> + UidTuple,
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
    Doublet<U1, U2>: UniqueIdentifier<DataType = <U2 as UniqueIdentifier>::DataType>,
{
    fn write(&mut self) -> Option<Data<Doublet<U1, U2, 1>>> {
        <_ as Write<U2>>::write(self).map(|x| x.transmute())
    }
}
impl<C, U1, U2> Write<Doublet<U1, U2>> for C
where
    C: Update + Write<U1> + Write<U2> + UidTuple,
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
    Doublet<U1, U2>: UniqueIdentifier<DataType = UID2<U1, U2>>,
{
    fn write(&mut self) -> Option<Data<Doublet<U1, U2>>> {
        let data1 = <_ as Write<U1>>::write(self);
        let data2 = <_ as Write<U2>>::write(self);

        data1
            .map(|data| data.into_arc())
            .zip(data2.map(|data| data.into_arc()))
            .map(|data| Data::new(data))
        // Some(Data::new(data))
    }
}

// READ
impl<C, U1, U2> Read<Doublet<U1, U2, 0>> for C
where
    C: Update + Read<U1> + Read<U2> + UidTuple,
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
    Doublet<U1, U2>: UniqueIdentifier<DataType = <U1 as UniqueIdentifier>::DataType>,
{
    fn read(&mut self, data: Data<Doublet<U1, U2, 0>>) {
        <_ as Read<U1>>::read(self, data.transmute());
    }
}
impl<C, U1, U2> Read<Doublet<U1, U2, 1>> for C
where
    C: Update + Read<U1> + Read<U2> + UidTuple,
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
    Doublet<U1, U2>: UniqueIdentifier<DataType = <U2 as UniqueIdentifier>::DataType>,
{
    fn read(&mut self, data: Data<Doublet<U1, U2, 1>>) {
        <_ as Read<U2>>::read(self, data.transmute());
    }
}
impl<C, U1, U2> Read<Doublet<U1, U2>> for C
where
    C: Update + Read<U1> + Read<U2> + UidTuple,
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
    Doublet<U1, U2>: UniqueIdentifier<DataType = UID2<U1, U2>>,
{
    fn read(&mut self, data: Data<Doublet<U1, U2>>) {
        let (data1, data2) = data.into_arc().deref().clone();
        <_ as Read<U1>>::read(self, data1.into());
        <_ as Read<U2>>::read(self, data2.into());
    }
}

/// Client to get [Doublet] individual UIDs
///
/// [ID] is the index in the UID list: `[U1, U2]`,
/// the index of the UID that corresponds to [ID] get extracted   
pub struct Get<U1, U2, const ID: usize>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
    data: Arc<UID2<U1, U2>>,
}

// DEFAULT
impl<U1, U2, const ID: usize> Default for Get<U1, U2, ID>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
    <U1 as UniqueIdentifier>::DataType: Default,
    <U2 as UniqueIdentifier>::DataType: Default,
{
    fn default() -> Self {
        Self {
            data: Default::default(),
        }
    }
}

// UPDATE
impl<U1, U2, const ID: usize> Update for Get<U1, U2, ID>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
}

// READ
impl<U1, U2, const ID: usize> Read<Doublet<U1, U2>> for Get<U1, U2, ID>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
    fn read(&mut self, data: Data<Doublet<U1, U2>>) {
        self.data = data.into_arc();
    }
}

// WRITE
impl<U1, U2> Write<U1> for Get<U1, U2, 0>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
    fn write(&mut self) -> Option<Data<U1>> {
        let (data1, _data2) = self.data.deref().clone();
        Some(data1.into())
    }
}
impl<U1, U2> Write<U2> for Get<U1, U2, 1>
where
    U1: UniqueIdentifier,
    U2: UniqueIdentifier,
{
    fn write(&mut self) -> Option<Data<U2>> {
        let (_data1, data2) = self.data.deref().clone();
        Some(data2.into())
    }
}
