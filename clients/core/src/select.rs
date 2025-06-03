/*!
# Select vector element

Select and return some elements for an input [Vec]

## Examples

Selecting the 3rd element
```
use gmt_dos_clients::select::Select;
let select = Select::<f64>::new(2);
```

Selecting the 1st and 3rd elements
```
use gmt_dos_clients::select::Select;
let select = Select::<f64>::new(vec![0,2]);
```

Selecting elements from the 2nd to the 4th
```
use gmt_dos_clients::select::Select;
let select = Select::<f64>::new(1..3);

*/

use std::{marker::PhantomData, ops::Range, sync::Arc};

use interface::{
    Data, Read, UniqueIdentifier, Update, Write,
    units::{Arcsec, Mas, MuM, NM, UnitsConversion},
};

pub enum Selection {
    Index(usize),
    Range(Range<usize>),
    Indices(Vec<usize>),
}

impl From<usize> for Selection {
    fn from(value: usize) -> Self {
        Self::Index(value)
    }
}

impl From<Range<usize>> for Selection {
    fn from(value: Range<usize>) -> Self {
        Self::Range(value)
    }
}

impl From<Vec<usize>> for Selection {
    fn from(value: Vec<usize>) -> Self {
        Self::Indices(value)
    }
}
pub struct Select<T> {
    selection: Selection,
    data: Arc<Vec<T>>,
    inner: PhantomData<T>,
}

impl<T> Select<T> {
    pub fn new(select: impl Into<Selection>) -> Self {
        Self {
            selection: select.into(),
            data: Arc::new(Vec::new()),
            inner: PhantomData,
        }
    }
}

impl<T: Send + Sync> Update for Select<T> {}

impl<T, U> Read<U> for Select<T>
where
    U: UniqueIdentifier<DataType = Vec<T>>,
    T: Send + Sync,
{
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

impl<T, U> Write<U> for Select<T>
where
    U: UniqueIdentifier<DataType = Vec<T>>,
    T: Clone + Send + Sync,
{
    fn write(&mut self) -> Option<Data<U>> {
        match &self.selection {
            Selection::Index(idx) => self.data.get(*idx).map(|data| vec![data.clone()].into()),
            Selection::Range(range) => range
                .clone()
                .map(|idx| self.data.get(idx).cloned())
                .collect::<Option<Vec<T>>>()
                .map(|data| data.into()),
            Selection::Indices(idxs) => idxs
                .iter()
                .map(|idx| self.data.get(*idx).cloned())
                .collect::<Option<Vec<T>>>()
                .map(|data| data.into()),
        }
    }
}

pub struct USelect<S>(Select<f64>, PhantomData<S>);

impl<S> USelect<S> {
    pub fn new(select: impl Into<Selection>) -> Self {
        Self(Select::new(select), PhantomData)
    }
}

impl<S: Send + Sync> Update for USelect<S> {}

impl<U, S: Send + Sync> Read<U> for USelect<S>
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn read(&mut self, data: Data<U>) {
        <Select<f64> as Read<U>>::read(&mut self.0, data);
    }
}

macro_rules! impl_uselect {
    ( $( $t:ident ),* ) => {
        $(
            impl<U> Write<U> for USelect<$t<U>>
            where
                U: UniqueIdentifier<DataType = Vec<f64>>,
            {
                fn write(&mut self) -> Option<Data<U>> {
                    <Select<f64> as Write<U>>::write(&mut self.0)
                        .as_ref()
                        .map(|data| {
                            <$t<U> as UnitsConversion>::conversion(data)
                                .unwrap()
                                .transmute()
                        })
                }
            }
    )*
    };
}
impl_uselect! {NM, MuM, Arcsec, Mas}
