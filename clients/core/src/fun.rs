use std::{marker::PhantomData, sync::Arc};

use interface::{Data, Read, UniqueIdentifier, Update, Write};

pub struct Fun<X, Y, F: Fn(&X) -> Y> {
    x: PhantomData<X>,
    y: Arc<Y>,
    f: F,
}

impl<X, Y, F> Fun<X, Y, F>
where
    Y: Default,
    F: Fn(&X) -> Y,
{
    pub fn new(f: F) -> Self {
        Fun {
            x: PhantomData,
            y: Default::default(),
            f,
        }
    }

    // pub fn call(&self) -> Y {
    //     (self.f)(self.u)
    // }
}

impl<X, Y, F> Update for Fun<X, Y, F>
where
    X: Send + Sync,
    Y: Send + Sync,
    F: Send + Sync + Fn(&X) -> Y,
{
}

impl<X, Y, F, U> Read<U> for Fun<X, Y, F>
where
    X: Send + Sync,
    Y: Send + Sync,
    F: Send + Sync + Fn(&X) -> Y,
    U: UniqueIdentifier<DataType = X>,
{
    fn read(&mut self, data: Data<U>) {
        self.y = Arc::new((self.f)(&data.as_arc()));
    }
}

impl<X, Y, F, U> Write<U> for Fun<X, Y, F>
where
    X: Send + Sync,
    Y: Send + Sync,
    F: Send + Sync + Fn(&X) -> Y,
    U: UniqueIdentifier<DataType = Y>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.y.clone().into())
    }
}
