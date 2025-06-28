use std::ops::{Add, Mul, Neg, Sub};

use crate::UniqueIdentifier;

use super::Data;

impl<T, U> Mul<T> for Data<U>
where
    T: Copy + Mul<T, Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn mul(self, rhs: T) -> Self::Output {
        Data::new(self.0.iter().map(|x| *x * rhs).collect())
    }
}

impl<T, U> Add<Data<U>> for Data<U>
where
    T: Copy + Add<T, Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn add(self, rhs: Data<U>) -> Self::Output {
        Data::new(
            self.0
                .iter()
                .zip(rhs.0.iter())
                .map(|(x, y)| *x + *y)
                .collect(),
        )
    }
}
impl<T, U> Add<Data<U>> for Vec<T>
where
    T: Copy + Add<T, Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn add(self, rhs: Data<U>) -> Self::Output {
        Data::new(
            self.iter()
                .zip(rhs.0.iter())
                .map(|(x, y)| *x + *y)
                .collect(),
        )
    }
}
impl<'a, T, U> Add<Data<U>> for &'a [T]
where
    T: Copy + Add<T, Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn add(self, rhs: Data<U>) -> Self::Output {
        Data::new(
            self.iter()
                .zip(rhs.0.iter())
                .map(|(x, y)| *x + *y)
                .collect(),
        )
    }
}

impl<'a, T, U, I> Add<I> for Data<U>
where
    T: Copy + Add<T, Output = T> + 'a,
    U: UniqueIdentifier<DataType = Vec<T>>,
    I: IntoIterator<Item = &'a T>,
{
    type Output = Data<U>;

    fn add(self, rhs: I) -> Self::Output {
        Data::new(
            self.0
                .iter()
                .zip(rhs.into_iter())
                .map(|(x, y)| *x + *y)
                .collect(),
        )
    }
}

impl<T, U> Sub<Data<U>> for Data<U>
where
    T: Copy + Sub<T, Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn sub(self, rhs: Data<U>) -> Self::Output {
        Data::new(
            self.0
                .iter()
                .zip(rhs.0.iter())
                .map(|(x, y)| *x - *y)
                .collect(),
        )
    }
}
impl<T, U> Sub<Data<U>> for Vec<T>
where
    T: Copy + Sub<T, Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn sub(self, rhs: Data<U>) -> Self::Output {
        Data::new(
            self.iter()
                .zip(rhs.0.iter())
                .map(|(x, y)| *x - *y)
                .collect(),
        )
    }
}
impl<'a, T, U> Sub<Data<U>> for &'a [T]
where
    T: Copy + Sub<T, Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn sub(self, rhs: Data<U>) -> Self::Output {
        Data::new(
            self.iter()
                .zip(rhs.0.iter())
                .map(|(x, y)| *x - *y)
                .collect(),
        )
    }
}
impl<'a, T, U, I> Sub<I> for Data<U>
where
    T: Copy + Sub<T, Output = T> + 'a,
    U: UniqueIdentifier<DataType = Vec<T>>,
    I: IntoIterator<Item = &'a T>,
{
    type Output = Data<U>;

    fn sub(self, rhs: I) -> Self::Output {
        Data::new(
            self.0
                .iter()
                .zip(rhs.into_iter())
                .map(|(x, y)| *x - *y)
                .collect(),
        )
    }
}

impl<T, U> Neg for Data<U>
where
    T: Copy + Neg<Output = T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    type Output = Data<U>;

    fn neg(self) -> Self::Output {
        Data::new(self.0.iter().map(|x| -*x).collect())
    }
}
