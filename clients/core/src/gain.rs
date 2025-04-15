use super::{Data, Read, UniqueIdentifier, Update, Write};
#[cfg(all(feature = "faer", feature = "nalgebra"))]
use faer_ext::IntoFaer;
#[cfg(all(feature = "faer", feature = "nalgebra"))]
use nalgebra as na;
use num_traits::{One, Zero};
use std::fmt::Debug;
use std::ops::{AddAssign, Mul, MulAssign};
use std::sync::Arc;

pub enum GainKind<T> {
    #[cfg(feature = "faer")]
    Matrix(faer::Mat<T>),
    Vector(Vec<T>),
    #[cfg(feature = "faer")]
    SplitMatrix(Vec<faer::Mat<T>>),
}
#[cfg(feature = "faer")]
impl<T> Mul<&[T]> for &GainKind<T>
where
    T: Zero
        + Clone
        + Copy
        + PartialEq
        + Debug
        + One
        + AddAssign
        + Mul
        + MulAssign
        + faer_traits::RealField,
{
    type Output = Vec<T>;
    fn mul(self, rhs: &[T]) -> Self::Output {
        match self {
            #[cfg(feature = "faer")]
            GainKind::Matrix(mat) => (mat
                * faer::MatRef::from_column_major_slice(&rhs, rhs.len(), 1))
            .col_as_slice(0)
            .to_vec(),
            GainKind::Vector(val) => rhs.into_iter().zip(val).map(|(&u, &v)| v * u).collect(),
            #[cfg(feature = "faer")]
            GainKind::SplitMatrix(mats) => {
                let mut a = 0;
                mats.iter()
                    .flat_map(|mat| {
                        let n = mat.ncols();
                        let x = faer::MatRef::from_column_major_slice(&rhs[a..a + n], n, 1);
                        let y = mat * x;
                        a += n;
                        y.col_as_slice(0).to_vec()
                    })
                    .collect()
            }
        }
    }
}
#[cfg(not(feature = "faer"))]
impl<T> Mul<&[T]> for &GainKind<T>
where
    T: Zero + Clone + Copy + PartialEq + Debug + One + AddAssign + Mul + MulAssign,
{
    type Output = Vec<T>;
    fn mul(self, rhs: &[T]) -> Self::Output {
        match self {
            GainKind::Vector(val) => rhs.into_iter().zip(val).map(|(&u, &v)| v * u).collect(),
        }
    }
}
#[cfg(all(feature = "faer", feature = "nalgebra"))]
impl<T: Clone> From<na::DMatrix<T>> for GainKind<T> {
    fn from(value: na::DMatrix<T>) -> Self {
        Self::Matrix(value.view_range(.., ..).into_faer().cloned())
    }
}
#[cfg(all(feature = "faer", feature = "nalgebra"))]
impl<T: Clone> From<Vec<na::DMatrix<T>>> for GainKind<T> {
    fn from(value: Vec<na::DMatrix<T>>) -> Self {
        Self::SplitMatrix(
            value
                .into_iter()
                .map(|value| value.view_range(.., ..).into_faer().cloned())
                .collect(),
        )
    }
}
impl<T> From<Vec<T>> for GainKind<T> {
    fn from(value: Vec<T>) -> Self {
        Self::Vector(value)
    }
}
#[cfg(feature = "faer")]
impl<T> From<Vec<faer::Mat<T>>> for GainKind<T> {
    fn from(value: Vec<faer::Mat<T>>) -> Self {
        Self::SplitMatrix(value)
    }
}
#[cfg(feature = "faer")]
impl<'a, T: Clone> From<&'a [faer::MatRef<'a, T>]> for GainKind<T> {
    fn from(value: &'a [faer::MatRef<'a, T>]) -> Self {
        Self::SplitMatrix(value.into_iter().map(|mat| mat.cloned()).collect())
    }
}
impl<T> GainKind<T> {
    pub fn ncols(&self) -> usize {
        match self {
            #[cfg(feature = "faer")]
            GainKind::Matrix(mat) => mat.ncols(),
            GainKind::Vector(val) => val.len(),
            #[cfg(feature = "faer")]
            GainKind::SplitMatrix(mats) => mats.iter().map(|mat| mat.ncols()).sum(),
        }
    }
    pub fn nrows(&self) -> usize {
        match self {
            #[cfg(feature = "faer")]
            GainKind::Matrix(mat) => mat.nrows(),
            GainKind::Vector(val) => val.len(),
            #[cfg(feature = "faer")]
            GainKind::SplitMatrix(mats) => mats.iter().map(|mat| mat.nrows()).sum(),
        }
    }
}

/// Gain client
///
/// Applies the gain to the input signal either
/// as a matrix multiplication or
/// as an element wise vector multiplication
pub struct Gain<T> {
    u: Arc<Vec<T>>,
    y: Arc<Vec<T>>,
    gain: GainKind<T>,
}
impl<T> Gain<T>
where
    T: Zero + Clone + Copy + PartialEq + Debug + 'static,
{
    /// Creates a new [Gain] clients
    ///
    /// The gain is either a matrix of dimensions `Ny`x`Nu`,
    /// a vector of size `Ny`=`Nu` or a vector or matrices such
    /// as the sum of the number of columns is equal to `Nu` and
    /// the sum of the number of rows is equal to `Ny`
    pub fn new<G: Into<GainKind<T>>>(gain: G) -> Self {
        let gain: GainKind<T> = gain.into();
        Self {
            u: Arc::new(vec![T::zero(); gain.ncols()]),
            y: Arc::new(vec![T::zero(); gain.nrows()]),
            gain,
        }
    }
}
#[cfg(feature = "faer")]
impl<T> Update for Gain<T>
where
    T: Zero
        + Clone
        + Copy
        + PartialEq
        + Debug
        + One
        + AddAssign
        + Mul
        + MulAssign
        + Send
        + Sync
        + faer_traits::RealField,
{
    fn update(&mut self) {
        self.y = Arc::new(&self.gain * self.u.as_slice());
    }
}
#[cfg(not(feature = "faer"))]
impl<T> Update for Gain<T>
where
    T: Zero + Clone + Copy + PartialEq + Debug + One + AddAssign + Mul + MulAssign + Send + Sync,
{
    fn update(&mut self) {
        self.y = Arc::new(&self.gain * self.u.as_slice());
    }
}
#[cfg(feature = "faer")]
impl<T, U> Read<U> for Gain<T>
where
    T: Zero
        + Clone
        + Copy
        + PartialEq
        + Debug
        + One
        + AddAssign
        + Mul
        + MulAssign
        + Send
        + Sync
        + faer_traits::RealField,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        assert_eq!(
            self.u.len(),
            data.len(),
            "data size ({}) do not match expected input size ({})",
            data.len(),
            self.u.len()
        );
        self.u = data.into_arc();
    }
}
#[cfg(not(feature = "faer"))]
impl<T, U> Read<U> for Gain<T>
where
    T: Zero + Clone + Copy + PartialEq + Debug + One + AddAssign + Mul + MulAssign + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        assert_eq!(
            self.u.len(),
            data.len(),
            "data size ({}) do not match expected input size ({})",
            data.len(),
            self.u.len()
        );
        self.u = data.into_arc();
    }
}
#[cfg(feature = "faer")]
impl<T, U> Write<U> for Gain<T>
where
    T: Zero
        + Clone
        + Copy
        + PartialEq
        + Debug
        + One
        + AddAssign
        + Mul
        + MulAssign
        + Send
        + Sync
        + faer_traits::RealField,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some((&self.y).into())
    }
}
#[cfg(not(feature = "faer"))]
impl<T, U> Write<U> for Gain<T>
where
    T: Zero + Clone + Copy + PartialEq + Debug + One + AddAssign + Mul + MulAssign + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some((&self.y).into())
    }
}
