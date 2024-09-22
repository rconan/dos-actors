use faer::{Mat, MatRef};
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use serde::{Deserialize, Serialize};
use std::{
    fmt::{Display, Formatter},
    ops::{Div, Mul, SubAssign},
    sync::Arc,
};

use super::{Calib, CalibPinv};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Reconstructor {
    calib: Vec<Calib>,
    pinv: Vec<Option<CalibPinv<f64>>>,
    data: Arc<Vec<f64>>,
    estimate: Arc<Vec<f64>>,
}

impl From<Calib> for Reconstructor {
    fn from(calib: Calib) -> Self {
        Self::new(vec![calib])
    }
}

impl Reconstructor {
    pub fn new(calib: Vec<Calib>) -> Self {
        Self {
            pinv: vec![None; calib.len()],
            calib,
            ..Default::default()
        }
    }
    pub fn pseudoinverse(&mut self) -> &mut Self {
        self.pinv = self.calib.iter().map(|c| Some(c.pseudoinverse())).collect();
        self
    }
    pub fn area(&self) -> usize {
        self.calib.iter().map(|c| c.area()).sum()
    }
    pub fn match_areas(&mut self, other: &mut Self) {
        self.calib
            .iter_mut()
            .zip(&mut other.calib)
            .for_each(|(c, oc)| c.match_areas(oc));
    }
    pub fn least_square_solve(&mut self, b: &Reconstructor) -> Vec<Mat<f64>> {
        self.pinv()
            .zip(&b.calib)
            .map(|(pinv, calib)| pinv * calib)
            .collect()
    }
    // pub fn iter(&self) -> impl Iterator<Item = MatRef<'_, f64>> {
    //     self.calib.iter().map(|c| c.mat_ref())
    // }
    pub fn calib(&self) -> impl Iterator<Item = &Calib> {
        self.calib.iter()
    }
    pub fn pinv(&mut self) -> impl Iterator<Item = &CalibPinv<f64>> {
        self.pinv
            .iter_mut()
            .zip(&self.calib)
            .map(|(p, c)| p.get_or_insert_with(|| c.pseudoinverse()))
            .map(|p| &*p)
    }
    pub fn calib_pinv(&mut self) -> impl Iterator<Item = (&Calib, &CalibPinv<f64>)> {
        self.pinv
            .iter_mut()
            .zip(&self.calib)
            .map(|(p, c)| (c, p.get_or_insert_with(|| c.pseudoinverse())))
            .map(|(c, p)| (c, &*p))
    }
    pub fn cross_talks(&self) -> Vec<usize> {
        let n = self.calib[0].mask.len();
        (0..n)
            .map(|i| {
                self.calib
                    .iter()
                    .fold(0usize, |m, c| m + if c.mask[i] { 1 } else { 0 })
            })
            .collect()
    }
    pub fn n_cross_talks(&self) -> usize {
        self.cross_talks().iter().filter(|&&c| c > 1).count()
    }
}

impl Display for Reconstructor {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "RECONSTRUCTOR (non-zeros={}): ", self.area())?;
        for (c, ic) in self.calib.iter().zip(&self.pinv) {
            if let Some(ic) = ic {
                writeln!(f, " * {c} ; cond: {:6.3E}", ic.cond)?
            } else {
                writeln!(f, " * {c}")?
            }
        }
        Ok(())
    }
}

impl Update for Reconstructor {
    fn update(&mut self) {
        let data = Arc::clone(&self.data);
        self.estimate = Arc::new(
            self.calib_pinv()
                .flat_map(|(c, ic)| ic * c.mask(&data))
                .collect(),
        );
    }
}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for Reconstructor {
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for Reconstructor {
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.estimate.clone().into())
    }
}

impl Mul<Vec<Mat<f64>>> for &Reconstructor {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: Vec<Mat<f64>>) -> Self::Output {
        self.calib.iter().zip(rhs).map(|(c, m)| c * m).collect()
    }
}

impl Mul<MatRef<'_, f64>> for &Reconstructor {
    type Output = Vec<Mat<f64>>;

    fn mul(self, rhs: MatRef<'_, f64>) -> Self::Output {
        self.calib.iter().map(|c| c * rhs).collect()
    }
}

impl Div<&Reconstructor> for MatRef<'_, f64> {
    type Output = Vec<Mat<f64>>;

    fn div(self, rhs: &Reconstructor) -> Self::Output {
        rhs.pinv
            .iter()
            .filter_map(|ic| ic.as_ref().map(|ic| ic * self))
            .collect()
    }
}

impl SubAssign<Vec<Mat<f64>>> for &mut Reconstructor {
    fn sub_assign(&mut self, rhs: Vec<Mat<f64>>) {
        self.calib
            .iter_mut()
            .zip(rhs.into_iter())
            .for_each(|(mut c, r)| c -= r);
        self.pinv = vec![None; self.calib.len()];
    }
}
