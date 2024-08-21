use crate::ltao::calibration::calib::CalibPinv;
use crate::ltao::calibration::Calib;
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use std::fmt::{Display, Formatter};
use std::sync::Arc;

#[derive(Debug, Default)]
pub struct Reconstructor {
    calib: Vec<Calib>,
    pinv: Vec<CalibPinv<f64>>,
    data: Arc<Vec<f64>>,
    estimate: Arc<Vec<f64>>,
}

impl Reconstructor {
    pub fn new(calib: Vec<Calib>) -> Self {
        Self {
            calib,
            ..Default::default()
        }
    }
    pub fn pseudoinverse(&mut self) {
        self.pinv = self.calib.iter().map(|c| c.pseudoinverse()).collect();
    }
    pub fn area(&self) -> usize {
        self.calib.iter().map(|c| c.area()).sum()
    }
}

impl Display for Reconstructor {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        if self.pinv.is_empty() {
            writeln!(
                f,
                "RECONSTRUCTOR (non-zeros={}; w/o pseudo-inverse): ",
                self.area()
            )?;
            for c in self.calib.iter() {
                writeln!(f, " * {c}")?
            }
        } else {
            writeln!(f, "RECONSTRUCTOR (non-zeros={}): ", self.area())?;
            for (c, ic) in self.calib.iter().zip(&self.pinv) {
                writeln!(f, " * {c} ; cond: {:6.3E}", ic.cond)?
            }
        }
        Ok(())
    }
}

impl Update for Reconstructor {
    fn update(&mut self) {
        self.estimate = Arc::new(
            self.calib
                .iter()
                .zip(&self.pinv)
                .flat_map(|(c, ic)| ic * c.mask(&self.data))
                .collect(),
        );
    }
}

impl<U: UniqueIdentifier<DataType=Vec<f64>>> Read<U> for Reconstructor {
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

impl<U: UniqueIdentifier<DataType=Vec<f64>>> Write<U> for Reconstructor {
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.estimate.clone().into())
    }
}
