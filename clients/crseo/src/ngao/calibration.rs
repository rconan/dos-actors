use std::{
    ops::{Deref, DerefMut, Mul},
    sync::Arc,
};

use crseo::CrseoError;
use interface::{Data, Read, UniqueIdentifier, Update, Write};

#[derive(Debug, thiserror::Error)]
pub enum CalibratingError {
    #[error("crseo error")]
    Crseo(#[from] CrseoError),
}

/// Sensor calibration interface
pub trait Calibrating {
    type ProcessorData: Default;
    type Output;
    // type Calibrator;
    // fn calibrating(&self) -> Result<Self::Calibrator, CalibratingError>;
}

/// Sensor calibration
pub struct Calibration<C: Calibrating> {
    pub(crate) calibrator: C,
    pub(crate) output: Arc<C::Output>,
}

impl<C: Calibrating> Deref for Calibration<C> {
    type Target = C;

    fn deref(&self) -> &Self::Target {
        &self.calibrator
    }
}

impl<C: Calibrating> DerefMut for Calibration<C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.calibrator
    }
}

impl<C: Calibrating + Send + Sync> Update for Calibration<C>
where
    <C as Calibrating>::ProcessorData: Sync + Send,
    <C as Calibrating>::Output: Send + Sync,
    // for<'a> &'a C: Mul<&'a C::ProcessorData, Output = ()>,
{
    // fn update(&mut self) {
    //     &self.calibrator * &self.data
    // }
}

impl<C: Calibrating + Send + Sync, T: UniqueIdentifier<DataType = C::ProcessorData>> Read<T>
    for Calibration<C>
where
    <C as Calibrating>::ProcessorData: Send + Sync,
    <C as Calibrating>::Output: Send + Sync,
    for<'a> &'a C: Mul<&'a C::ProcessorData, Output = <C as Calibrating>::Output>,
{
    fn read(&mut self, data: Data<T>) {
        let value = data.as_arc();
        self.output = Arc::new(&self.calibrator * &value);
    }
}

impl<C: Calibrating + Send + Sync, T: UniqueIdentifier<DataType = C::Output>> Write<T>
    for Calibration<C>
where
    <C as Calibrating>::ProcessorData: Send + Sync,
    <C as Calibrating>::Output: Send + Sync,
{
    fn write(&mut self) -> Option<Data<T>> {
        Some(Data::from(&self.output))
    }
}
