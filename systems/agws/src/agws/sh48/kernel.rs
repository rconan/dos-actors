use std::ops::{Deref, DerefMut};

use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModelBuilder,
    calibration::{CalibrationMode, ClosedLoopCalib, Reconstructor},
    centroiding::CentroidsProcessing,
    crseo::FromBuilder,
    sensors::Camera,
};
use gmt_dos_clients_io::{
    Estimate,
    optics::{Dev, Frame, SensorData},
};
use interface::{TryRead, TryUpdate, TryWrite, UniqueIdentifier};

use crate::kernels::{Kernel, KernelError, KernelSpecs};

use super::Sh48;

type Result<T> = std::result::Result<T, KernelError>;

pub struct Sh48Kern<T: KernelSpecs>(pub(crate) Kernel<T>);
impl<T: KernelSpecs> Deref for Sh48Kern<T> {
    type Target = Kernel<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<T: KernelSpecs> DerefMut for Sh48Kern<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T: KernelSpecs> TryUpdate for Sh48Kern<T>
where
    Kernel<T>: TryUpdate,
{
    type Error = <Kernel<T> as TryUpdate>::Error;

    fn try_update(&mut self) -> std::result::Result<&mut Self, Self::Error> {
        self.0.try_update()?;
        Ok(self)
    }
}

impl<T: KernelSpecs, U: UniqueIdentifier> TryRead<U> for Sh48Kern<T>
where
    Kernel<T>: TryRead<U>,
{
    type Error = <Kernel<T> as TryRead<U>>::Error;

    fn try_read(
        &mut self,
        data: interface::Data<U>,
    ) -> std::result::Result<&mut Self, <Self as TryRead<U>>::Error> {
        self.0.try_read(data)?;
        Ok(self)
    }
}

impl<T: KernelSpecs, U: UniqueIdentifier> TryWrite<U> for Sh48Kern<T>
where
    Kernel<T>: TryWrite<U>,
{
    type Error = <Kernel<T> as TryWrite<U>>::Error;

    fn try_write(
        &mut self,
    ) -> std::result::Result<Option<interface::Data<U>>, <Self as TryWrite<U>>::Error> {
        self.0.try_write()
    }
}
impl<const I: usize> KernelSpecs for Sh48<I> {
    type Sensor = Camera<I>;

    type Processor = CentroidsProcessing;

    type Estimator = Reconstructor<CalibrationMode, ClosedLoopCalib>;

    type Integrator = gmt_dos_clients::integrator::Integrator<Estimate>;

    type Input = Frame<Dev>;

    type Data = SensorData;

    type Output = Estimate;

    fn processor(
        model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
    ) -> Result<Self::Processor> {
        let mut centroids = CentroidsProcessing::try_from(model)?;
        model.initialize(&mut centroids);
        Ok(centroids)
    }
}
