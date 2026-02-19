use std::ops::{Deref, DerefMut};

use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModelBuilder, calibration::Reconstructor,
    centroiding::CentroidsProcessing, crseo::FromBuilder, sensors::Camera,
};
use gmt_dos_clients_io::{
    gmt_m2::{M2RigidBodyMotions, fsm::M2FSMFsmCommand},
    optics::{Dev, Frame, SensorData},
};
use interface::{TryRead, TryUpdate, TryWrite, UniqueIdentifier};

use crate::kernels::{Kernel, KernelError, KernelSpecs};

use super::{Sh24, Sh24TT};

type Result<T> = std::result::Result<T, KernelError>;

pub struct Sh24Kern<T: KernelSpecs>(pub(crate) Kernel<T>);
impl<T: KernelSpecs> Deref for Sh24Kern<T> {
    type Target = Kernel<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<T: KernelSpecs> DerefMut for Sh24Kern<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T: KernelSpecs> TryUpdate for Sh24Kern<T>
where
    Kernel<T>: TryUpdate,
{
    type Error = <Kernel<T> as TryUpdate>::Error;

    fn try_update(&mut self) -> std::result::Result<&mut Self, Self::Error> {
        self.0.try_update()?;
        Ok(self)
    }
}

impl<T: KernelSpecs, U: UniqueIdentifier> TryRead<U> for Sh24Kern<T>
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

impl<T: KernelSpecs, U: UniqueIdentifier> TryWrite<U> for Sh24Kern<T>
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

impl<const I: usize> KernelSpecs for Sh24<I> {
    type Sensor = Camera<I>;

    type Processor = CentroidsProcessing;

    type Estimator = Reconstructor;

    type Integrator = gmt_dos_clients::integrator::Integrator<M2FSMFsmCommand>;

    type Input = Frame<Dev>;

    type Data = SensorData;

    type Output = M2FSMFsmCommand;

    fn processor(
        model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
    ) -> Result<Self::Processor> {
        let mut centroids = CentroidsProcessing::try_from(model)?;
        model.initialize(&mut centroids);
        Ok(centroids)
    }
}
impl<const I: usize> KernelSpecs for Sh24TT<I> {
    type Sensor = Camera<I>;

    type Processor = CentroidsProcessing;

    type Estimator = Reconstructor;

    type Integrator = gmt_dos_clients::integrator::Integrator<M2RigidBodyMotions>;

    type Input = Frame<Dev>;

    type Data = SensorData;

    type Output = M2RigidBodyMotions;

    fn processor(
        model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
    ) -> Result<Self::Processor> {
        let mut centroids = CentroidsProcessing::try_from(model)?;
        model.initialize(&mut centroids);
        Ok(centroids)
    }
}
