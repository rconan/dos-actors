mod kernel;

use std::{
    convert::Infallible,
    fmt::Display,
    ops::{Deref, DerefMut},
};

use gmt_dos_clients_crseo::{OpticalModel, sensors::Camera};
use gmt_dos_clients_io::optics::SensorData;
use interface::{Data, Read, TryRead, TryUpdate, TryWrite, UniqueIdentifier, Update, Write};

use crate::kernels::{Kernel, KernelSpecs};

pub struct Sh24<const I: usize>(pub(crate) OpticalModel<Camera<I>>);
pub struct Sh24TT<const I: usize>(pub(crate) OpticalModel<Camera<I>>);

impl<const I: usize> Display for Sh24<I> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "AGWS SH24")?;
        write!(f, "{}", self.0)
    }
}

impl<const I: usize> Deref for Sh24<I> {
    type Target = OpticalModel<Camera<I>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const I: usize> DerefMut for Sh24<I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl<const I: usize> Deref for Sh24TT<I> {
    type Target = OpticalModel<Camera<I>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const I: usize> DerefMut for Sh24TT<I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const I: usize> TryUpdate for Sh24<I> {
    type Error = Infallible;

    fn try_update(&mut self) -> std::result::Result<&mut Self, Self::Error> {
        self.0.update();
        Ok(self)
    }
}

impl<const I: usize> TryWrite<SensorData> for Kernel<Sh24<I>> {
    type Error = Infallible;

    fn try_write(
        &mut self,
    ) -> std::result::Result<Option<Data<SensorData>>, <Self as TryWrite<SensorData>>::Error> {
        Ok(<<Sh24<I> as KernelSpecs>::Processor as Write<_>>::write(
            &mut self.processor,
        ))
    }
}

impl<U, const I: usize> TryRead<U> for Sh24<I>
where
    U: UniqueIdentifier,
    OpticalModel<Camera<I>>: Read<U>,
{
    type Error = Infallible;

    fn try_read(
        &mut self,
        data: Data<U>,
    ) -> std::result::Result<&mut Self, <Self as TryRead<U>>::Error> {
        <_ as Read<U>>::read(&mut self.0, data);
        Ok(self)
    }
}
impl<U, const I: usize> TryWrite<U> for Sh24<I>
where
    U: UniqueIdentifier,
    OpticalModel<Camera<I>>: Write<U>,
{
    type Error = Infallible;

    fn try_write(&mut self) -> std::result::Result<Option<Data<U>>, <Self as TryWrite<U>>::Error> {
        Ok(<_ as Write<U>>::write(&mut self.0))
    }
}
