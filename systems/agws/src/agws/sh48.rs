mod kernel;

use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use gmt_dos_clients_crseo::{sensors::Camera, OpticalModel};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

pub struct Sh48<const I: usize>(pub(crate) OpticalModel<Camera<I>>);

impl<const I: usize> Display for Sh48<I> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "AGWS SH48")?;
        write!(f, "{}", self.0)
    }
}

impl<const I: usize> Deref for Sh48<I> {
    type Target = OpticalModel<Camera<I>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const I: usize> DerefMut for Sh48<I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const I: usize> Update for Sh48<I> {
    fn update(&mut self) {
        self.0.update();
    }
}

impl<U, const I: usize> Read<U> for Sh48<I>
where
    U: UniqueIdentifier,
    OpticalModel<Camera<I>>: Read<U>,
{
    fn read(&mut self, data: Data<U>) {
        <_ as Read<U>>::read(&mut self.0, data);
    }
}
impl<U, const I: usize> Write<U> for Sh48<I>
where
    U: UniqueIdentifier,
    OpticalModel<Camera<I>>: Write<U>,
{
    fn write(&mut self) -> Option<Data<U>> {
        <_ as Write<U>>::write(&mut self.0)
    }
}
