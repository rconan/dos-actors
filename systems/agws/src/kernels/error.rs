use std::{fmt::Display, io};

use gmt_dos_clients_crseo::centroiding::CentroidsError;

#[derive(Debug, thiserror::Error)]
pub enum KernelError {
    #[error("failed kernel centroiding initialization")]
    Centroiding(#[from] CentroidsError),
    #[error("failed to load kernel data")]
    IO(#[from] io::Error),
    #[error("failed to read pickle file")]
    Pickle(#[from] serde_pickle::Error),
}

#[derive(Debug)]
pub struct KernelReadError(Box<dyn std::error::Error + Send + Sync>);
impl Display for KernelReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Kernel failed to read data caused by")?;
        std::fmt::Display::fmt(&self.0, f)
    }
}
impl std::error::Error for KernelReadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(self.0.as_ref())
    }
}
impl From<Box<dyn std::error::Error + Send + Sync>> for KernelReadError {
    fn from(value: Box<dyn std::error::Error + Send + Sync>) -> Self {
        KernelReadError(value)
    }
}

#[derive(Debug)]
pub struct KernelUpdateError(Box<dyn std::error::Error + Send + Sync>);
impl Display for KernelUpdateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Kernel failed to update cause by")?;
        std::fmt::Display::fmt(&self.0, f)
    }
}
impl std::error::Error for KernelUpdateError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(self.0.as_ref())
    }
}
impl From<Box<dyn std::error::Error + Send + Sync>> for KernelUpdateError {
    fn from(value: Box<dyn std::error::Error + Send + Sync>) -> Self {
        KernelUpdateError(value)
    }
}

#[derive(Debug)]
pub struct KernelWriteError(Box<dyn std::error::Error + Send + Sync>);
impl Display for KernelWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Kernel failed to write data caused by")?;
        std::fmt::Display::fmt(&self.0, f)
    }
}
impl std::error::Error for KernelWriteError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(self.0.as_ref())
    }
}
impl From<Box<dyn std::error::Error + Send + Sync>> for KernelWriteError {
    fn from(value: Box<dyn std::error::Error + Send + Sync>) -> Self {
        KernelWriteError(value)
    }
}
