use std::{any::type_name, fmt::Display, io, marker::PhantomData};

use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder, centroiding::CentroidsError, crseo::FromBuilder,
};
use interface::{Data, TryRead, TryUpdate, TryWrite, UniqueIdentifier, Write};

pub struct KernelFrame<T>(PhantomData<T>)
where
    T: KernelSpecs;
// OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
//     DeviceInitialize<T::Processor>;
impl<T> UniqueIdentifier for KernelFrame<T>
where
    T: KernelSpecs + Send + Sync,
    // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Input: UniqueIdentifier,
{
    type DataType = <<T as KernelSpecs>::Input as UniqueIdentifier>::DataType;
}
impl<T> Write<KernelFrame<T>> for OpticalModel<<T as KernelSpecs>::Sensor>
where
    T: KernelSpecs + Send + Sync,
    // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
    KernelFrame<T>:
        UniqueIdentifier<DataType = <<T as KernelSpecs>::Input as UniqueIdentifier>::DataType>,
    <T as KernelSpecs>::Input: UniqueIdentifier,
    Self: Write<<T as KernelSpecs>::Input>,
{
    fn write(&mut self) -> Option<Data<KernelFrame<T>>> {
        <Self as Write<<T as KernelSpecs>::Input>>::write(self)
            .map(|data| data.transmute::<KernelFrame<T>>())
    }
}

#[derive(Debug, thiserror::Error)]
pub enum KernelError {
    #[error("failed kernel centroiding initialization")]
    Centroiding(#[from] CentroidsError),
    #[error("failed to load kernel data")]
    IO(#[from] io::Error),
    #[error("failed to read pickle file")]
    Pickle(#[from] serde_pickle::Error),
}
type Result<T> = std::result::Result<T, KernelError>;

pub trait KernelSpecs {
    type Sensor: FromBuilder;
    type Processor;
    type Estimator;
    type Integrator;
    type Input: Send + Sync;
    type Data: Send + Sync;
    type Output: Send + Sync;
    fn processor(
        model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
    ) -> Result<Self::Processor>;
}

pub struct Kernel<T>
where
    T: KernelSpecs,
    // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
{
    pub(crate) processor: <T as KernelSpecs>::Processor,
    estimator: Option<<T as KernelSpecs>::Estimator>,
    integrator: Option<<T as KernelSpecs>::Integrator>,
}

impl<T> Display for Kernel<T>
where
    T: KernelSpecs,
    // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Processor: Display,
    <T as KernelSpecs>::Estimator: Display,
    <T as KernelSpecs>::Integrator: Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "{} KERNEL", type_name::<T>().to_uppercase())?;
        writeln!(f, "|* processor:")?;
        writeln!(f, "| {}", self.processor)?;
        writeln!(f, "|* estimator:")?;
        if let Some(estimator) = self.estimator.as_ref() {
            writeln!(f, "|* estimator:")?;
            writeln!(f, "| {}", estimator)?;
        }
        if let Some(integrator) = self.integrator.as_ref() {
            writeln!(f, "|* integrator:")?;
            writeln!(f, "| {}", integrator)?;
        }
        Ok(())
    }
}

impl<T> Kernel<T>
where
    T: KernelSpecs,
    // <T as KernelSpecs>::Sensor: DerefMut, // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
{
    pub fn new(
        model: &OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>,
        // estimator: T::Estimator,
    ) -> Result<Self> {
        Ok(Self {
            processor: <T as KernelSpecs>::processor(model)?,
            estimator: None,
            integrator: None,
        })
    }
    pub fn estimator(mut self, estimator: T::Estimator) -> Self {
        self.estimator = Some(estimator);
        self
    }
    pub fn processor(&self) -> &<T as KernelSpecs>::Processor {
        &self.processor
    }
}

// impl<T> TryRead<<T as KernelSpecs>::Input> for Kernel<T>
// where
//     T:  KernelSpecs ,
//     OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
//         DeviceInitialize<T::Processor>,
//     <T as KernelSpecs>::Input: UniqueIdentifier,
//     <T as KernelSpecs>::Processor: TryRead<<T as KernelSpecs>::Input>,
//     <T as KernelSpecs>::Data: UniqueIdentifier,
//     <T as KernelSpecs>::Processor: TryWrite<<T as KernelSpecs>::Data>,
//     <T as KernelSpecs>::Estimator: TryRead<<T as KernelSpecs>::Data>,
//     <T as KernelSpecs>::Output: UniqueIdentifier,
//     <T as KernelSpecs>::Estimator: TryWrite<<T as KernelSpecs>::Output>,
//     <T as KernelSpecs>::Integrator: TryRead<<T as KernelSpecs>::Output>,
// {
//     fn read(&mut self, data: Data<<T as KernelSpecs>::Input>) {
//         <<T as KernelSpecs>::Processor as TryRead<_>>::read(&mut self.processor, data);
//     }
// }

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

impl<T> TryRead<KernelFrame<T>> for Kernel<T>
where
    T: KernelSpecs,
    KernelFrame<T>:
        UniqueIdentifier<DataType = <<T as KernelSpecs>::Input as UniqueIdentifier>::DataType>,
    <T as KernelSpecs>::Input: UniqueIdentifier,
    <T as KernelSpecs>::Processor: TryRead<<T as KernelSpecs>::Input>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as KernelSpecs>::Processor: TryWrite<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Estimator: TryRead<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Estimator: TryWrite<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: TryRead<<T as KernelSpecs>::Output>,
    <<T as KernelSpecs>::Processor as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Processor as TryWrite<<T as KernelSpecs>::Data>>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryRead<<T as KernelSpecs>::Data>>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryWrite<<T as KernelSpecs>::Output>>::Error: 'static,
    <<T as KernelSpecs>::Integrator as TryRead<<T as KernelSpecs>::Output>>::Error: 'static,
    <<T as KernelSpecs>::Integrator as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Processor as TryRead<<T as KernelSpecs>::Input>>::Error: 'static,
{
    type Error = KernelReadError;

    fn try_read(
        &mut self,
        data: Data<KernelFrame<T>>,
    ) -> std::result::Result<&mut Self, <Self as TryRead<KernelFrame<T>>>::Error> {
        <<T as KernelSpecs>::Processor as TryRead<<T as KernelSpecs>::Input>>::boxed_try_read(
            &mut self.processor,
            data.transmute::<<T as KernelSpecs>::Input>(),
        )?;
        Ok(self)
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

impl<T> TryUpdate for Kernel<T>
where
    T: KernelSpecs,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as KernelSpecs>::Processor: TryWrite<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Estimator: TryRead<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Estimator: TryWrite<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: TryRead<<T as KernelSpecs>::Output>,
    <<T as KernelSpecs>::Processor as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Processor as TryWrite<<T as KernelSpecs>::Data>>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryRead<<T as KernelSpecs>::Data>>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryWrite<<T as KernelSpecs>::Output>>::Error: 'static,
    <<T as KernelSpecs>::Integrator as TryRead<<T as KernelSpecs>::Output>>::Error: 'static,
    <<T as KernelSpecs>::Integrator as TryUpdate>::Error: 'static,
{
    type Error = KernelUpdateError;

    fn try_update(&mut self) -> std::result::Result<&mut Self, Self::Error> {
        log::info!("updating kernel: {}", type_name::<T>());
        self.processor.boxed_try_update()?;
        if let Some(estimator) = self.estimator.as_mut() {
            if let Some(data) = <<T as KernelSpecs>::Processor as TryWrite<
                <T as KernelSpecs>::Data,
            >>::boxed_try_write(&mut self.processor)?
            {
                <<T as KernelSpecs>::Estimator as TryRead<<T as KernelSpecs>::Data>>::boxed_try_read(
                    estimator, data,
                )
                    ?;
                estimator.boxed_try_update()?;
            };
            if let Some(integrator) = self.integrator.as_mut() {
                if let Some(data) = <<T as KernelSpecs>::Estimator as TryWrite<
                    <T as KernelSpecs>::Output,
                >>::boxed_try_write(estimator)?
                {
                    <<T as KernelSpecs>::Integrator as TryRead<<T as KernelSpecs>::Output>>::boxed_try_read(
                        integrator, data,
                    )?;
                    integrator.boxed_try_update()?;
                };
            };
        };
        Ok(self)
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

impl<T> TryWrite<<T as KernelSpecs>::Output> for Kernel<T>
where
    T: KernelSpecs,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Integrator: TryWrite<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as KernelSpecs>::Processor: TryWrite<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Estimator: TryRead<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Estimator: TryWrite<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: TryRead<<T as KernelSpecs>::Output>,
    <<T as KernelSpecs>::Processor as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Processor as TryWrite<<T as KernelSpecs>::Data>>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryRead<<T as KernelSpecs>::Data>>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Estimator as TryWrite<<T as KernelSpecs>::Output>>::Error: 'static,
    <<T as KernelSpecs>::Integrator as TryRead<<T as KernelSpecs>::Output>>::Error: 'static,
    <<T as KernelSpecs>::Integrator as TryUpdate>::Error: 'static,
    <<T as KernelSpecs>::Integrator as TryWrite<<T as KernelSpecs>::Output>>::Error: 'static,
{
    type Error = KernelWriteError;

    fn try_write(
        &mut self,
    ) -> std::result::Result<
        Option<Data<<T as KernelSpecs>::Output>>,
        <Self as TryWrite<<T as KernelSpecs>::Output>>::Error,
    > {
        Ok(if let Some(integrator) = self.integrator.as_mut() {
            <<T as KernelSpecs>::Integrator as TryWrite<_>>::boxed_try_write(integrator)?
        } else {
            if let Some(estimator) = self.estimator.as_mut() {
                <<T as KernelSpecs>::Estimator as TryWrite<_>>::boxed_try_write(estimator)?
            } else {
                None
            }
        })
    }
}
