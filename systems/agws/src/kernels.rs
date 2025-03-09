use std::{any::type_name, fmt::Display, io, marker::PhantomData};

use gmt_dos_clients_crseo::{
    centroiding::CentroidsError, crseo::FromBuilder, OpticalModel, OpticalModelBuilder,
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};

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
    processor: <T as KernelSpecs>::Processor,
    estimator: <T as KernelSpecs>::Estimator,
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
        write!(f, "| {}", self.estimator)?;
        writeln!(f, "|* integrator:")?;
        if let Some(integrator) = self.integrator.as_ref() {
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
        estimator: T::Estimator,
    ) -> Result<Self> {
        Ok(Self {
            processor: <T as KernelSpecs>::processor(model)?,
            estimator,
            integrator: None,
        })
    }
}

// impl<T> Read<<T as KernelSpecs>::Input> for Kernel<T>
// where
//     T:  KernelSpecs ,
//     OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
//         DeviceInitialize<T::Processor>,
//     <T as KernelSpecs>::Input: UniqueIdentifier,
//     <T as KernelSpecs>::Processor: Read<<T as KernelSpecs>::Input>,
//     <T as KernelSpecs>::Data: UniqueIdentifier,
//     <T as KernelSpecs>::Processor: Write<<T as KernelSpecs>::Data>,
//     <T as KernelSpecs>::Estimator: Read<<T as KernelSpecs>::Data>,
//     <T as KernelSpecs>::Output: UniqueIdentifier,
//     <T as KernelSpecs>::Estimator: Write<<T as KernelSpecs>::Output>,
//     <T as KernelSpecs>::Integrator: Read<<T as KernelSpecs>::Output>,
// {
//     fn read(&mut self, data: Data<<T as KernelSpecs>::Input>) {
//         <<T as KernelSpecs>::Processor as Read<_>>::read(&mut self.processor, data);
//     }
// }
impl<T> Read<KernelFrame<T>> for Kernel<T>
where
    T: KernelSpecs,
    // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
    KernelFrame<T>:
        UniqueIdentifier<DataType = <<T as KernelSpecs>::Input as UniqueIdentifier>::DataType>,
    <T as KernelSpecs>::Input: UniqueIdentifier,
    <T as KernelSpecs>::Processor: Read<<T as KernelSpecs>::Input>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as KernelSpecs>::Processor: Write<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Estimator: Read<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Estimator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: Read<<T as KernelSpecs>::Output>,
{
    fn read(&mut self, data: Data<KernelFrame<T>>) {
        <<T as KernelSpecs>::Processor as Read<<T as KernelSpecs>::Input>>::read(
            &mut self.processor,
            data.transmute::<<T as KernelSpecs>::Input>(),
        );
    }
}
impl<T> Update for Kernel<T>
where
    T: KernelSpecs,
    // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as KernelSpecs>::Processor: Write<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Estimator: Read<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Estimator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: Read<<T as KernelSpecs>::Output>,
{
    fn update(&mut self) {
        self.processor.update();
        <<T as KernelSpecs>::Processor as Write<<T as KernelSpecs>::Data>>::write(
            &mut self.processor,
        )
        .map(|data| {
            <<T as KernelSpecs>::Estimator as Read<<T as KernelSpecs>::Data>>::read(
                &mut self.estimator,
                data,
            )
        });
        self.estimator.update();
        <<T as KernelSpecs>::Estimator as Write<<T as KernelSpecs>::Output>>::write(
            &mut self.estimator,
        )
        .map(|data| {
            self.integrator.as_mut().map(|integrator| {
                <<T as KernelSpecs>::Integrator as Read<<T as KernelSpecs>::Output>>::read(
                    integrator, data,
                );
                integrator.update();
            })
        });
    }
}
impl<T> Write<<T as KernelSpecs>::Output> for Kernel<T>
where
    T: KernelSpecs,
    // OpticalModelBuilder<<T::Sensor as FromBuilder>::ComponentBuilder>:
    //     DeviceInitialize<T::Processor>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Integrator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Data: UniqueIdentifier,
    <T as KernelSpecs>::Processor: Write<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Estimator: Read<<T as KernelSpecs>::Data>,
    <T as KernelSpecs>::Output: UniqueIdentifier,
    <T as KernelSpecs>::Estimator: Write<<T as KernelSpecs>::Output>,
    <T as KernelSpecs>::Integrator: Read<<T as KernelSpecs>::Output>,
{
    fn write(&mut self) -> Option<Data<<T as KernelSpecs>::Output>> {
        if let Some(integrator) = self.integrator.as_mut() {
            <<T as KernelSpecs>::Integrator as Write<_>>::write(integrator)
        } else {
            <<T as KernelSpecs>::Estimator as Write<_>>::write(&mut self.estimator)
        }
    }
}
