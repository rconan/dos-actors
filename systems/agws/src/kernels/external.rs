use interface::{Data, Read, TryRead, TryUpdate, UniqueIdentifier};

use super::{Kernel, KernelSpecs};

pub trait ExternalParameter {}

impl<T, U> TryRead<U> for Kernel<T>
where
    T: KernelSpecs,
    U: UniqueIdentifier + ExternalParameter,
    <T as KernelSpecs>::Processor: Sync + Send,
    <T as KernelSpecs>::Estimator: Sync + Send,
    <T as KernelSpecs>::Estimator: Read<U>,
    <T as KernelSpecs>::Controller: Sync + Send,
    Kernel<T>: TryUpdate,
{
    type Error = <<T as KernelSpecs>::Estimator as TryRead<U>>::Error;

    fn try_read(
        &mut self,
        data: Data<U>,
    ) -> std::result::Result<&mut Self, <Self as TryRead<U>>::Error> {
        if let Some(estimator) = self.estimator.as_mut() {
            <<T as KernelSpecs>::Estimator as TryRead<U>>::try_read(estimator, data)?;
        }
        Ok(self)
    }
}
