use gmt_dos_clients_crseo::{
    calibration::{CalibrationMode, ClosedLoopCalib, Reconstructor},
    centroiding::CentroidsProcessing,
    crseo::FromBuilder,
    sensors::Camera,
    DeviceInitialize, OpticalModelBuilder,
};
use gmt_dos_clients_io::{
    optics::{Dev, Frame, SensorData},
    Estimate,
};

use crate::kernels::{KernelError, KernelSpecs};

use super::Sh48;

type Result<T> = std::result::Result<T, KernelError>;

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
