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

use crate::{
    kernels::{KernelError, KernelSpecs},
    qp::ActiveOptics,
};

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

