//! Active Optics Control Algorithm
//!
//! Implementation of the GMT Active Optics Control Algorithm
//! as described in "Natural Seeing Control Algorithms Description
//! Document" (GMT-DOC-04426)

use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModelBuilder, centroiding::CentroidsProcessing, crseo::FromBuilder,
    sensors::Camera,
};
use gmt_dos_clients_io::{
    Estimate,
    optics::{Dev, Frame, SensorData},
};
use interface::{
    Data, Read, Right, TryWrite, Update, Write,
    optics::{
        OpticsState,
        state::{MirrorState, OpticalState},
    },
};
use std::{
    convert::Infallible,
    io::{self},
    ops::{Deref, DerefMut},
    sync::Arc,
};

use crate::kernels::{Kernel, KernelError, KernelSpecs};

pub mod active_optics;
pub mod qp;
#[doc(inline)]
pub use qp::QP;

// Ratio between cost J1 (WFS slope fitting) and J3 (control effort).
pub const J1_J3_RATIO: f64 = 10.0;
// Minimum value assigned to rho3 factor
pub const MIN_RHO3: f64 = 1.0e-6;

#[derive(Debug, thiserror::Error)]
pub enum QpError {
    #[error("failed to open data file {filename:?}")]
    Open { filename: String, source: io::Error },
    #[error("failed to deserialize QP data file")]
    Pickle(#[from] serde_pickle::Error),
    #[error("failed to setup QP problem")]
    QpSetup(#[from] osqp::SetupError),
    #[error("failed to solve QP problem")]
    QpSolve,
    #[error("expected some calibration matrix, found none")]
    MissingCalibration,
}

pub struct ActiveOptics<
    const I: usize,
    const M1_RBM: usize,
    const M2_RBM: usize,
    const M1_BM: usize,
    const N_MODE: usize,
>(active_optics::ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>);
impl<
    const I: usize,
    const M1_RBM: usize,
    const M2_RBM: usize,
    const M1_BM: usize,
    const N_MODE: usize,
> Deref for ActiveOptics<I, M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    type Target = active_optics::ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<
    const I: usize,
    const M1_RBM: usize,
    const M2_RBM: usize,
    const M1_BM: usize,
    const N_MODE: usize,
> DerefMut for ActiveOptics<I, M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<
    const I: usize,
    const M1_RBM: usize,
    const M2_RBM: usize,
    const M1_BM: usize,
    const N_MODE: usize,
> KernelSpecs for ActiveOptics<I, M1_RBM, M2_RBM, M1_BM, N_MODE>
{
    type Sensor = Camera<I>;

    type Processor = CentroidsProcessing;

    type Estimator = active_optics::ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>;

    type Integrator = gmt_dos_clients::integrator::Integrator<Estimate>;

    type Input = Frame<Dev>;

    type Data = SensorData;

    type Output = Estimate;

    fn processor(
        model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
    ) -> Result<Self::Processor, KernelError> {
        let mut centroids = CentroidsProcessing::try_from(model)?;
        model.initialize(&mut centroids);
        Ok(centroids)
    }
}
// impl<const M1_RBM: usize, const M2_RBM: usize, const M1_BM: usize, const N_MODE: usize>
//     Write<OpticsState> for ActiveOptics<M1_RBM, M2_RBM, M1_BM, N_MODE>
// {
//     fn write(&mut self) -> Option<Data<OpticsState>> {
//         let m1 = MirrorState::new(self.u[..42].chunks(6), self.u[84..].chunks(M1_BM));
//         let m2 = MirrorState::from_rbms(&self.u[42..84]);
//         Some(Data::new(OpticalState::new(m1, m2)))
//     }
// }

#[derive(Default)]
pub struct Estimate2OpticsState {
    u: Arc<Vec<f64>>,
}

impl Estimate2OpticsState {
    pub fn new() -> Self {
        Self {
            u: Arc::new(vec![0f64; 84 + 27 * 7]),
        }
    }
}

impl Update for Estimate2OpticsState {}
impl Read<Estimate> for Estimate2OpticsState {
    fn read(&mut self, data: Data<Estimate>) {
        self.u = data.into_arc();
    }
}
impl Write<Right<OpticsState>> for Estimate2OpticsState {
    fn write(&mut self) -> Option<Data<Right<OpticsState>>> {
        let m1 = MirrorState::new(self.u[..42].chunks(6), self.u[84..].chunks(27));
        let m2 = MirrorState::from_rbms(&self.u[42..84]);
        Some(Data::new(OpticalState::new(m1, m2)))
    }
}
impl Write<OpticsState> for Estimate2OpticsState {
    fn write(&mut self) -> Option<Data<OpticsState>> {
        let m1 = MirrorState::new(self.u[..42].chunks(6), self.u[84..].chunks(27));
        let m2 = MirrorState::from_rbms(&self.u[42..84]);
        Some(Data::new(OpticalState::new(m1, m2)))
    }
}

impl<
    const I: usize,
    const M1_RBM: usize,
    const M2_RBM: usize,
    const M1_BM: usize,
    const N_MODE: usize,
> TryWrite<SensorData> for Kernel<ActiveOptics<I, M1_RBM, M2_RBM, M1_BM, N_MODE>>
{
    type Error = Infallible;

    fn try_write(
        &mut self,
    ) -> std::result::Result<Option<Data<SensorData>>, <Self as TryWrite<SensorData>>::Error> {
        Ok(
            <<ActiveOptics<I, M1_RBM, M2_RBM, M1_BM, N_MODE> as KernelSpecs>::Processor as Write<
                _,
            >>::write(&mut self.processor),
        )
    }
}
