/*!
#  GMT Optical Model

A [gmt_dos-actors] client for optical propagation through the GMT using [crseo].

[OpticalModel] represents the GMT optics model with M1 and M2 segmented optical prescriptions.
An optical sensor may be added to [OpticalModel], available sensors are listed in the [sensors] module.
The data processing algorithms applied to the [sensors] output are defined in the [processing] module.
The [calibration] module contains the traits and types used for the calibration of the sensors with respect to either
the rigid body motion or the shapes of M1 and M2.


[gmt_dos-actors]: https://docs.rs/gmt_dos-actors
[crseo]: https://docs.rs/crseo
 */

/* pub(crate) mod optical_model;
pub use optical_model::{
    OpticalModel, OpticalModelBuilder, OpticalModelOptions, PSSnOptions, ShackHartmannOptions,
};
pub(crate) mod shackhartmann;

mod sensor;
pub use sensor::SensorBuilder;
*/

use std::ops::{Deref, DerefMut};

use interface::{Data, TimerMarker, UniqueIdentifier, Update, Write};

mod error;
pub use error::{CeoError, Result};

pub mod ngao;
// pub use ngao::{DetectorFrame, GuideStar, ResidualM2modes, ResidualPistonMode, WavefrontSensor};
//     , , OpticalModel, OpticalModelBuilder, ,
//     , WavefrontSensor,
// };

mod wavefront_stats;
pub use wavefront_stats::WavefrontStats;

mod pyramid;
pub use pyramid::{PyramidCalibrator, PyramidCommand, PyramidMeasurements, PyramidProcessor};

// mod ltao;
// pub use ltao::{
//     Calibrate, CalibrateSegment, CalibrationMode, Centroids, DispersedFringeSensor,
//     DispersedFringeSensorBuidler, DispersedFringeSensorProcessing, NoSensor, OpticalModel,
//     OpticalModelBuilder, Reconstructor, WaveSensor, WaveSensorBuilder,
// };

pub mod calibration;
mod optical_model;
pub mod processing;
pub mod sensors;

pub use optical_model::{builder::OpticalModelBuilder, OpticalModel};
pub use processing::{centroiding, DispersedFringeSensorProcessing};

impl<T> TimerMarker for OpticalModel<T> {}

/// Interface for initialization of data processing pipeline
pub trait DeviceInitialize<D> {
    /// Initialize a data processing pipeline `D`
    fn initialize(&self, device: &mut D);
}

pub trait Processing {
    type ProcessorData;
    fn processing(&self) -> Self::ProcessorData;
}

/// Sensor data processor
#[derive(Default, Debug)]
pub struct Processor<P: Processing>(P);

impl<P: Processing> From<P> for Processor<P> {
    fn from(value: P) -> Self {
        Processor(value)
    }
}

impl<P: Processing> Deref for Processor<P> {
    type Target = P;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<P: Processing> DerefMut for Processor<P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<P: Processing + Send + Sync> Update for Processor<P> {
    // fn update(&mut self) {
    //     self.processing();
    // }
}

impl<P, T> Write<T> for Processor<P>
where
    P: Processing + Send + Sync,
    T: UniqueIdentifier<DataType = P::ProcessorData>,
{
    fn write(&mut self) -> Option<Data<T>> {
        let data: <P as Processing>::ProcessorData = self.processing();
        Some(Data::new(data))
    }
}

/*

#[cfg(feature = "crseo")]
/// GMT M1 & M2 state
#[derive(UID)]
#[uid(data = "crseo::gmt::SegmentsDof")]
pub enum GmtState {}
pub enum PointingError {}
impl UniqueIdentifier for PointingError {
    type DataType = (f64, f64);
}

#[cfg(feature = "fem")]
impl<S> dos_actors::io::Write<M1modes> for fem::dos::DiscreteModalSolver<S>
where
    S: fem::dos::Solver + Default,
{
    fn write(&mut self) -> Option<std::sync::Arc<dos_actors::io::Data<M1modes>>> {
        let mut data: std::sync::Arc<dos_actors::io::Data<fem::dos::M1SegmentsAxialD>> =
            self.write()?;
        let inner = std::sync::Arc::get_mut(&mut data)?;
        Some(std::sync::Arc::new(inner.into()))
    }
}
 */
