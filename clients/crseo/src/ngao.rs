//! # Natural Guide Star Adaptive Optics
//!
//! Integrated model of the NGAO Observing Performance Mode of the GMT

use gmt_dos_clients_io::optics::M2modes;
use interface::{Data, Read, UID};

use crate::{OpticalModel, Processor, PyramidProcessor};

mod wavefront_sensor;
pub use wavefront_sensor::{DetectorFrame, GuideStar}; //, WavefrontSensor};

mod calibration;
pub use calibration::{Calibrating, CalibratingError, Calibration};

#[derive(UID)]
pub enum ResidualPistonMode {}

#[derive(UID)]
#[alias(name = M2modes, client = OpticalModel, traits = Read)]
pub enum ResidualM2modes {}

impl Read<DetectorFrame> for Processor<PyramidProcessor> {
    fn read(&mut self, data: Data<DetectorFrame>) {
        self.frame = data.as_arc();
    }
}

/* mod optical_model;
pub use optical_model::OpticalModel;

mod builder;
pub use builder::OpticalModelBuilder;

// mod sensor_fusion;
// pub use sensor_fusion::{HdfsIntegrator, HdfsOrNot, PwfsIntegrator};





pub enum ResidualM2modes {}
impl ::interface::UniqueIdentifier for ResidualM2modes {
    const PORT: u16 = <M2modes as ::interface::UniqueIdentifier>::PORT;
    type DataType = <M2modes as ::interface::UniqueIdentifier>::DataType;
}
impl<T: SegmentWiseSensor> ::interface::Read<ResidualM2modes> for OpticalModel<T> {
    #[inline]
    fn read(&mut self, data: ::interface::Data<ResidualM2modes>) {
        <Self as ::interface::Read<M2modes>>::read(self, data.transmute());
    }
}

#[derive(UID)]
pub enum M1Rxy {}
 */
