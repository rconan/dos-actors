// WARNING: DEPRECATED MODULE

use crseo::{
    cu, wavefrontsensor::Calibration, Atmosphere, Cu, Fwhm, Gmt, PSSnEstimates, Source,
    WavefrontSensor,
};
use gmt_dos_clients_domeseeing::DomeSeeing;
use interface::{Data, Read, Update, Write};
use nalgebra as na;
use std::{
    fmt::Debug,
    ops::{Deref, DerefMut},
};

mod builder;
pub use builder::OpticalModelBuilder;
mod options;
pub use options::{OpticalModelOptions, PSSnOptions, ShackHartmannOptions};

type Cuf32 = Cu<cu::Single>;

/// Sensor data transform operator
pub enum SensorFn {
    None,
    Fn(Box<dyn Fn(Vec<f64>) -> Vec<f64> + Send>),
    Matrix(na::DMatrix<f64>),
    Calibration(Calibration),
}
impl Default for SensorFn {
    fn default() -> Self {
        SensorFn::None
    }
}
impl Debug for SensorFn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::None => write!(f, "None"),
            Self::Fn(_arg0) => f.debug_tuple("Fn").finish(),
            Self::Matrix(arg0) => f.debug_tuple("Matrix").field(arg0).finish(),
            Self::Calibration(arg0) => f.debug_tuple("Calibration").field(arg0).finish(),
        }
    }
}
impl From<na::DMatrix<f64>> for SensorFn {
    fn from(value: na::DMatrix<f64>) -> Self {
        SensorFn::Matrix(value)
    }
}
impl From<Calibration> for SensorFn {
    fn from(value: Calibration) -> Self {
        SensorFn::Calibration(value)
    }
}
impl From<Box<dyn Fn(Vec<f64>) -> Vec<f64> + Send>> for SensorFn {
    fn from(value: Box<dyn Fn(Vec<f64>) -> Vec<f64> + Send>) -> Self {
        SensorFn::Fn(value)
    }
}

/* /// GmtBuilder Optical Model
pub struct OpticalModel {
    pub gmt: Gmt,
    pub src: Source,
    pub sensor: Option<Box<dyn WavefrontSensor>>,
    pub segment_wise_sensor: Option<Box<dyn WavefrontSensor>>,
    pub atm: Option<Atmosphere>,
    pub dome_seeing: Option<DomeSeeing>,
    pub static_aberration: Option<Cuf32>,
    pub pssn: Option<Box<dyn PSSnEstimates>>,
    pub sensor_fn: SensorFn,
    pub(crate) frame: Option<Vec<f32>>,
    tau: f64,
}
impl OpticalModel {
    pub fn builder() -> OpticalModelBuilder {
        OpticalModelBuilder::new()
    }
    pub fn sensor_matrix_transform(&mut self, mat: na::DMatrix<f64>) -> &mut Self {
        self.sensor_fn = SensorFn::Matrix(mat);
        self
    }
}

impl Update for OpticalModel {
    fn update(&mut self) {
        self.src.through(&mut self.gmt).xpupil();
        if let Some(atm) = &mut self.atm {
            atm.secs += self.tau;
            self.src.through(atm);
        }
        if let Some(dome_seeing) = &mut self.dome_seeing {
            self.src.add_same(dome_seeing.next().unwrap().as_slice());
        }
        if let Some(static_aberration) = &self.static_aberration {
            self.src.add_same(&[static_aberration]);
        }
        if let Some(sensor) = &mut self.sensor {
            //self.src.through(sensor);
            sensor.deref_mut().propagate(&mut self.src);
        }
        if let Some(sensor) = &mut self.segment_wise_sensor {
            (*sensor).propagate(&mut self.src);
        }
        if let Some(pssn) = &mut self.pssn {
            self.src.through(pssn);
        }
    }
}

impl gmt_dos_clients::interface::TimerMarker for OpticalModel {}

#[cfg(feature = "crseo")]
impl Read<super::GmtState> for OpticalModel {
    fn read(&mut self, data: Data<super::GmtState>) {
        if let Err(e) = &data.apply_to(&mut self.gmt) {
            crate::print_error("Failed applying GmtBuilder state", e);
        }
    }
}
impl Read<super::M1RigidBodyMotions> for OpticalModel {
    fn read(&mut self, data: Data<super::M1RigidBodyMotions>) {
        data.chunks(6).enumerate().for_each(|(sid0, v)| {
            self.gmt
                .m1_segment_state((sid0 + 1) as i32, &v[..3], &v[3..]);
        });
    }
}
impl Read<super::M1ModeShapes> for OpticalModel {
    fn read(&mut self, data: Data<super::M1ModeShapes>) {
        self.gmt.m1_modes(&data);
    }
}
impl Write<super::M1ModeShapes> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::M1ModeShapes>> {
        Some(Data::new(self.gmt.a1.clone()))
    }
}
impl Read<super::M2RigidBodyMotions> for OpticalModel {
    fn read(&mut self, data: Data<super::M2RigidBodyMotions>) {
        data.chunks(6).enumerate().for_each(|(sid0, v)| {
            self.gmt
                .m2_segment_state((sid0 + 1) as i32, &v[..3], &v[3..]);
        });
    }
}
impl Read<super::M2rxy> for OpticalModel {
    fn read(&mut self, data: Data<super::M2rxy>) {
        let t_xyz = vec![0f64; 3];
        let mut r_xyz = vec![0f64; 3];
        data.chunks(2).enumerate().for_each(|(sid0, v)| {
            r_xyz[0] = v[0];
            r_xyz[1] = v[1];
            self.gmt.m2_segment_state((sid0 + 1) as i32, &t_xyz, &r_xyz);
        });
    }
}
impl Read<super::M2modes> for OpticalModel {
    fn read(&mut self, data: Data<super::M2modes>) {
        self.gmt.m2_modes(&data);
    }
}
impl Write<super::M2modes> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::M2modes>> {
        Some(Data::new(self.gmt.a2.clone()))
    }
}
impl Write<super::WfeRms> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::WfeRms>> {
        Some(Data::new(self.src.wfe_rms()))
    }
}
impl Write<super::Wavefront> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::Wavefront>> {
        Some(Data::new(self.src.phase().to_vec()))
    }
}
impl Write<super::TipTilt> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::TipTilt>> {
        Some(Data::new(self.src.gradients()))
    }
}
impl Write<super::SegmentWfe> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::SegmentWfe>> {
        Some(Data::new(
            self.src
                .segment_wfe()
                .into_iter()
                .flat_map(|(p, s)| vec![p, s])
                .collect(),
        ))
    }
}
impl Write<super::SegmentWfeRms> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::SegmentWfeRms>> {
        Some(Data::new(self.src.segment_wfe_rms()))
    }
}
impl Write<super::SegmentPiston> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::SegmentPiston>> {
        Some(Data::new(self.src.segment_piston()))
    }
}
impl Write<super::SegmentGradients> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::SegmentGradients>> {
        Some(Data::new(self.src.segment_gradients()))
    }
}
impl Write<super::SegmentTipTilt> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::SegmentTipTilt>> {
        Some(Data::new(self.src.segment_gradients()))
    }
}
impl Write<super::PSSn> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::PSSn>> {
        match self.pssn {
            Some(ref mut pssn) => Some(Data::new(pssn.estimates())),
            None => panic!("PSSn is not declared for this optical model"),
        }
    }
}
impl Write<super::PSSnFwhm> for OpticalModel {
    fn write(&mut self) -> Option<Data<super::PSSnFwhm>> {
        match self.pssn {
            Some(ref mut pssn) => {
                let mut fwhm = Fwhm::new();
                fwhm.build(&mut self.src);
                let data: Vec<f64> = vec![pssn.estimates(), fwhm.from_complex_otf(&pssn.otf())]
                    .into_iter()
                    .flatten()
                    .collect();
                Some(Data::new(data))
            }
            None => panic!("PSSn is not declared for this optical model"),
        }
    }
}

impl Read<super::PointingError> for OpticalModel {
    fn read(&mut self, data: Data<super::PointingError>) {
        self.gmt.pointing_error = Some(data.deref().clone());
    }
}
/// Source wavefront error RMS `[m]`
#[derive(UID)]
pub enum WfeRms {}
impl Size<WfeRms> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize
    }
}
/// Wavefront in the exit pupil \[m\]
#[derive(UID)]
#[uid(data = "Vec<f32>")]
pub enum Wavefront {}
impl Size<Wavefront> for OpticalModel {
    fn len(&self) -> usize {
        let n = self.src.pupil_sampling as usize;
        self.src.size as usize * n * n
    }
}
/// Source wavefront gradient pupil average `2x[rd]`
#[derive(UID)]
pub enum TipTilt {}
impl Size<TipTilt> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize * 2
    }
}
/// Source segment wavefront piston and standard deviation `([m],[m])x7`
#[derive(UID)]
pub enum SegmentWfe {}
impl Size<SegmentWfe> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize * 7 * 2
    }
}
/// Source segment wavefront error RMS `7x[m]`
#[derive(UID)]
pub enum SegmentWfeRms {}
impl Size<SegmentWfeRms> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize * 7
    }
}
/// Source segment piston `7x[m]`
#[derive(UID)]
pub enum SegmentPiston {}
impl Size<SegmentPiston> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize * 7
    }
}
/// Source segment tip-tilt `[7x[rd],7x[rd]]`
#[derive(UID)]
pub enum SegmentGradients {}
impl Size<SegmentGradients> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize * 14
    }
}
#[derive(UID)]
pub enum SegmentTipTilt {}
impl Size<SegmentTipTilt> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize * 14
    }
}
/// Source PSSn
#[derive(UID)]
pub enum PSSn {}
impl Size<PSSn> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize
    }
}
/// Source PSSn and FWHM
#[derive(UID)]
pub enum PSSnFwhm {}
impl Size<PSSnFwhm> for OpticalModel {
    fn len(&self) -> usize {
        self.src.size as usize * 2
    }
}
 */
