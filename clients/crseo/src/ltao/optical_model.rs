use crate::ltao::SensorProperty;
use crate::{NoSensor, OpticalModelBuilder};
use crseo::{FromBuilder, Gmt, Imaging, Propagation, Source};
use gmt_dos_clients_io::{
    gmt_m1::{segment::RBM, M1RigidBodyMotions},
    gmt_m2::asm::{segment::AsmCommand, M2ASMAsmCommand},
    optics::{Dev, Frame, Host},
};
use interface::{Data, Read, Update, Write};

pub mod builder;
pub mod no_sensor;
mod stats;
pub mod wave_sensor;

#[derive(Debug, thiserror::Error)]
pub enum OpticalModelError {
    #[error("failed to build optical model")]
    Crseo(#[from] crseo::error::CrseoError),
}

pub type Result<T> = std::result::Result<T, OpticalModelError>;

pub struct OpticalModel<T = NoSensor> {
    pub(crate) gmt: Gmt,
    pub src: Source,
    pub(crate) sensor: Option<T>,
}

impl<T> OpticalModel<T> {
    pub fn sensor(&mut self) -> Option<&mut T> {
        self.sensor.as_mut()
    }
}
unsafe impl<T> Send for OpticalModel<T> {}
unsafe impl<T> Sync for OpticalModel<T> {}

impl<T> OpticalModel<T>
where
    T: FromBuilder,
{
    pub fn builder() -> OpticalModelBuilder<<T as FromBuilder>::ComponentBuilder> {
        Default::default()
    }
}
impl<T: Propagation> Update for OpticalModel<T> {
    fn update(&mut self) {
        self.src.through(&mut self.gmt).xpupil();
        if let Some(sensor) = &mut self.sensor {
            self.src.through(sensor);
        }
    }
}

impl Write<Frame<Dev>> for OpticalModel<Imaging> {
    fn write(&mut self) -> Option<Data<Frame<Dev>>> {
        self.sensor.as_mut().map(|imgr| {
            let frame = imgr.frame().clone();
            imgr.reset();
            Data::new(frame)
        })
    }
}

impl Write<Frame<Host>> for OpticalModel<Imaging> {
    fn write(&mut self) -> Option<Data<Frame<Host>>> {
        self.sensor.as_mut().map(|imgr| {
            {
                let frame = Vec::<f32>::from(&mut imgr.frame());
                imgr.reset();
                frame
            }
            .into()
        })
    }
}

impl<T: SensorProperty, const SID: u8> Read<RBM<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<RBM<SID>>) {
        self.gmt
            .m1_segment_state(SID as i32, &data[..3], &data[3..]);
    }
}

impl<T: SensorProperty> Read<M1RigidBodyMotions> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        data.chunks(6).enumerate().for_each(|(sid, data)| {
            self.gmt
                .m1_segment_state(1 + sid as i32, &data[..3], &data[3..]);
        });
    }
}
impl<T: SensorProperty, const SID: u8> Read<AsmCommand<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<AsmCommand<SID>>) {
        self.gmt.m2_segment_modes(SID, &data);
    }
}

impl<T: SensorProperty> Read<M2ASMAsmCommand> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2ASMAsmCommand>) {
        self.gmt.m2_modes(&data);
    }
}
