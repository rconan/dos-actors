use std::{cell::RefCell, rc::Rc, sync::Arc};

use crseo::{
    wavefrontsensor::PhaseSensor, Atmosphere, CrseoError, FromBuilder, Gmt, SegmentWiseSensor,
    Source,
};
use gmt_dos_clients_domeseeing::{DomeSeeing, DomeSeeingError};
use gmt_dos_clients_io::optics::SegmentD7Piston;
use interface::{select::Selector, Data, TimerMarker, UniqueIdentifier, Units, Update, Write};

use crate::{DetectorFrame, OpticalModelBuilder};

mod m1;
mod m2;
mod stats;

#[non_exhaustive]
#[derive(Debug, thiserror::Error)]
pub enum OpticalModelError {
    #[error(transparent)]
    CRSEO(#[from] CrseoError),
    #[error(transparent)]
    DomeSeeing(#[from] DomeSeeingError),
}

/// GMT optical model
///
/// ```no_run
/// use gmt_dos_clients_crseo::OpticalModel;
/// use crseo::wavefrontsensor::PhaseSensor;
/// let optical_model_builder = OpticalModel::<PhaseSensor>::builder().build()?;
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub struct OpticalModel<T = PhaseSensor> {
    pub gmt: Gmt,
    pub src: Rc<RefCell<Source>>,
    pub atm: Option<Atmosphere>,
    pub dome_seeing: Option<DomeSeeing>,
    pub tau: f64,
    pub piston: Option<Arc<Vec<f64>>>,
    pub sensor: Option<T>,
}

impl Default for OpticalModel<PhaseSensor> {
    fn default() -> Self {
        <OpticalModelBuilder as Default>::default().build().unwrap()
    }
}

unsafe impl<T> Send for OpticalModel<T> {}
unsafe impl<T> Sync for OpticalModel<T> {}

impl<T> OpticalModel<T>
where
    T: FromBuilder,
    T::ComponentBuilder: Default,
{
    /// Return the [OpticalModelBuilder]
    pub fn builder() -> OpticalModelBuilder<T::ComponentBuilder> {
        Default::default()
    }
}

impl<T> Units for OpticalModel<T> {}
impl<T> Selector for OpticalModel<T> {}

impl<T> TimerMarker for OpticalModel<T> {}
impl<T> Update for OpticalModel<T>
where
    T: SegmentWiseSensor,
    OpticalModel<T>: Send + Sync,
{
    fn update(&mut self) {
        self.src.borrow_mut().through(&mut self.gmt).xpupil();
        if let Some(atm) = &mut self.atm {
            atm.secs += self.tau;
            self.src.borrow_mut().through(atm);
        }
        if let Some(dome_seeing) = &mut self.dome_seeing {
            self.src
                .borrow_mut()
                .add_same(dome_seeing.next().unwrap().as_slice());
        }
        if let Some(piston) = self.piston.as_deref() {
            self.src.borrow_mut().add_piston(piston.as_slice());
        }
        if let Some(sensor) = &mut self.sensor {
            sensor.propagate(&mut *self.src.borrow_mut())
        }
    }
}

// impl<T> Write<GuideStar> for OpticalModel<T> {
//     fn write(&mut self) -> Option<Data<GuideStar>> {
//         Some(Data::new(self.src.clone()))
//     }
// }

impl<T> Write<DetectorFrame> for OpticalModel<T>
where
    T: SegmentWiseSensor,
    DetectorFrame: UniqueIdentifier<DataType = crseo::Frame>,
{
    fn write(&mut self) -> Option<Data<DetectorFrame>> {
        self.sensor.as_mut().map(|sensor| {
            let frame = SegmentWiseSensor::frame(sensor);
            <T as crseo::WavefrontSensor>::reset(sensor);
            Data::new(frame)
        })
    }
}

impl<T: SegmentWiseSensor, const E: i32> Write<SegmentD7Piston<E>> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<SegmentD7Piston<E>>> {
        let data = self.src.borrow_mut().segment_wfe();
        let p7 = data[6].0;
        // let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, _)| (p - p7) * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

// #[derive(interface::UID)]
// #[uid(data = (Vec<f32>,Vec<bool>))]
pub enum GmtWavefront {}
impl UniqueIdentifier for GmtWavefront {
    type DataType = (Vec<f32>, Vec<bool>);
}

impl<T: SegmentWiseSensor> Write<GmtWavefront> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<GmtWavefront>> {
        let src = &mut *self.src.borrow_mut();
        let amplitude: Vec<_> = src.amplitude().into_iter().map(|a| a > 0.).collect();
        let phase = src.phase();
        let phase: Vec<_> = amplitude
            .iter()
            .zip(phase)
            .filter(|(&a, _)| a)
            .map(|(_, &p)| p)
            .collect();
        Some(Data::new((phase, amplitude)))
    }
}
