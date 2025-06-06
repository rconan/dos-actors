use crate::{
    sensors::{Camera, DispersedFringeSensor, NoSensor, SensorPropagation},
    OpticalModel,
};
use crseo::{Imaging, Pyramid};
use gmt_dos_clients_io::optics::{
    SegmentPiston, SegmentTipTilt, SegmentWfe, SegmentWfeRms, TipTilt, Wavefront, WfeRms,
};
use interface::{Data, Size, Write};

impl<T: SensorPropagation, const E: i32> Size<WfeRms<E>> for OpticalModel<T> {
    fn len(&self) -> usize {
        self.src.size as usize
    }
}

impl<T: SensorPropagation + SourceWavefront, const E: i32> Write<WfeRms<E>> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<WfeRms<E>>> {
        Some(
            match E {
                0 => self.src.wfe_rms(),
                exp => self.src.wfe_rms_10e(exp),
            }
            .into(),
        )
    }
}

impl<T: SensorPropagation, const E: i32> Size<SegmentWfeRms<E>> for OpticalModel<T> {
    fn len(&self) -> usize {
        (self.src.size as usize) * 7
    }
}
impl<T: SensorPropagation, const E: i32> Write<SegmentWfeRms<E>> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<SegmentWfeRms<E>>> {
        Some(
            match E {
                0 => self.src.segment_wfe_rms(),
                exp => self.src.segment_wfe_rms_10e(exp),
            }
            .into(),
        )
    }
}

impl<T: SensorPropagation + SourceWavefront, const E: i32> Size<SegmentPiston<E>>
    for OpticalModel<T>
{
    fn len(&self) -> usize {
        (self.src.size as usize) * 7
    }
}
impl<T: SensorPropagation + SourceWavefront, const E: i32> Write<SegmentPiston<E>>
    for OpticalModel<T>
{
    fn write(&mut self) -> Option<Data<SegmentPiston<E>>> {
        Some(
            match E {
                0 => self.src.segment_piston(),
                exp => self.src.segment_piston_10e(exp),
            }
            .into(),
        )
    }
}
// impl<T: SensorPropagation> Read<SegmentPiston> for OpticalModel<T> {
//     fn read(&mut self, data: Data<SegmentPiston>) {
//         self.piston = Some(data.into_arc());
//     }
// }

impl<T: SensorPropagation> Size<SegmentWfe> for OpticalModel<T> {
    fn len(&self) -> usize {
        (self.src.size as usize) * 7
    }
}
impl<T: SensorPropagation> Write<SegmentWfe> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<SegmentWfe>> {
        Some(Data::new(self.src.segment_wfe()))
    }
}

impl<T: SensorPropagation + SourceWavefront> Size<SegmentTipTilt> for OpticalModel<T> {
    fn len(&self) -> usize {
        (self.src.size as usize) * 7 * 2
    }
}
impl<T: SensorPropagation + SourceWavefront> Write<SegmentTipTilt> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<SegmentTipTilt>> {
        Some(Data::new(self.src.segment_gradients()))
    }
}
impl<T: SensorPropagation + SourceWavefront> Size<TipTilt> for OpticalModel<T> {
    fn len(&self) -> usize {
        (self.src.size as usize) * 2
    }
}
impl<T: SensorPropagation + SourceWavefront> Write<TipTilt> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<TipTilt>> {
        Some(Data::new(self.src.gradients()))
    }
}

pub trait SourceWavefront {}
impl SourceWavefront for NoSensor {}
impl SourceWavefront for Imaging {}
impl<const I: usize> SourceWavefront for Camera<I> {}
impl SourceWavefront for Pyramid {}
impl<const C: usize, const F: usize> SourceWavefront for DispersedFringeSensor<C, F> {}

impl<T: SensorPropagation + SourceWavefront> Size<Wavefront> for OpticalModel<T> {
    fn len(&self) -> usize {
        self.src.pupil_sampling().pow(2) * self.src.size as usize
    }
}
impl<T: SensorPropagation + SourceWavefront> Write<Wavefront> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<Wavefront>> {
        Some(Data::new(
            self.src.phase().into_iter().map(|x| *x as f64).collect(),
        ))
    }
}
