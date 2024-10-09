use crate::ngao::GuideStar;
use gmt_dos_clients_io::optics::{
    SegmentD7Piston, SegmentDWfe, SegmentPiston, SegmentWfe, SegmentWfeRms, WfeRms,
};
use interface::{Data, Read, Size, UniqueIdentifier, Update, Write};

/// Optical metrics derived from the wavefront map
///
/// ```no_run
/// use gmt_dos_clients_crseo::WavefrontStats;
/// let stats: WavefrontStats = Default::default();
/// ```
#[derive(Debug, Default)]
pub struct WavefrontStats<const N_SRC: usize = 1> {
    segment_wfe: Vec<(f64, f64)>,
    wfe_rms: Vec<f64>,
}

impl<const N_SRC: usize> Update for WavefrontStats<N_SRC> {}

impl<const N_SRC: usize> Read<GuideStar> for WavefrontStats<N_SRC> {
    fn read(&mut self, data: Data<GuideStar>) {
        let src = &mut (data.lock().unwrap());
        self.wfe_rms = src.wfe_rms();
        self.segment_wfe = src.segment_wfe();
    }
}

impl<const N_SRC: usize, const E: i32> Size<WfeRms<E>> for WavefrontStats<N_SRC> {
    fn len(&self) -> usize {
        N_SRC
    }
}

impl<const N_SRC: usize, const E: i32> Write<WfeRms<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<WfeRms<E>>> {
        let data = &self.wfe_rms;
        Some(
            data.into_iter()
                .map(|s| *s * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

pub enum Wfe<const E: i32 = 0> {}
impl<const E: i32> UniqueIdentifier for Wfe<E> {
    type DataType = (Vec<f64>, Vec<(f64, f64)>);
}
impl<const N_SRC: usize, const E: i32> Write<Wfe<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<Wfe<E>>> {
        let data: Vec<_> = self
            .segment_wfe
            .iter()
            .map(|(p, s)| (*p * 10_f64.powi(-E), *s * 10_f64.powi(-E)))
            .collect();
        let wfe_rms: Vec<_> = self.wfe_rms.iter().map(|x| x * 10_f64.powi(-E)).collect();
        Some(Data::new((wfe_rms, data)))
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentWfe<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentWfe<E>>> {
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, s)| (*p * 10_f64.powi(-E), *s * 10_f64.powi(-E)))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentDWfe<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentDWfe<E>>> {
        let data = &self.segment_wfe;
        let p7 = data[6].0;
        Some(
            data.into_iter()
                .map(|(p, s)| ((*p - p7) * 10_f64.powi(-E), *s * 10_f64.powi(-E)))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Size<SegmentPiston<E>> for WavefrontStats<N_SRC> {
    fn len(&self) -> usize {
        N_SRC * 7
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentPiston<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentPiston<E>>> {
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, _)| *p * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentD7Piston<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentD7Piston<E>>> {
        let data = &self.segment_wfe;
        let p7 = data[6].0;
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, _)| (*p - p7) * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<const N_SRC: usize, const E: i32> Size<SegmentWfeRms<E>> for WavefrontStats<N_SRC> {
    fn len(&self) -> usize {
        N_SRC * 7
    }
}

impl<const N_SRC: usize, const E: i32> Write<SegmentWfeRms<E>> for WavefrontStats<N_SRC> {
    fn write(&mut self) -> Option<Data<SegmentWfeRms<E>>> {
        let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(_, s)| *s * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}
