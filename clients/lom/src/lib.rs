//! # M1 & M2 Rigid Body Motions to Linear Optical Model
//!
//! Transforms M1 and M2 rigid body motions to optical metrics
//! (tip-tilt, segment piston and  segment tip-tilt) using
//! linear optical sensitivity matrices

use std::{io::Read, path::Path, sync::Arc};

use flate2::bufread::GzDecoder;
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{M2RigidBodyMotions, asm::M2ASMReferenceBodyNodes},
    optics::{
        M1State, M2State, MaskedWavefront, SegmentD21PistonRSS, SegmentPiston, SegmentTipTilt,
        SegmentWfeRms, TipTilt, Wavefront, WfeRms,
    },
};
use gmt_lom::{LOM, LinearOpticalModelError, Loader};
use interface::{self, Data, Size, Units, Update, Write, optics::Optics};

mod optical_sensitivity;
pub use optical_sensitivity::OpticalSensitivities;

/// M1 & M2 Rigid Body Motions to Linear Optical Model
#[derive(Debug, Clone)]
pub struct LinearOpticalModel {
    lom: LOM,
    m1_rbm: Arc<Vec<f64>>,
    m2_rbm: Arc<Vec<f64>>,
}
impl LinearOpticalModel {
    pub fn new() -> std::result::Result<Self, LinearOpticalModelError> {
        let sens = include_bytes!("optical_sensitivities.rs.bin.gz");
        let mut gz = GzDecoder::new(sens.as_slice());
        let mut bytes = vec![];
        gz.read_to_end(&mut bytes)?;
        Ok(Self {
            lom: LOM::try_from(bytes.as_slice())?,
            m1_rbm: Arc::new(vec![0f64; 42]),
            m2_rbm: Arc::new(vec![0f64; 42]),
        })
    }
    pub fn new_with_path(
        path: impl AsRef<Path>,
    ) -> std::result::Result<Self, LinearOpticalModelError> {
        let path = path.as_ref();
        let filename = path.file_name().unwrap();
        let sens_loader = Loader::<gmt_lom::OpticalSensitivities>::default()
            .path(path.parent().unwrap())
            .filename(filename.to_str().unwrap());
        Ok(Self {
            lom: LOM::builder()
                .load_optical_sensitivities(sens_loader)?
                .build()?,
            m1_rbm: Arc::new(vec![0f64; 42]),
            m2_rbm: Arc::new(vec![0f64; 42]),
        })
    }
}

impl Units for LinearOpticalModel {}
impl Optics for LinearOpticalModel {}

impl Update for LinearOpticalModel {
    fn update(&mut self) {
        self.lom.rbm = vec![(self.m1_rbm.as_slice(), self.m2_rbm.as_slice())]
            .into_iter()
            .collect();
    }
}
impl interface::Read<M1RigidBodyMotions> for LinearOpticalModel {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        self.m1_rbm = data.into_arc();
    }
}
impl interface::Read<M1State> for LinearOpticalModel {
    fn read(&mut self, data: Data<M1State>) {
        self.m1_rbm = Arc::new(data.into_rbms().unwrap_or_default());
    }
}
impl interface::Read<M2RigidBodyMotions> for LinearOpticalModel {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.m2_rbm = data.into_arc();
    }
}
impl interface::Read<M2State> for LinearOpticalModel {
    fn read(&mut self, data: Data<M2State>) {
        self.m2_rbm = Arc::new(data.into_rbms().unwrap_or_default());
    }
}
impl interface::Read<M2ASMReferenceBodyNodes> for LinearOpticalModel {
    fn read(&mut self, data: Data<M2ASMReferenceBodyNodes>) {
        self.m2_rbm = data.into_arc();
    }
}

impl Write<TipTilt> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<TipTilt>> {
        Some(Data::new(self.lom.tiptilt().into()))
    }
}
impl Write<SegmentTipTilt> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<SegmentTipTilt>> {
        Some(Data::new(self.lom.segment_tiptilt().into()))
    }
}
impl<const E: i32> Write<SegmentPiston<E>> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<SegmentPiston<E>>> {
        let mut piston: Vec<f64> = self.lom.segment_piston().into();
        if E != 0 {
            piston.iter_mut().for_each(|p| {
                *p *= 10f64.powi(-E);
            });
        }
        Some(piston.into())
    }
}
impl<const E: i32> Write<SegmentWfeRms<E>> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<SegmentWfeRms<E>>> {
        Some(self.lom.segment_wfe_rms::<E>().into())
    }
}
impl<const E: i32> Write<SegmentD21PistonRSS<E>> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<SegmentD21PistonRSS<E>>> {
        let piston: Vec<f64> = self.lom.segment_piston().into();
        let mut sum_squared = 0f64;
        for p_i in piston.iter().take(6) {
            for p_j in piston.iter().skip(1) {
                sum_squared += (p_i - p_j).powi(2);
            }
        }
        let mut rss = sum_squared.sqrt();
        if E != 0 {
            rss *= 10f64.powi(-E);
        }
        Some(vec![rss].into())
    }
}

impl Write<Wavefront> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<Wavefront>> {
        Some(Data::new(self.lom.wavefront().into()))
    }
}

impl Write<MaskedWavefront> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<MaskedWavefront>> {
        Some(Data::new(self.lom.masked_wavefront().into()))
    }
}

impl<const E: i32> Write<WfeRms<E>> for LinearOpticalModel {
    fn write(&mut self) -> Option<Data<WfeRms<E>>> {
        let wavefront = self.lom.masked_wavefront();
        let n = wavefront.len() as f64;
        let (s, m) = wavefront
            .into_iter()
            .fold((0f64, 0f64), |(mut s, mut m), w| {
                s += w * w;
                m += w;
                (s, m)
            });
        let mut wfe_rms = ((s - m * m / n) / n).sqrt();
        if E != 0 {
            wfe_rms *= 10f64.powi(-E);
        }
        Some(Data::new(vec![wfe_rms]))
    }
}

impl Size<TipTilt> for LinearOpticalModel {
    fn len(&self) -> usize {
        2
    }
}
impl Size<SegmentTipTilt> for LinearOpticalModel {
    fn len(&self) -> usize {
        14
    }
}
impl<const E: i32> Size<SegmentPiston<E>> for LinearOpticalModel {
    fn len(&self) -> usize {
        7
    }
}
impl Size<Wavefront> for LinearOpticalModel {
    fn len(&self) -> usize {
        512 * 512
    }
}
impl<const E: i32> Size<WfeRms<E>> for LinearOpticalModel {
    fn len(&self) -> usize {
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn m1_segment_rxy() {
        let mut rbm2lom = LinearOpticalModel::new().unwrap();
        let s = 1e-6;
        let m1_rbm: Vec<_> = (0..7)
            .flat_map(|_| {
                let mut v = vec![0f64; 6];
                v[3] = s;
                v
            })
            .collect();
        <LinearOpticalModel as interface::Read<M1RigidBodyMotions>>::read(
            &mut rbm2lom,
            m1_rbm.into(),
        );
        rbm2lom.update();
        let mut stt: Vec<f64> = <LinearOpticalModel as Write<SegmentTipTilt>>::write(&mut rbm2lom)
            .unwrap()
            .into();
        stt.iter_mut().for_each(|x| *x /= s);
        let stt_mag: Vec<_> = stt[..7]
            .iter()
            .zip(&stt[7..])
            .map(|(x, y)| x.hypot(*y))
            .collect();
        println!("STT: {:.2?}", stt_mag);
    }

    #[test]
    fn m2_segment_rxy() {
        let mut rbm2lom = LinearOpticalModel::new().unwrap();
        let s = 1e-6;
        let m1_rbm: Vec<_> = (0..7)
            .flat_map(|_| {
                let mut v = vec![0f64; 6];
                v[4] = s;
                v
            })
            .collect();
        <LinearOpticalModel as interface::Read<M2RigidBodyMotions>>::read(
            &mut rbm2lom,
            m1_rbm.into(),
        );
        rbm2lom.update();
        let mut stt: Vec<f64> = <LinearOpticalModel as Write<SegmentTipTilt>>::write(&mut rbm2lom)
            .unwrap()
            .into();
        stt.iter_mut().for_each(|x| *x /= s);
        let stt_mag: Vec<_> = stt[..7]
            .iter()
            .zip(&stt[7..])
            .map(|(x, y)| x.hypot(*y))
            .collect();
        println!("STT: {:.2?}", stt_mag);
    }

    #[test]
    fn m1_segment_txy() {
        let mut rbm2lom = LinearOpticalModel::new().unwrap();
        let s = 1e-6;
        let m1_rbm: Vec<_> = (0..7)
            .flat_map(|_| {
                let mut v = vec![0f64; 6];
                v[0] = s;
                v
            })
            .collect();
        <LinearOpticalModel as interface::Read<M1RigidBodyMotions>>::read(
            &mut rbm2lom,
            m1_rbm.into(),
        );
        rbm2lom.update();
        let mut stt: Vec<f64> = <LinearOpticalModel as Write<SegmentTipTilt>>::write(&mut rbm2lom)
            .unwrap()
            .into();
        stt.iter_mut().for_each(|x| *x /= s);
        let stt_mag: Vec<_> = stt[..7]
            .iter()
            .zip(&stt[7..])
            .map(|(x, y)| x.hypot(*y))
            .collect();
        println!("STT: {:.3?}", stt_mag);
    }
}
