use crseo::{
    FromBuilder, Source,
    gmt::{GmtM1, GmtM2},
};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::{
        Calib, Calibration, CalibrationMode, GlobalCalibration, MirrorMode, MixedMirrorMode,
        Reconstructor,
        algebra::{Block, CalibProps},
    },
    sensors::{NoSensor, WaveSensor, builders::WaveSensorBuilder},
};
use image::{Rgb, RgbImage, imageops};
use skyangle::Conversion;
use std::{
    fmt::Debug,
    ops::{Div, Sub},
};

const N_PX: usize = 2 * 48 - 1;
const N_GS: usize = 3;

fn main() -> anyhow::Result<()> {
    let zenith: Vec<_> = vec![7f32, 5f32, 8f32]
        .into_iter()
        .map(|x| x.from_arcmin())
        .collect();

    let azimuth: Vec<_> = vec![-10f32, 95f32, 255f32]
        .into_iter()
        .map(|x| x.to_radians())
        .collect();

    let agws_gs = Source::builder()
        .size(N_GS)
        .pupil_sampling(N_PX)
        // .on_ring(6f32.from_arcmin())
        .zenith_azimuth(zenith, azimuth);
    let omb: OpticalModelBuilder<WaveSensorBuilder> = OpticalModel::<WaveSensor>::builder()
        .source(agws_gs.clone())
        .sensor(OpticalModel::<NoSensor>::builder().source(agws_gs).into());
    println!("{}", omb.clone().build()?);

    println!(" == M1 ==");
    let rbm = MirrorMode::from(CalibrationMode::rbm(1e-6)).remove(7);
    let recon1 = <WaveSensor as Calibration<GmtM1>>::calibrate_serial(&(omb.clone().into()), rbm)?;
    // .diagonal();
    println!("{recon1}");

    let mut mask1 = vec![0u32; N_PX.pow(2) * N_GS];
    recon1.calib().for_each(|c| {
        c.mask_as_slice()
            .into_iter()
            .zip(&mut mask1)
            .for_each(|(m, m1)| {
                if *m {
                    *m1 = 1u32;
                }
            })
    });
    as_image(&mask1, (N_PX, N_PX * N_GS))
        .mask(Default::default())
        .scale(2.5)
        .build()
        .save("mask1.png")?;

    let recon1 = recon1.diagonal();
    println!("{recon1}");

    println!(" == M2 ==");
    let rbm = MirrorMode::from(CalibrationMode::rbm(1e-6)).update((
        7,
        CalibrationMode::RBM([
            Some(1e-6),
            Some(1e-6),
            Some(1e-6),
            Some(1e-6),
            Some(1e-6),
            None,
        ]),
    ));
    let mut recon2 =
        <WaveSensor as Calibration<GmtM2>>::calibrate_serial(&(omb.clone().into()), rbm)?;
    recon2.pseudoinverse();
    println!("{recon2}");

    let mut recon2 = recon2.diagonal();
    recon2.pseudoinverse();
    println!("{recon2}");

    let mut mask2 = vec![0u32; N_PX.pow(2) * N_GS];
    recon2.calib().for_each(|c| {
        c.mask_as_slice()
            .into_iter()
            .zip(&mut mask2)
            .for_each(|(m, m2)| {
                if *m {
                    *m2 = 1u32;
                }
            })
    });
    as_image(&mask2, (N_PX, N_PX * N_GS))
        .mask(Default::default())
        .scale(2.5)
        .build()
        .save("mask2.png")?;

    let mut recon3 = <WaveSensor as GlobalCalibration<GmtM2>>::calibrate(
        &omb,
        CalibrationMode::GlobalTipTilt(1e-6),
    )?;
    recon3.pseudoinverse();
    println!("{recon3}");

    let c = &recon3.calib_slice()[0];
    for i in 0..2 {
        let mut phase = vec![0f64; c.mask_as_slice().len()];
        phase
            .iter_mut()
            .zip(c.mask_as_slice())
            .filter_map(|(p, m)| m.then(|| p))
            .zip(c.mat_ref().col(i).iter())
            .for_each(|(p, v)| *p = *v);
        as_image(&phase, (N_PX, N_PX * N_GS))
            .scale(2.5)
            .build()
            .save(format!("m2_grxy_{i}.png"))?;
    }
    
    let mut recon4 = <WaveSensor as GlobalCalibration<GmtM2>>::calibrate(
        &omb,
        CalibrationMode::GlobalTxyz(1e-6),
    )?;
    recon4.pseudoinverse();
    println!("{recon4}");

    let c = &recon4.calib_slice()[0];
    for i in 0..3 {
        let mut phase = vec![0f64; c.mask_as_slice().len()];
        phase
            .iter_mut()
            .zip(c.mask_as_slice())
            .filter_map(|(p, m)| m.then(|| p))
            .zip(c.mat_ref().col(i).iter())
            .for_each(|(p, v)| *p = *v);
        as_image(&phase, (N_PX, N_PX * N_GS))
            .scale(2.5)
            .build()
            .save(format!("m2_gtxyz_{i}.png"))?;
    }
    
    let d2: Calib<MixedMirrorMode> = Calib::<MirrorMode>::from(recon2).into();
    let d3: Calib<MixedMirrorMode> = Calib::<CalibrationMode>::from(recon3).into();

    let d = Calib::<MixedMirrorMode>::block(&[&[&d2, &d3]]);
    let mut recon: Reconstructor<MixedMirrorMode> = d.into();
    recon.pseudoinverse();
    println!("{recon}");

    Ok(())
}

#[derive(Debug)]
pub struct Mask<T> {
    threshold: T,
    color: Rgb<u8>,
}
impl<T: Default> Default for Mask<T> {
    fn default() -> Self {
        Self {
            threshold: Default::default(),
            color: Rgb([255; 3]),
        }
    }
}
impl<T: Default> Mask<T> {
    pub fn new() -> Self {
        Default::default()
    }
    pub fn threshold(mut self, t: T) -> Self {
        self.threshold = t;
        self
    }
    pub fn color(mut self, c: impl Into<Rgb<u8>>) -> Self {
        self.color = c.into();
        self
    }
}
#[derive(Debug)]
pub enum ColorMap<T> {
    Mask(Mask<T>),
    Map(colorous::Gradient),
}
impl<T> Default for ColorMap<T> {
    fn default() -> Self {
        Self::Map(colorous::SPECTRAL)
    }
}
impl<T> From<colorous::Gradient> for ColorMap<T> {
    fn from(value: colorous::Gradient) -> Self {
        Self::Map(value)
    }
}

#[derive(Debug, Default)]
pub struct ImageBuilder<'a, T> {
    width: u32,
    height: u32,
    data: &'a [T],
    scale: Option<f32>,
    colormap: ColorMap<T>,
}
impl<'a, T> ImageBuilder<'a, T>
where
    T: Default + PartialOrd + Copy + Sub<Output = T> + Div<Output = T> + TryInto<f64>,
    <T as TryInto<f64>>::Error: Debug,
{
    pub fn scale(mut self, scale: f32) -> Self {
        self.scale = Some(scale);
        self
    }
    pub fn mask(mut self, mask: Mask<T>) -> Self {
        self.colormap = ColorMap::Mask(mask);
        self
    }
    pub fn colormap(mut self, colormap: impl Into<ColorMap<T>>) -> Self {
        self.colormap = colormap.into();
        self
    }
    pub fn build(self) -> image::ImageBuffer<Rgb<u8>, Vec<u8>> {
        let mut image = RgbImage::new(self.width, self.height);
        image
            .enumerate_pixels_mut()
            .zip(self.data)
            .for_each(|((_, _, px), p)| match self.colormap {
                ColorMap::Mask(Mask { threshold, color }) => {
                    if *p > threshold {
                        *px = color;
                    }
                }
                ColorMap::Map(map) => {
                    let a = self
                        .data
                        .iter()
                        .max_by(|a, b| a.partial_cmp(b).unwrap())
                        .unwrap()
                        .clone();
                    let b = self
                        .data
                        .iter()
                        .min_by(|a, b| a.partial_cmp(b).unwrap())
                        .unwrap()
                        .clone();
                    let u = (*p - b) / (a - b);
                    let color = map.eval_continuous(u.try_into().unwrap());
                    *px = Rgb([color.r, color.g, color.b])
                }
            });
        imageops::flip_vertical_in_place(&mut image);
        if let Some(scale) = self.scale {
            let nwidth = self.width as f32 * scale;
            let nheight = self.height as f32 * scale;
            imageops::resize(
                &image,
                nwidth.round() as u32,
                nheight.round() as u32,
                imageops::FilterType::Triangle,
            )
        } else {
            image
        }
    }
}

fn as_image<'a, T, U>(data: &'a [T], (width, height): (U, U)) -> ImageBuilder<'a, T>
where
    T: Default + PartialOrd,
    U: TryInto<u32>,
    <U as TryInto<u32>>::Error: Debug,
{
    ImageBuilder {
        data,
        width: width.try_into().unwrap(),
        height: height.try_into().unwrap(),
        ..Default::default()
    }
}
