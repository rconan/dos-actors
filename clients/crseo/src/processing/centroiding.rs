//! # Centroids processing pipeline

use crate::{
    sensors::builders::CameraBuilder, DeviceInitialize, OpticalModel, OpticalModelBuilder,
};
use crseo::{
    centroiding::CentroidingBuilder, imaging::ImagingBuilder, Builder, Centroiding, Imaging,
};
use gmt_dos_clients_io::optics::{Dev, Frame};
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use std::{marker::PhantomData, sync::Arc};

#[derive(Debug, thiserror::Error)]
pub enum CentroidsError {
    #[error("failed to build optical model")]
    Crseo(#[from] crseo::error::CrseoError),
}

pub type Result<T> = std::result::Result<T, CentroidsError>;

/// Full centroids
pub struct Full;
/// Mean centroids removed
pub struct ZeroMean;
/// Centroids marker
pub trait CentroidKind {
    fn is_full() -> bool {
        true
    }
}
impl CentroidKind for Full {}
impl CentroidKind for ZeroMean {
    fn is_full() -> bool {
        false
    }
}

/// Centroids processing
///
/// Compute the centroids for a Shack-Hartmann type of sensor.
/// The generic parameter `K` allows to compute the centroids
/// with ([ZeroMean]) or without ([Full])  mean subtraction.
pub struct CentroidsProcessing<K = Full>
where
    K: CentroidKind,
{
    pub(crate) reference: Centroiding,
    pub(crate) centroids: Centroiding,
    frame: Option<Arc<crseo::imaging::Frame>>,
    kind: PhantomData<K>,
}

unsafe impl<K: CentroidKind> Send for CentroidsProcessing<K> {}
unsafe impl<K: CentroidKind> Sync for CentroidsProcessing<K> {}

impl<K: CentroidKind> TryFrom<&ImagingBuilder> for CentroidsProcessing<K> {
    type Error = CentroidsError;

    fn try_from(imgr: &ImagingBuilder) -> Result<Self> {
        Ok(Self {
            reference: CentroidingBuilder::from(imgr).build()?,
            centroids: CentroidingBuilder::from(imgr).build()?,
            frame: None,
            kind: PhantomData,
        })
    }
}

impl<K: CentroidKind, const I: usize> TryFrom<&CameraBuilder<I>> for CentroidsProcessing<K> {
    type Error = CentroidsError;

    fn try_from(camera: &CameraBuilder<I>) -> Result<Self> {
        Self::try_from(&camera.0)
    }
}

impl<K: CentroidKind, const I: usize> TryFrom<&OpticalModelBuilder<CameraBuilder<I>>>
    for CentroidsProcessing<K>
{
    type Error = CentroidsError;

    fn try_from(om: &OpticalModelBuilder<CameraBuilder<I>>) -> Result<Self> {
        Self::try_from(om.sensor.as_ref().unwrap())
    }
}

// impl DeviceInitialize for OpticalModel<Imaging> {
//     type Device = Centroids<K>;

//     fn initialize(&mut self, device: &mut Self::Device) {
//         self.update();
//         let imgr = self.sensor.as_mut().unwrap();
//         device.reference.process(&mut imgr.frame(), None);
//         device
//             .reference
//             .valid_lenslets(Some(imgr.fluxlet_threshold), None);
//         imgr.reset();
//     }
// }

// impl<K: CentroidKind, const I: usize> DeviceInitialize for OpticalModel<Camera<I>> {
//     type Device = Centroids<K>;

//     fn initialize(&mut self, device: &mut Self::Device) {
//         self.update();
//         let imgr = self.sensor.as_mut().unwrap();
//         device.reference.process(&mut imgr.frame(), None);
//         device
//             .reference
//             .valid_lenslets(Some(imgr.fluxlet_threshold), None);
//         imgr.reset();
//     }
// }

impl<K: CentroidKind, const I: usize> DeviceInitialize<CentroidsProcessing<K>>
    for OpticalModelBuilder<CameraBuilder<I>>
{
    fn initialize(&self, device: &mut CentroidsProcessing<K>) {
        let mut om = self.clone_into::<1>().build().unwrap();
        om.update();
        let imgr = om.sensor.as_mut().unwrap();
        device.reference.process(&mut imgr.frame(), None);
        device
            .reference
            .valid_lenslets(Some(imgr.fluxlet_threshold), None);
        imgr.reset();
    }
}

impl<K: CentroidKind> DeviceInitialize<CentroidsProcessing<K>>
    for OpticalModelBuilder<ImagingBuilder>
{
    fn initialize(&self, device: &mut CentroidsProcessing<K>) {
        let mut om = self.clone().build().unwrap();
        om.update();
        let imgr = om.sensor.as_mut().unwrap();
        device.reference.process(&mut imgr.frame(), None);
        device
            .reference
            .valid_lenslets(Some(imgr.fluxlet_threshold), None);
        imgr.reset();
    }
}

impl<K: CentroidKind> CentroidsProcessing<K> {
    /// Sets up centroiding for [OpticalModel]`<`[Imaging]`>`
    pub fn setup(&mut self, optical_model: &mut OpticalModel<Imaging>) {
        optical_model.update();
        let imgr = optical_model.sensor.as_mut().unwrap();
        self.reference.process(&mut imgr.frame(), None);
        self.reference
            .valid_lenslets(Some(imgr.fluxlet_threshold), None);
        imgr.reset();
    }
    /// Sets the valid lenslet mask
    pub fn set_valid_lenslets(&mut self, valid_mask: impl Into<Vec<i8>>) {
        self.reference.valid_lenslets(None, Some(valid_mask.into()));
    }
    /// Returns the valid lenslet mask
    pub fn get_valid_lenslets(&self) -> &[i8] {
        &self.reference.valid_lenslets
    }
    /// Returns the # of valid lenslets
    pub fn n_valid_lenslets(&self) -> &[usize] {
        &self.reference.n_valid_lenslet
    }
}

impl CentroidsProcessing<Full> {
    pub fn kind(&self) -> &str {
        "full"
    }
}
impl CentroidsProcessing<ZeroMean> {
    pub fn kind(&self) -> &str {
        "zero_mean"
    }
}

impl<K: CentroidKind> Update for CentroidsProcessing<K> {
    fn update(&mut self) {
        self.centroids
            .process(self.frame.as_ref().unwrap(), Some(&self.reference));
    }
}

impl<K: CentroidKind> Read<Frame<Dev>> for CentroidsProcessing<K> {
    fn read(&mut self, data: Data<Frame<Dev>>) {
        self.frame = Some(data.into_arc())
    }
}

impl<U> Write<U> for CentroidsProcessing<Full>
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(
            self.centroids
                .grab()
                .centroids
                .iter()
                .map(|x| *x as f64)
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<U> Write<U> for CentroidsProcessing<ZeroMean>
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(
            self.centroids
                .grab()
                .remove_mean(Some(&self.reference.valid_lenslets))
                .centroids
                .iter()
                .map(|x| *x as f64)
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

#[cfg(test)]
mod tests {

    use ::interface::{Update, Write};
    use crseo::{
        imaging::{ImagingBuilder, LensletArray},
        Builder, FromBuilder, Gmt, Source,
    };
    use gmt_dos_clients_io::optics::{Frame, Host};

    use crate::sensors::Camera;

    use super::*;

    #[test]
    fn imgr_flux() -> std::result::Result<(), Box<dyn std::error::Error>> {
        let n_gs = 2;
        let _flux0 = {
            let mut gmt = Gmt::builder().build()?;
            let mut src = Source::builder().size(n_gs).build()?;
            println!("{src}");

            let imgr_builder = ImagingBuilder::default().n_sensor(n_gs);

            let mut centroiding = CentroidingBuilder::from(&imgr_builder).build()?;

            let mut imgr = imgr_builder.build()?;
            println!("{imgr}");

            src.through(&mut gmt).xpupil().through(&mut imgr);

            let frame = imgr.frame();
            let f = Vec::<f32>::from(&frame);
            // serde_pickle::to_writer(&mut File::create("frame.pkl")?, &f, Default::default())?;
            dbg!(f.iter().sum::<f32>());
            centroiding.process(&frame, None).grab();

            dbg!(centroiding.lenslet_array_flux());
            dbg!(&centroiding.centroids);
        };

        let cam = Camera::<1>::builder().n_sensor(n_gs);
        let mut centroids: CentroidsProcessing = CentroidsProcessing::try_from(&cam)?;

        let mut om: OpticalModel<Camera> = OpticalModel::<Camera<1>>::builder()
            .source(
                Source::builder()
                    .size(n_gs)
                    .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs]),
            )
            .sensor(cam)
            .build()?;
        om.update();
        println!("{om}");

        <OpticalModel<Camera<1>> as Write<Frame<Dev>>>::write(&mut om).map(|data| {
            <CentroidsProcessing as Read<Frame<Dev>>>::read(&mut centroids, data);
            centroids.update();
            dbg!(centroids.centroids.lenslet_array_flux())
        });
        <OpticalModel<Camera<1>> as Write<Frame<Host>>>::write(&mut om)
            .map(|data| data.iter().sum::<f32>())
            .map(|x| dbg!(x));
        Ok(())
    }

    #[test]
    fn sh_flux() -> std::result::Result<(), Box<dyn std::error::Error>> {
        let n_gs = 3;
        let n_lenslet = 48;
        let n_px_lenslet = 32;
        let flux0 = {
            let mut gmt = Gmt::builder().build()?;
            let mut src = Source::builder()
                .size(n_gs)
                .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs])
                .pupil_sampling(n_lenslet * n_px_lenslet + 1)
                .build()?;
            println!("{src}");

            let imgr_builder = ImagingBuilder::default().n_sensor(n_gs).lenslet_array(
                LensletArray::default()
                    .n_side_lenslet(n_lenslet)
                    .n_px_lenslet(n_px_lenslet),
            );

            let mut imgr = imgr_builder.build()?;
            println!("{imgr}");

            src.through(&mut gmt).xpupil().through(&mut imgr);

            let frame = imgr.frame();
            Vec::<f32>::from(&frame).iter().sum::<f32>()
        };

        let mut om: OpticalModel<Camera> = OpticalModel::<Camera<1>>::builder()
            .source(
                Source::builder()
                    .size(n_gs)
                    .zenith_azimuth(vec![0.; n_gs], vec![0.; n_gs]),
            )
            .sensor(
                Camera::<1>::builder().n_sensor(n_gs).lenslet_array(
                    LensletArray::default()
                        .n_side_lenslet(n_lenslet)
                        .n_px_lenslet(n_px_lenslet),
                ),
            )
            .build()?;
        om.update();
        println!("{om}");
        <OpticalModel<Camera<1>> as Write<Frame<Host>>>::write(&mut om)
            .map(|data| data.iter().sum::<f32>())
            .map(|x| assert_eq!(x, flux0));

        Ok(())
    }
}
