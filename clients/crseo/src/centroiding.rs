use crate::sensors::CameraBuilder;
use crate::{DeviceInitialize, OpticalModel, OpticalModelBuilder};
use crseo::centroiding::CentroidingBuilder;
use crseo::imaging::ImagingBuilder;
use crseo::{Builder, Centroiding, Imaging};
use gmt_dos_clients_io::optics::{Dev, Frame, SensorData};
use interface::{Data, Read, Update, Write};
use std::marker::PhantomData;
use std::sync::Arc;

#[derive(Debug, thiserror::Error)]
pub enum CentroidsError {
    #[error("failed to build optical model")]
    Crseo(#[from] crseo::error::CrseoError),
}

pub type Result<T> = std::result::Result<T, CentroidsError>;

pub struct Full;
pub struct ZeroMean;
pub trait CentroidKind {}
impl CentroidKind for Full {}
impl CentroidKind for ZeroMean {}

pub struct Centroids<K = Full>
where
    K: CentroidKind,
{
    pub(crate) reference: Centroiding,
    pub(crate) centroids: Centroiding,
    frame: Option<Arc<crseo::imaging::Frame>>,
    kind: PhantomData<K>,
}

unsafe impl<K: CentroidKind> Send for Centroids<K> {}
unsafe impl<K: CentroidKind> Sync for Centroids<K> {}

impl<K: CentroidKind> TryFrom<&ImagingBuilder> for Centroids<K> {
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

impl<K: CentroidKind, const I: usize> TryFrom<&CameraBuilder<I>> for Centroids<K> {
    type Error = CentroidsError;

    fn try_from(camera: &CameraBuilder<I>) -> Result<Self> {
        Self::try_from(&camera.0)
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

impl<K: CentroidKind, const I: usize> DeviceInitialize<Centroids<K>>
    for OpticalModelBuilder<CameraBuilder<I>>
{
    fn initialize(&mut self, device: &mut Centroids<K>) {
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

impl<K: CentroidKind> Centroids<K> {
    pub fn setup(&mut self, optical_model: &mut OpticalModel<Imaging>) {
        optical_model.update();
        let imgr = optical_model.sensor.as_mut().unwrap();
        self.reference.process(&mut imgr.frame(), None);
        self.reference
            .valid_lenslets(Some(imgr.fluxlet_threshold), None);
        imgr.reset();
    }
    pub fn n_valid_lenslets(&self) -> &[usize] {
        &self.reference.n_valid_lenslet
    }
}

impl<K: CentroidKind> Update for Centroids<K> {
    fn update(&mut self) {
        self.centroids
            .process(self.frame.as_ref().unwrap(), Some(&self.reference));
    }
}

impl<K: CentroidKind> Read<Frame<Dev>> for Centroids<K> {
    fn read(&mut self, data: Data<Frame<Dev>>) {
        self.frame = Some(data.into_arc())
    }
}

impl Write<SensorData> for Centroids<Full> {
    fn write(&mut self) -> Option<Data<SensorData>> {
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

impl Write<SensorData> for Centroids<ZeroMean> {
    fn write(&mut self) -> Option<Data<SensorData>> {
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
