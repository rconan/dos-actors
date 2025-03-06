use gmt_dos_actors::system::{Sys, SystemError};
use gmt_dos_clients_crseo::{
    calibration::Reconstructor,
    crseo::{
        builders::SourceBuilder,
        imaging::{Detector, LensletArray},
        FromBuilder, Source,
    },
    sensors::{builders::CameraBuilder, Camera},
    OpticalModel, OpticalModelBuilder, OpticalModelError,
};
use skyangle::Conversion;

use crate::{
    agws::{sh24::Sh24, sh48::Sh48},
    kernels::{Kernel, KernelError},
    Agws,
};

#[derive(Debug, thiserror::Error)]
pub enum AgwsBuilderError {
    #[error("failed to build AGWS optical model")]
    OpticalModel(#[from] OpticalModelError),
    #[error("missing SH24 reconstructor")]
    Sh24Reconstructor,
    #[error("AGWS kernel error")]
    AgwsKernel(#[from] KernelError),
    #[error("AGWS system error")]
    AgwsSystem(#[from] SystemError),
}

/// AGWS guide star [builders](gmt_dos_clients_crseo::crseo::builders::SourceBuilder)
pub struct AgwsGuideStar;
impl AgwsGuideStar {
    /// SH48 guide stars builder
    pub fn sh48() -> SourceBuilder {
        Source::builder().size(3).zenith_azimuth(
            vec![6f32.from_arcmin(), 5f32.from_arcmin(), 7f32.from_arcmin()],
            vec![15f32.to_radians(), 135f32.to_radians(), 255f32.to_radians()],
        )
    }
    /// SH24 guide stars builder
    pub fn sh24() -> SourceBuilder {
        Source::builder().zenith_azimuth(vec![4f32.from_arcmin()], vec![180f32.to_radians()])
    }
}

/// [Agws] builder
#[derive(Debug, Clone)]
pub struct AgwsBuilder<const SH48_I: usize = 1, const SH24_I: usize = 1> {
    sh48: OpticalModelBuilder<CameraBuilder<SH48_I>>,
    sh24: OpticalModelBuilder<CameraBuilder<SH24_I>>,
    sh24_recon: Option<Reconstructor>,
}

impl<const SH48_I: usize, const SH24_I: usize> Default for AgwsBuilder<SH48_I, SH24_I> {
    fn default() -> Self {
        let sh48 = OpticalModel::<Camera<SH48_I>>::builder()
            .sensor(
                Camera::<SH48_I>::builder()
                    .n_sensor(3)
                    .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(24))
                    .detector(Detector::default().n_px_imagelet(8))
                    .lenslet_flux(0.75),
            )
            .source(AgwsGuideStar::sh48());
        let sh24 = OpticalModel::<Camera<SH24_I>>::builder()
            .sensor(
                Camera::<SH24_I>::builder()
                    .lenslet_array(LensletArray::default().n_side_lenslet(24).n_px_lenslet(24))
                    .detector(Detector::default().n_px_imagelet(16).n_px_framelet(8))
                    .lenslet_flux(0.75),
            )
            .source(AgwsGuideStar::sh24());
        Self {
            sh48,
            sh24,
            sh24_recon: None,
        }
    }
}

impl<const SH48_I: usize, const SH24_I: usize> AgwsBuilder<SH48_I, SH24_I> {
    /// Create a new builder instance
    pub fn new() -> Self {
        Default::default()
    }
    /// Returns a clone of the AGWS SH48 builder
    pub fn sh48(&self) -> OpticalModelBuilder<CameraBuilder<SH48_I>> {
        self.sh48.clone()
    }
    /// Returns a clone of the AGWS SH24 builder
    pub fn sh24(&self) -> OpticalModelBuilder<CameraBuilder<SH24_I>> {
        self.sh24.clone()
    }
    /// Sets the path to SH24 calibration file
    pub fn sh24_calibration(mut self, sh24_recon: Reconstructor) -> Self {
        self.sh24_recon = Some(sh24_recon);
        self
    }
    /// Build an [Agws] [system](gmt_dos_actors::system::Sys) instance
    pub fn build(self) -> Result<Sys<Agws<SH48_I, SH24_I>>, AgwsBuilderError> {
        let Some(sh24_recon) = self.sh24_recon else {
            return Err(AgwsBuilderError::Sh24Reconstructor);
        };
        let sh48 = self.sh48.build()?;
        log::info!("SH48:\n{}", sh48);
        let sh24_kernel = Kernel::<Sh24<SH24_I>>::new(&self.sh24, sh24_recon)?;
        let sh24 = self.sh24.build()?;
        log::info!("SH24:\n{}", sh24);
        Ok(Sys::new(Agws {
            sh48: Sh48(sh48).into(),
            sh24: Sh24(sh24).into(),
            sh24_kernel: sh24_kernel.into(),
        })
        .build()?)
    }
}
