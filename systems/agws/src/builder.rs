pub mod shack_hartmann;

use std::path::Path;

use gmt_dos_actors::system::{Sys, SystemError};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder, OpticalModelError,
    crseo::{
        FromBuilder, Source,
        builders::{AtmosphereBuilder, AtmosphereBuilderError, GmtBuilder},
        imaging::{Detector, LensletArray},
    },
    sensors::{
        Camera, WaveSensor,
        builders::{CameraBuilder, WaveSensorBuilder},
    },
};
use gmt_dos_clients_io::optics::{Dev, Frame};
use interface::TryRead;
use shack_hartmann::{ShackHartmannBuilder, ShackHartmannBuilderError};

use crate::{
    Agws,
    agws::{
        sh24::{Sh24, kernel::Sh24Kern},
        sh48::{Sh48, kernel::Sh48Kern},
    },
    kernels::{Kernel, KernelError, KernelFrame, KernelSpecs},
};

#[derive(Debug, thiserror::Error)]
pub enum AgwsBuilderError {
    #[error("failed to build AGWS optical model")]
    OpticalModel(#[from] OpticalModelError),
    #[error("AGWS kernel error")]
    AgwsKernel(#[from] KernelError),
    #[error("AGWS system error")]
    AgwsSystem(#[from] SystemError),
    #[error("failed load atmospheric turbulence model")]
    Atmosphere(#[from] AtmosphereBuilderError),
    #[error("ShackHartmann builder error")]
    ShackHartmann(#[from] ShackHartmannBuilderError),
}

pub struct AgwsShackHartmann;
impl AgwsShackHartmann {
    /// 48x48 Shack-Hartmann wavefront sensor
    ///
    /// The pixel scale is 0.4" and the lenslet field-of-view is 3.2"
    pub fn sh48<const I: usize>() -> CameraBuilder<I> {
        Camera::<I>::builder()
            .n_sensor(3)
            .lenslet_array(LensletArray::default().n_side_lenslet(48).n_px_lenslet(18))
            .detector(Detector::default().n_px_imagelet(24).n_px_framelet(8))
            .lenslet_flux(0.75)
    }
    /// 24x24 Shack-Hartmann wavefront sensor
    ///
    /// The pixel scale is 0.4" and the lenslet field-of-view is 4.8"
    pub fn sh24<const I: usize>() -> CameraBuilder<I> {
        Camera::<I>::builder()
            .lenslet_array(LensletArray::default().n_side_lenslet(24).n_px_lenslet(36))
            .detector(Detector::default().n_px_imagelet(72).n_px_framelet(12))
            .lenslet_flux(0.75)
    }
}

/// [Agws] builder
// #[derive(Debug)]
pub struct AgwsBuilder<
    const SH48_I: usize = 1,
    const SH24_I: usize = 1,
    K48 = Sh48<SH48_I>,
    K24 = Sh24<SH24_I>,
> where
    K48: KernelSpecs,
    K24: KernelSpecs,
{
    sh48: ShackHartmannBuilder<<K48 as KernelSpecs>::Estimator, SH48_I>,
    // sh24: OpticalModelBuilder<CameraBuilder<SH24_I>>,
    // sh24_recon: Option<Reconstructor>,
    sh24: ShackHartmannBuilder<<K24 as KernelSpecs>::Estimator, SH24_I>,
    gmt: Option<GmtBuilder>,
    atm: Option<(AtmosphereBuilder, f64)>,
}

impl<const SH48_I: usize, const SH24_I: usize, K48, K24> Default
    for AgwsBuilder<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs,
    K24: KernelSpecs,
{
    fn default() -> Self {
        Self {
            sh48: ShackHartmannBuilder::sh48(),
            sh24: ShackHartmannBuilder::sh24(),
            gmt: None,
            atm: None,
        }
    }
}

impl<const SH48_I: usize, const SH24_I: usize, K48, K24> AgwsBuilder<SH48_I, SH24_I, K48, K24>
where
    K48: KernelSpecs<Input = Frame<Dev>, Sensor = Camera<SH48_I>> + 'static + Send + Sync,
    K24: KernelSpecs<Input = Frame<Dev>, Sensor = Camera<SH24_I>> + 'static + Send + Sync,
    Sh24Kern<K24>: TryRead<KernelFrame<K24>>,
    Sh48Kern<K48>: TryRead<KernelFrame<K48>>,
{
    /// Create a new builder instance
    pub fn new() -> Self {
        Default::default()
    }
    /// Sets the GMT [builder](gmt_dos_clients_crseo::crseo::GmtBuilder)
    pub fn gmt(mut self, gmt: GmtBuilder) -> Self {
        self.gmt = Some(gmt);
        self
    }
    /// Loads the atmospheric turbulence model sampled at the given frequency in Hz
    pub fn load_atmosphere(
        mut self,
        path: impl AsRef<Path>,
        sampling_frequency: f64,
    ) -> Result<Self, AgwsBuilderError> {
        self.atm = Some((AtmosphereBuilder::load(path)?, sampling_frequency));
        Ok(self)
    }
    /// Sets the AGWS SH48 builder
    pub fn sh48(
        mut self,
        sh48: ShackHartmannBuilder<<K48 as KernelSpecs>::Estimator, SH48_I>,
    ) -> Self {
        self.sh48 = sh48;
        self
    }
    /// Sets the AGWS SH24 builder
    pub fn sh24(
        mut self,
        sh24: ShackHartmannBuilder<<K24 as KernelSpecs>::Estimator, SH24_I>,
    ) -> Self {
        self.sh24 = sh24;
        self
    }
    /// Sets the reconstructor for AGWS SH24
    pub fn sh24_calibration(mut self, sh24_recon: <K24 as KernelSpecs>::Estimator) -> Self {
        self.sh24 = self.sh24.reconstructor(sh24_recon);
        self
    }
    /// Sets the reconstructor for AGWS SH48
    pub fn sh48_calibration(mut self, sh48_recon: <K48 as KernelSpecs>::Estimator) -> Self {
        self.sh48 = self.sh48.reconstructor(sh48_recon);
        self
    }
    /// Returns a single [OpticalModelBuilder](https://docs.rs/gmt_dos-clients_crseo/latest/gmt_dos_clients_crseo/struct.OpticalModelBuilder.html) using as many [WaveSensor](https://docs.rs/gmt_dos-clients_crseo/latest/gmt_dos_clients_crseo/sensors/struct.WaveSensor.html) sensors as AGWS guide stars
    pub fn wave_sensor(&self) -> OpticalModelBuilder<WaveSensorBuilder> {
        let zenith: Vec<_> = self
            .sh24
            .src
            .zenith
            .iter()
            .chain(&self.sh48.src.zenith)
            .cloned()
            .collect();
        let azimuth: Vec<_> = self
            .sh24
            .src
            .azimuth
            .iter()
            .chain(&self.sh48.src.azimuth)
            .cloned()
            .collect();
        let src = Source::builder()
            .size(zenith.len())
            .zenith_azimuth(zenith, azimuth);
        OpticalModel::<WaveSensor>::builder()
            .gmt(self.gmt.clone().unwrap_or_default())
            .source(src.clone())
            .sensor(
                WaveSensor::builder()
                    .gmt(self.gmt.clone().unwrap_or_default())
                    .source(src),
            )
    }
    /// Build an [Agws] [system](gmt_dos_actors::system::Sys) instance
    pub fn build(self) -> Result<Sys<Agws<SH48_I, SH24_I, K48, K24>>, AgwsBuilderError> {
        let (sh24_label, sh48_label) = if self.atm.is_none() {
            (
                format!("GMT Optics\nw/ SH24<{}>", SH24_I),
                format!("GMT Optics\nw/ {} SH48<{}>", self.sh48.src.size, SH48_I),
            )
        } else {
            (
                format!("GMT Optics & Atmosphere\nw/ SH24<{}>", SH24_I),
                format!(
                    "GMT Optics & Atmosphere\nw/ {} SH48<{}>",
                    self.sh48.src.size, SH48_I
                ),
            )
        };
        let mut sh48 = OpticalModelBuilder::from(&self.sh48);
        let mut sh24 = OpticalModelBuilder::from(&self.sh24);
        if let Some((atm, sampling_frequency)) = self.atm {
            sh48 = sh48
                .atmosphere(atm.clone())
                .sampling_frequency(sampling_frequency);
            sh24 = sh24
                .atmosphere(atm.clone())
                .sampling_frequency(sampling_frequency);
        }
        if let Some(gmt) = self.gmt {
            sh48 = sh48.gmt(gmt.clone());
            sh24 = sh24.gmt(gmt);
        }
        let sh48 = sh48.build()?;
        log::info!("SH48:\n{}", sh48);
        let sh24 = sh24.build()?;
        log::info!("SH24:\n{}", sh24);

        let sh24_kern = Kernel::<K24>::try_from(self.sh24)
            .map_err(|_e| ShackHartmannBuilderError::Reconstructor)?;
        let sh48_kern = Kernel::<K48>::try_from(self.sh48)
            .map_err(|_e| ShackHartmannBuilderError::Reconstructor)?;

        Ok(Sys::new(Agws {
            sh24: (Sh24(sh24), sh24_label).into(),
            sh48: (Sh48(sh48), sh48_label).into(),
            sh24_kernel: Sh24Kern(sh24_kern).into(),
            sh48_kernel: Sh48Kern(sh48_kern).into(),
        })
        .build()?)
    }
}
