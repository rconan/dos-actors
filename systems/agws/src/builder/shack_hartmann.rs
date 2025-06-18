use gmt_dos_clients_crseo::{
    crseo::{builders::SourceBuilder, FromBuilder, Source},
    sensors::{builders::CameraBuilder, Camera},
    OpticalModel, OpticalModelBuilder, OpticalModelError,
};
use skyangle::Conversion;

use crate::kernels::{Kernel, KernelError, KernelSpecs};

use super::AgwsShackHartmann;

/// AGWS guide star [builders](gmt_dos_clients_crseo::crseo::builders::SourceBuilder)
pub struct AgwsGuideStar;
impl AgwsGuideStar {
    /// SH48 guide stars builder
    ///
    /// SH48 uses 3 guide stars (@750nm) at the following coordinates wrt. to the pointing axis:
    /// (6',15°), (5',145°) and (7',255°)
    pub fn sh48() -> SourceBuilder {
        Source::builder().band("VIS").size(3).zenith_azimuth(
            vec![6f32.from_arcmin(), 5f32.from_arcmin(), 7f32.from_arcmin()],
            vec![15f32.to_radians(), 145f32.to_radians(), 255f32.to_radians()],
        )
    }
    /// SH24 guide stars builder
    ///
    /// SH24 uses 1 guide star (@750nm) at the following coordinate wrt. to the pointing axis: (4',180°)
    pub fn sh24() -> SourceBuilder {
        Source::builder()
            .band("VIS")
            .zenith_azimuth(vec![4f32.from_arcmin()], vec![180f32.to_radians()])
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ShackHartmannBuilderError {
    #[error("missing Shack-Hartmann reconstructor")]
    Reconstructor,
    #[error("failed to build Kernel from ShackHartmannBuilder")]
    Kernel(#[from] KernelError),
}

/// AGWS Shack-Hartmann wavefront sensor builder
#[derive(Debug, Default, Clone)]
pub struct ShackHartmannBuilder<R: Default, const I: usize = 1> {
    sh: CameraBuilder<I>,
    pub(crate) src: SourceBuilder,
    recon: Option<R>,
    calibration_src_fwhm: Option<f64>,
    use_calibration_src: bool,
}
impl<R: Default, const I: usize> ShackHartmannBuilder<R, I> {
    /// Creates a new default instance
    pub fn new() -> Self {
        Default::default()
    }
    /// Returns the AGWS SH24 builder
    pub fn sh24() -> Self {
        Self::default()
            .sensor(AgwsShackHartmann::sh24())
            .source(AgwsGuideStar::sh24())
            .calibration_src_fwhm(12.)
    }
    /// Returns the AGWS SH48 builder
    pub fn sh48() -> Self {
        Self::default()
            .sensor(AgwsShackHartmann::sh48())
            .source(AgwsGuideStar::sh48())
            .calibration_src_fwhm(6.)
    }
    /// Sets the camera builder
    pub fn sensor(mut self, sh: CameraBuilder<I>) -> Self {
        self.sh = sh;
        self
    }
    /// Sets the source builder
    pub fn source(mut self, src: SourceBuilder) -> Self {
        self.src = src;
        self
    }
    /// Sets the sensor reconstructor
    pub fn reconstructor(mut self, recon: R) -> Self {
        self.recon = Some(recon);
        self
    }
    /// Sets the source image FWHM
    pub fn calibration_src_fwhm(mut self, fwhm: f64) -> Self {
        self.calibration_src_fwhm = Some(fwhm);
        self
    }
    /// Uses the calibration source properties
    /// when converting to [OpticalModelBuilder]
    pub fn use_calibration_src(mut self) -> Self {
        self.use_calibration_src = true;
        self
    }
}
impl<R: Default, const I: usize> From<ShackHartmannBuilder<R, I>>
    for OpticalModelBuilder<CameraBuilder<I>>
{
    fn from(value: ShackHartmannBuilder<R, I>) -> Self {
        OpticalModel::<Camera<I>>::builder()
            .sensor(value.sh)
            .source(if value.use_calibration_src {
                value
                    .src
                    .fwhm(value.calibration_src_fwhm.unwrap_or_default())
            } else {
                value.src
            })
    }
}

impl<R: Default, const I: usize> TryFrom<ShackHartmannBuilder<R, I>> for OpticalModel<Camera<I>> {
    type Error = OpticalModelError;

    fn try_from(value: ShackHartmannBuilder<R, I>) -> Result<Self, Self::Error> {
        OpticalModelBuilder::<CameraBuilder<I>>::from(value).build()
    }
}
impl<R: Clone + Default, T, const I: usize> TryFrom<&ShackHartmannBuilder<R, I>> for Kernel<T>
where
    T: KernelSpecs<Sensor = Camera<I>, Estimator = R>,
{
    type Error = ShackHartmannBuilderError;

    fn try_from(value: &ShackHartmannBuilder<R, I>) -> Result<Self, Self::Error> {
        let ShackHartmannBuilder {
            sh,
            mut src,
            recon,
            calibration_src_fwhm,
            ..
        } = value.clone();
        if let Some(fwhm) = calibration_src_fwhm {
            src = src.fwhm(fwhm);
        }
        let model = OpticalModel::<Camera<I>>::builder().sensor(sh).source(src);
        if let Some(estimator) = recon {
            Ok(Kernel::new(&model)?.estimator(estimator.clone()))
        } else {
            Ok(Kernel::new(&model)?)
        }
    }
}
