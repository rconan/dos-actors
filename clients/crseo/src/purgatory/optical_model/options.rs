use crseo::{
    pssn::{AtmosphereTelescopeError, TelescopeError},
    wavefrontsensor::PhaseSensorBuilder,
    AtmosphereBuilder, Diffractive, Geometric, PSSnBuilder, SegmentWiseSensorBuilder,
    ShackHartmannBuilder,
};
use serde::{Deserialize, Serialize};

/// Shack-Hartmann wavefront sensor type: [Diffractive] or [Geometric]
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
#[serde(tag = "shackhartmann", content = "shackhartmann-args")]
pub enum ShackHartmannOptions {
    Diffractive(ShackHartmannBuilder<Diffractive>),
    Geometric(ShackHartmannBuilder<Geometric>),
}
/// PSSn model
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
#[serde(tag = "pssn", content = "pssn-args")]
pub enum PSSnOptions {
    Telescope(PSSnBuilder<TelescopeError>),
    AtmosphereTelescope(PSSnBuilder<AtmosphereTelescopeError>),
}

/// Options for [OpticalModelBuilder]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum OpticalModelOptions<T = PhaseSensorBuilder>
where
    T: SegmentWiseSensorBuilder,
{
    Atmosphere {
        builder: AtmosphereBuilder,
        time_step: f64,
    },
    ShackHartmann {
        options: ShackHartmannOptions,
        flux_threshold: f64,
    },
    SegmentWiseSensor(T),
    DomeSeeing {
        cfd_case: String,
        upsampling_rate: usize,
    },
    StaticAberration(Vec<f32>),
    PSSn(PSSnOptions),
}
