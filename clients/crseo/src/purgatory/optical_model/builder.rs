use crseo::{Builder, GmtBuilder, PSSnEstimates, SourceBuilder, WavefrontSensorBuilder};
use gmt_dos_clients::interface::Size;
use gmt_dos_clients_domeseeing::DomeSeeing;
use gmt_dos_clients_io::domeseeing::DomeSeeingOpd;
use serde::{Deserialize, Serialize};

use crate::{
    OpticalModel, OpticalModelOptions, PSSnOptions, Result, SensorBuilder, ShackHartmannOptions,
};

use super::SensorFn;

/// GmtBuilder optical model builder
#[derive(Debug, Serialize, Deserialize)]
pub struct OpticalModelBuilder {
    gmt: GmtBuilder,
    src: SourceBuilder,
    options: Option<Vec<OpticalModelOptions>>,
    #[serde(skip)]
    sensor_fn: SensorFn,
}
impl Default for OpticalModelBuilder {
    fn default() -> Self {
        Self {
            gmt: GmtBuilder::default(),
            src: SourceBuilder::default(),
            options: None,
            sensor_fn: SensorFn::None,
        }
    }
}

impl OpticalModelBuilder {
    /// Creates a new GmtBuilder optical model
    ///
    /// Creates a default builder based on the default parameters for [GmtBuilder] and [SourceBuilder]
    pub fn new() -> Self {
        Default::default()
    }
    /// Sets the GmtBuilder builder
    pub fn gmt(self, gmt: GmtBuilder) -> Self {
        Self { gmt, ..self }
    }
    /// Sets the `Source` builder
    pub fn source(self, src: SourceBuilder) -> Self {
        Self { src, ..self }
    }
    /// Sets [OpticalModel] [options](OpticalModelOptions)
    pub fn options(self, options: Vec<OpticalModelOptions>) -> Self {
        Self {
            options: Some(options),
            ..self
        }
    }
    /// Sets the sensor data transform operator
    pub fn sensor_fn<T: Into<SensorFn>>(mut self, operator: T) -> Self {
        self.sensor_fn = operator.into();
        self
    }
    /// Builds a new GmtBuilder optical model
    ///
    /// If there is `Some` sensor, it is initialized.

    pub fn build(self) -> Result<OpticalModel> {
        let gmt = self.gmt.clone().build()?;
        let src = self.src.clone().build()?;
        let mut optical_model = OpticalModel {
            gmt,
            src,
            sensor: None,
            segment_wise_sensor: None,
            atm: None,
            dome_seeing: None,
            static_aberration: None,
            pssn: None,
            sensor_fn: self.sensor_fn,
            frame: None,
            tau: 0f64,
        };
        if let Some(options) = self.options {
            options.into_iter().for_each(|option| match option {
                OpticalModelOptions::PSSn(PSSnOptions::Telescope(pssn_builder)) => {
                    optical_model.pssn = pssn_builder
                        .source(self.src.clone())
                        .build()
                        .ok()
                        .map(|x| Box::new(x) as Box<dyn PSSnEstimates>);
                }
                OpticalModelOptions::PSSn(PSSnOptions::AtmosphereTelescope(pssn_builder)) => {
                    optical_model.pssn = pssn_builder
                        .source(self.src.clone())
                        .build()
                        .ok()
                        .map(|x| Box::new(x) as Box<dyn PSSnEstimates>);
                }
                OpticalModelOptions::Atmosphere { builder, time_step } => {
                    optical_model.atm = builder.build().ok();
                    optical_model.tau = time_step;
                }
                OpticalModelOptions::ShackHartmann {
                    options,
                    flux_threshold,
                } => match options {
                    ShackHartmannOptions::Diffractive(sensor_builder) => {
                        optical_model.src = sensor_builder
                            .guide_stars(Some(self.src.clone()))
                            .build()
                            .unwrap();
                        optical_model.sensor = SensorBuilder::build(
                            sensor_builder,
                            self.gmt.clone(),
                            self.src.clone(),
                            flux_threshold,
                        )
                        .ok();
                    }
                    ShackHartmannOptions::Geometric(sensor_builder) => {
                        optical_model.src = sensor_builder
                            .guide_stars(Some(self.src.clone()))
                            .build()
                            .unwrap();
                        optical_model.sensor = SensorBuilder::build(
                            sensor_builder,
                            self.gmt.clone(),
                            self.src.clone(),
                            flux_threshold,
                        )
                        .ok();
                    }
                },
                OpticalModelOptions::DomeSeeing {
                    cfd_case,
                    upsampling_rate,
                } => {
                    optical_model.dome_seeing =
                        DomeSeeing::new(cfd_case, upsampling_rate, None).ok();
                }
                OpticalModelOptions::StaticAberration(phase) => {
                    optical_model.static_aberration = Some(phase.into());
                }
                OpticalModelOptions::SegmentWiseSensor(sensor_builder) => {
                    optical_model.src = sensor_builder
                        .guide_stars(Some(self.src.clone()))
                        .build()
                        .unwrap();
                    optical_model.segment_wise_sensor = SensorBuilder::build(
                        sensor_builder,
                        self.gmt.clone(),
                        self.src.clone(),
                        0f64,
                    )
                    .map(|mut sensor| {
                        (*sensor).reset();
                        sensor
                    })
                    .ok();
                }
            });
        }
        if let Some(dome_seeing) = optical_model.dome_seeing.as_ref() {
            let n_ds = <DomeSeeing as Size<DomeSeeingOpd>>::len(dome_seeing);
            let n_src = optical_model.src.pupil_sampling.pow(2) as usize;
            assert_eq!(
                n_ds, n_src,
                "the sizes of dome seeing and source wavefront do not match, {n_ds} versus {n_src}"
            );
        }
        if let Some(static_aberration) = optical_model.static_aberration.as_ref() {
            let n_sa = static_aberration.size();
            let n_src = optical_model.src.pupil_sampling.pow(2) as usize;
            assert_eq!(
                n_sa, n_src,
                "the sizes of static aberration and source wavefront do not match, {n_sa} versus {n_src}"
            );
        }
        Ok(optical_model)
    }
}
