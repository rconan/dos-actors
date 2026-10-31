use crseo::{
    Builder, PSSnEstimates, Pyramid, WavefrontSensorBuilder, wavefrontsensor::PyramidBuilder,
};

use crate::{
    OpticalModel, OpticalModelBuilder,
    optical_model::{OpticalModelError, builder::PSSnKind},
};

impl OpticalModelBuilder<PyramidBuilder> {
    pub fn build(self) -> Result<OpticalModel<Pyramid>, OpticalModelError> {
        super::super::builder::check_device(self.device)?;
        crseo::set_gpu(self.device);
        let src = self.sensor.as_ref().unwrap().guide_stars(Some(self.src));
        Ok(OpticalModel {
            gmt: self.gmt.build()?,
            src: src.build()?,
            atm: self.atm_builder.map(|atm| atm.build()).transpose()?,
            sensor: self.sensor.map(|sensor| sensor.build()).transpose()?,
            tau: self.sampling_frequency.map_or_else(|| 0f64, |x| x.recip()),
            phase_offset: None,
            pointing: Default::default(),
            device: self.device,
            pssn: if let Some(kind) = self.pssn {
                match kind {
                    PSSnKind::TelescopeError(pssn_builder) => {
                        Some(Box::new(pssn_builder.build()?) as Box<dyn PSSnEstimates>)
                    }
                    PSSnKind::AtmosphereTelescopeError(pssn_builder) => {
                        Some(Box::new(pssn_builder.build()?) as Box<dyn PSSnEstimates>)
                    }
                }
            } else {
                None
            },
        })
    }
}
