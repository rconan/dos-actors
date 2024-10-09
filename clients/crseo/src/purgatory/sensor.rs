use crate::Result;
use crseo::{
    wavefrontsensor::{GeomShackBuilder, PhaseSensorBuilder},
    Builder, GmtBuilder, SourceBuilder, WavefrontSensor, WavefrontSensorBuilder,
};

pub trait SensorBuilder: WavefrontSensorBuilder + Builder + Clone {
    fn build(
        self,
        gmt_builder: GmtBuilder,
        src_builder: SourceBuilder,
        threshold: f64,
    ) -> Result<Box<dyn WavefrontSensor>>;
}

impl SensorBuilder for GeomShackBuilder {
    fn build(
        self,
        _gmt_builder: GmtBuilder,
        _src_builder: SourceBuilder,
        _threshold: f64,
    ) -> Result<Box<dyn WavefrontSensor>> {
        Ok(Box::new(crseo::Builder::build(self)?))
    }
}

impl SensorBuilder for PhaseSensorBuilder {
    fn build(
        self,
        _gmt_builder: GmtBuilder,
        _src_builder: SourceBuilder,
        _threshold: f64,
    ) -> Result<Box<dyn WavefrontSensor>> {
        Ok(Box::new(crseo::Builder::build(self)?))
    }
}
