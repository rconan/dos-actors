use gmt_dos_actors::{prelude::Actor, ArcMutex};
use gmt_dos_clients_fem::{solvers::ExponentialMatrix, DiscreteModalSolver, StateSpaceError};
use gmt_dos_clients_io::gmt_fem::{
    inputs::MCM2SmHexF,
    outputs::{MCM2Lcl6D, MCM2SmHexD, OSSM1Lcl},
};
use gmt_dos_clients_m2_ctrl::AsmsPositioners;
use gmt_dos_clients_mount::Mount;
use gmt_dos_systems_m1::Calibration;

use crate::servos::GmtServoMechanisms;

pub mod asms_servo;
pub use asms_servo::AsmsServo;
mod wind_loads;
pub use wind_loads::WindLoads;
mod edge_sensors;
pub use edge_sensors::EdgeSensors;
mod m1_segment_figure;
pub use m1_segment_figure::M1SegmentFigure;

/// [GmtServoMechanisms](crate::GmtServoMechanisms) builder
#[derive(Debug, Clone, Default)]
pub struct ServosBuilder<const M1_RATE: usize, const M2_RATE: usize> {
    pub(crate) sim_sampling_frequency: f64,
    pub(crate) fem: gmt_fem::FEM,
    pub(crate) asms_servo: Option<AsmsServo>,
    pub(crate) wind_loads: Option<WindLoads>,
    pub(crate) edge_sensors: Option<EdgeSensors>,
    pub(crate) m1_segment_figure: Option<M1SegmentFigure>,
}

impl<const M1_RATE: usize, const M2_RATE: usize> ServosBuilder<M1_RATE, M2_RATE> {
    /// Sets the [ASMS](AsmsServo) builder
    pub fn asms_servo(mut self, asms_servo: AsmsServo) -> Self {
        self.asms_servo = Some(asms_servo);
        self
    }
    /// Sets the [WindLoads] builder
    pub fn wind_loads(mut self, wind_loads: WindLoads) -> Self {
        self.wind_loads = Some(wind_loads);
        self
    }
    /// Sets the [EdgeSensors] builder
    pub fn edge_sensors(mut self, edge_sensors: EdgeSensors) -> Self {
        self.edge_sensors = Some(edge_sensors);
        self
    }
    /// Sets the [M1SegmentFigure] builder
    pub fn m1_segment_figure(mut self, m1_segment_figure: M1SegmentFigure) -> Self {
        self.m1_segment_figure = Some(m1_segment_figure);
        self
    }
}

pub trait Include<'a, C> {
    /// Includes a component in the state space model of the GMT FEM
    fn including(self, component: Option<&'a mut C>) -> Result<Self, StateSpaceError>
    where
        Self: 'a + Sized;
}

impl<'a, const M1_RATE: usize, const M2_RATE: usize> TryFrom<ServosBuilder<M1_RATE, M2_RATE>>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    type Error = anyhow::Error;

    fn try_from(mut builder: ServosBuilder<M1_RATE, M2_RATE>) -> Result<Self, Self::Error> {
        let mut fem = builder.fem;

        if let Some(asms_servo) = builder.asms_servo.as_mut() {
            asms_servo.build(&fem)?;
        }

        let mount = Mount::new();

        log::info!("Calibrating M1");
        let m1_calibration = Calibration::new(&mut fem);
        let m1 = gmt_dos_systems_m1::M1::<M1_RATE>::new(&m1_calibration)?;

        log::info!("Calibrating ASMS positioners");
        let positioners = AsmsPositioners::new(&mut fem)?;
        log::info!("Calibrating ASMS");
        let asms = match &builder.asms_servo {
            Some(AsmsServo {
                voice_coils: Some(voice_coils),
                ..
            }) => gmt_dos_systems_m2::ASMS::<1>::new(&mut fem)?
                .modes(voice_coils.ins_transforms_view())
                .build()?,
            _ => gmt_dos_systems_m2::ASMS::<1>::new(&mut fem)?.build()?,
        };

        log::info!("Building structural state space model");
        let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
        let state_space = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem.clone())
            .sampling(builder.sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            .use_static_gain_compensation()
            .including_mount()
            .including_m1(Some(sids.clone()))?
            // .including_asms(Some(sids.clone()), None, None)?
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .ins::<MCM2SmHexF>()
            .outs::<MCM2SmHexD>()
            .including(builder.m1_segment_figure.as_mut())?
            .including(builder.asms_servo.as_mut())?
            .including(builder.wind_loads.as_mut())?
            .including(builder.edge_sensors.as_mut())?
            .build()?;

        Ok(Self {
            fem: Actor::new(state_space.into_arcx())
                .name("GMT Structural\nDynamic Model")
                .image("gmt-fem.png"),
            mount: (mount, "Mount\nController").into(),
            m1,
            m2_positioners: (positioners, "M2 Positioners\nController").into(),
            m2: asms,
        })
    }
}
