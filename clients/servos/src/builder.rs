use gmt_dos_actors::{
    ArcMutex,
    prelude::Actor,
    system::{Sys, SystemError},
};
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_fem::{
    DiscreteModalSolver, MountAxis, StateSpaceError, solvers::ExponentialMatrix,
};
#[cfg(topend = "FSM")]
use gmt_dos_clients_io::gmt_fem::{inputs::MCM2PZTF, outputs::MCM2PZTD};
use gmt_dos_clients_io::gmt_fem::{
    inputs::MCM2SmHexF,
    outputs::{MCM2Lcl6D, MCM2SmHexD, OSSM1Lcl},
};
use gmt_dos_clients_io::{Assembly, gmt_m1::assembly::M1ActuatorAppliedForces};
use gmt_dos_clients_m2_ctrl::{Positioners, PositionersError};
use gmt_dos_clients_mount::Mount;
use gmt_dos_clients_windloads::system::SigmoidCfdLoads;
use gmt_dos_systems_m2::M2Error;

use crate::servos::GmtServoMechanisms;

#[cfg(topend = "ASM")]
pub mod asms_servo;
#[cfg(topend = "ASM")]
pub use asms_servo::AsmsServo;
mod wind_loads;
pub use wind_loads::WindLoads;
mod edge_sensors;
pub use edge_sensors::EdgeSensors;
mod m1_segment_figure;
pub use m1_segment_figure::M1SegmentFigure;
mod fem_io;
pub use fem_io::FemIO;

#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub enum Telemetry {
    #[default]
    Full,
}

/// [GmtServoMechanisms](crate::GmtServoMechanisms) builder
#[derive(Debug, Clone, Default)]
pub struct ServosBuilder<const M1_RATE: usize, const M2_RATE: usize> {
    pub(crate) sim_sampling_frequency: f64,
    pub(crate) fem: gmt_fem::FEM,
    pub(crate) fem_zeta: Option<f64>,
    pub(crate) fem_no_dcgc: bool,
    #[cfg(topend = "ASM")]
    pub(crate) asms_servo: Option<AsmsServo>,
    pub(crate) wind_loads: Option<WindLoads>,
    pub(crate) edge_sensors: Option<EdgeSensors>,
    pub(crate) m1_segment_figure: Option<M1SegmentFigure>,
    pub(crate) telemetry: Option<Telemetry>,
    pub(crate) fem_io: Option<FemIO>,
    pub(crate) kinematics: Option<Vec<(KinematicOutput, MountAxis, Vec<f64>)>>,
}

/// FEM outputs with a calibrated kinematic gain
#[derive(Debug, Clone, Copy)]
pub enum KinematicOutput {
    /// M1 segments rigid body motions ([OSSM1Lcl])
    M1RigidBodyMotions,
    /// M2 segments rigid body motions ([MCM2Lcl6D])
    M2RigidBodyMotions,
}

impl<const M1_RATE: usize, const M2_RATE: usize> ServosBuilder<M1_RATE, M2_RATE> {
    #[cfg(topend = "ASM")]
    /// Sets the [ASMS](AsmsServo) builder
    pub fn asms_servo(mut self, asms_servo: AsmsServo) -> Self {
        self.asms_servo = Some(asms_servo);
        self
    }
    /// Sets the [WindLoads] builder
    pub fn wind_loads(mut self, wind_loads: impl Into<WindLoads>) -> Self {
        self.wind_loads = Some(wind_loads.into());
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
    /// Enables the telemetry
    pub fn with_telemetry(mut self) -> Self {
        self.telemetry = Some(Telemetry::Full);
        self
    }
    /// Sets the fem inputs and outputs
    pub fn fem_io(mut self, fem_io: FemIO) -> Self {
        self.fem_io = Some(fem_io);
        self
    }
    /// Sets the FEM proportional damping coefficients
    pub fn proportial_damping(mut self, value: f64) -> Self {
        self.fem_zeta = Some(value);
        self
    }
    /// Disables FEM DC gain compensation
    pub fn no_dc_gain_compensation(mut self) -> Self {
        self.fem_no_dcgc = true;
        self
    }
    /// Removes the mount kinematic motion from all the FEM outputs but the mount encoders
    ///
    /// The outputs fed back to the servo-mechanisms (e.g. the M1 hardpoints forces) are
    /// compensated too: otherwise the M1 actuators offload the kinematic part of the hardpoints
    /// forces and the M1 segments drift with the mount azimuth
    ///
    /// See [KinematicCompensation](gmt_dos_clients_fem::KinematicCompensation)
    pub fn kinematic_compensation(mut self) -> Self {
        self.kinematics.get_or_insert_with(Vec::new);
        self
    }
    /// Replaces the FEM kinematic gain of `output` for the mount `axis` by a calibrated `gain`
    ///
    /// It also turns on the [kinematic compensation](Self::kinematic_compensation)
    pub fn kinematic_calibration(
        mut self,
        output: KinematicOutput,
        axis: MountAxis,
        gain: Vec<f64>,
    ) -> Self {
        self.kinematics
            .get_or_insert_with(Vec::new)
            .push((output, axis, gain));
        self
    }
}

pub trait Include<'a, C> {
    /// Includes a component in the state space model of the GMT FEM
    fn including(self, component: Option<&'a mut C>) -> Result<Self, StateSpaceError>
    where
        Self: 'a + Sized;
}

#[derive(Debug, thiserror::Error)]
pub enum ServosBuilderError {
    #[error("failed to build FEM state space model")]
    StateSpace(#[from] StateSpaceError),
    #[error("failed to build one of the `servos` system")]
    System(#[from] SystemError),
    #[error("failed to build the ASMS")]
    #[cfg(topend = "ASM")]
    Asms(#[from] crate::asms_servo::AsmsServoError),
    #[error("failed to create a new M2 mirror instance")]
    M2Mirror(#[from] M2Error),
    #[error("failed to create a new M2 positioners instance")]
    M2Positioner(#[from] PositionersError),
}

impl<'a, const M1_RATE: usize, const M2_RATE: usize> TryFrom<ServosBuilder<M1_RATE, M2_RATE>>
    for GmtServoMechanisms<M1_RATE, M2_RATE>
{
    type Error = ServosBuilderError;

    fn try_from(mut builder: ServosBuilder<M1_RATE, M2_RATE>) -> Result<Self, Self::Error> {
        log::info!("converting ServosBuilder into GmtServoMechanism");
        let mut fem = builder.fem;

        let wind_loads =
            if let Some(cfd_loads) = builder.wind_loads.as_mut().and_then(|w| w.cfd_loads.take()) {
                let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(cfd_loads)?;
                Some(cfd_loads)
            } else {
                None
            };

        #[cfg(topend = "ASM")]
        if let Some(asms_servo) = builder.asms_servo.as_mut() {
            asms_servo.build(&fem)?;
        }

        let mount = Mount::new();

        log::info!("Calibrating M1");
        let m1 = if let Some(transforms) = builder
            .m1_segment_figure
            .as_mut()
            .and_then(|q| q.mode_2_force_transforms.take())
        {
            gmt_dos_systems_m1::M1::<M1_RATE>::builder(&mut fem).modes_to_forces(transforms)
        } else {
            gmt_dos_systems_m1::M1::<M1_RATE>::builder(&mut fem)
        }
        .build()?;

        log::info!("Calibrating ASMS positioners");
        let positioners = Positioners::new(&mut fem)?;

        #[cfg(topend = "ASM")]
        log::info!("Calibrating ASMS");
        #[cfg(topend = "ASM")]
        let m2 = match &builder.asms_servo {
            Some(AsmsServo {
                voice_coils: Some(voice_coils),
                ..
            }) => gmt_dos_systems_m2::ASMS::<1>::new(&mut fem)?
                .modes(voice_coils.ins_transforms_view())
                .build()?,
            _ => gmt_dos_systems_m2::ASMS::<1>::new(&mut fem)?.build()?,
        };

        log::info!("Building structural state space model");
        let sids: Vec<u8> = <M1ActuatorAppliedForces as Assembly>::SIDS.to_vec();
        let state_space_builder = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem.clone())
            .sampling(builder.sim_sampling_frequency as f64)
            .proportional_damping(builder.fem_zeta.unwrap_or(2. / 100.));
        let state_space_builder = if builder.fem_no_dcgc {
            log::info!("FEM without DC gain compensation");
            state_space_builder
        } else {
            log::info!("FEM with DC gain compensation");
            state_space_builder.use_static_gain_compensation()
        }
        .including_mount()
        .including_m1(Some(sids.clone()))?
        .including(builder.m1_segment_figure.as_mut())?
        .including(builder.wind_loads.as_mut())?
        .including(builder.edge_sensors.as_mut())?
        .including(builder.fem_io.as_mut())?
        .ins::<MCM2SmHexF>()
        .outs::<MCM2SmHexD>()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>();
        let state_space_builder = match builder.kinematics.take() {
            Some(calibrations) => {
                log::info!("FEM with kinematic compensation");
                // all the outputs, including the ones fed back to the servo-mechanisms,
                // are compensated
                let state_space_builder = state_space_builder.kinematic_compensation();
                calibrations.into_iter().fold(
                    state_space_builder,
                    |ssb, (output, axis, gain)| match output {
                        KinematicOutput::M1RigidBodyMotions => {
                            ssb.kinematic_calibration::<OSSM1Lcl>(axis, gain)
                        }
                        KinematicOutput::M2RigidBodyMotions => {
                            ssb.kinematic_calibration::<MCM2Lcl6D>(axis, gain)
                        }
                    },
                )
            }
            None => state_space_builder,
        };

        #[cfg(topend = "ASM")]
        let state_space = state_space_builder
            // .including_asms(Some(sids.clone()), None, None)?
            .including(builder.asms_servo.as_mut())?
            .build()?;

        #[cfg(topend = "FSM")]
        let m2 = gmt_dos_systems_m2::M2::new()?;

        #[cfg(topend = "FSM")]
        let state_space = state_space_builder
            // .including_asms(Some(sids.clone()), None, None)?
            .ins::<MCM2PZTF>()
            .outs::<MCM2PZTD>()
            .build()?;

        Ok(Self {
            fem: Actor::new(state_space.into_arcx())
                .name("GMT Structural\nDynamic Model")
                .image("gmt-fem.png"),
            mount: (mount, "Mount\nController").into(),
            m1,
            m2_positioners: (positioners, "M2 Positioners\nController").into(),
            m2,
            telemetry: builder.telemetry.and_then(|_| {
                Some(
                    (
                        Arrow::builder(10_000)
                            .filename("servos_telemetry.parquet")
                            .build(),
                        "Telemetry",
                    )
                        .into(),
                )
            }),
            wind_loads,
        })
    }
}
