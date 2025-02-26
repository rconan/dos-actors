use gmt_dos_clients_fem::{
    fem_io::actors_outputs::{M2EdgeSensors, OSSM1EdgeSensors},
    solvers::ExponentialMatrix,
    DiscreteStateSpace,
};
use nalgebra as na;

use super::Include;

#[derive(Debug, Clone, Default)]
struct M1EdgeSensor {
    transform: Option<na::DMatrix<f64>>,
}
#[derive(Debug, Clone, Default)]
struct M2EdgeSensor {
    transform: Option<na::DMatrix<f64>>,
}

/**
Edge sensors builder

The edge sensors builder adds the following outputs to the FEM:
 * [`M1EdgeSensors`](gmt_dos_clients_io::gmt_m1::M1EdgeSensors)
 * [`M2EdgeSensors`](gmt_dos_clients_io::gmt_m2::M2EdgeSensors)

 ```no_run
use gmt_dos_clients_servos::{GmtServoMechanisms, EdgeSensors};
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80; // 100Hz

let frequency = 8000_f64; // Hz
let fem = FEM::from_env()?;

let gmt_servos =
    GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
        .edge_sensors(EdgeSensors::both())
        .build()?;
# Ok::<(), Box<dyn std::error::Error>>(())
```
 */
#[derive(Debug, Default, Clone)]
pub struct EdgeSensors {
    m1: Option<M1EdgeSensor>,
    m2: Option<M2EdgeSensor>,
}

impl EdgeSensors {
    /// Creates a new empty [EdgeSensors] builder
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, EdgeSensors};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .edge_sensors(EdgeSensors::none())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn none() -> Self {
        Self { m1: None, m2: None }
    }
    /// Creates a new [EdgeSensors] builder for both M1 and M2 edge sensors
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, EdgeSensors};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .edge_sensors(EdgeSensors::both())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn both() -> Self {
        Self {
            m1: Some(M1EdgeSensor::default()),
            m2: Some(M2EdgeSensor::default()),
        }
    }
    /// Creates a new [EdgeSensors] builder for M1 edge sensors
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, EdgeSensors};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .edge_sensors(EdgeSensors::m1())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn m1() -> Self {
        Self {
            m1: Some(M1EdgeSensor::default()),
            m2: None,
        }
    }
    /// Applies a linear transformation to the M1 edge sensors
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, EdgeSensors};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .edge_sensors(EdgeSensors::m1().m1_with(nalgebra::DMatrix::<f64>::identity(288,288)))
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn m1_with(mut self, transform: na::DMatrix<f64>) -> Self {
        self.m1 = Some(M1EdgeSensor {
            transform: Some(transform),
        });
        self
    }
    /// Creates a new [EdgeSensors] builder for M2 edge sensors
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, EdgeSensors};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .edge_sensors(EdgeSensors::m2())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn m2() -> Self {
        Self {
            m1: None,
            m2: Some(M2EdgeSensor::default()),
        }
    }
    /// Applies a linear transformation to the M2 edge sensors
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, EdgeSensors};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .edge_sensors(EdgeSensors::m2().m2_with(nalgebra::DMatrix::<f64>::identity(48,48)))
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn m2_with(mut self, transform: na::DMatrix<f64>) -> Self {
        self.m2 = Some(M2EdgeSensor {
            transform: Some(transform),
        });
        self
    }
}

impl<'a> Include<'a, EdgeSensors> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(
        self,
        edge_sensors: Option<&'a mut EdgeSensors>,
    ) -> Result<Self, gmt_dos_clients_fem::StateSpaceError> {
        let Some(edge_sensors) = edge_sensors else {
            return Ok(self);
        };
        let mut state_space = self;
        state_space = match edge_sensors.m1.as_ref() {
            Some(M1EdgeSensor {
                transform: Some(transform),
            }) => state_space.outs_with::<OSSM1EdgeSensors>(transform.as_view()),
            Some(M1EdgeSensor { transform: None }) => state_space.outs::<OSSM1EdgeSensors>(),
            _ => state_space,
        };
        state_space = match edge_sensors.m2.as_ref() {
            Some(M2EdgeSensor {
                transform: Some(transform),
            }) => state_space.outs_with::<M2EdgeSensors>(transform.as_view()),
            Some(M2EdgeSensor { transform: None }) => state_space.outs::<M2EdgeSensors>(),
            _ => state_space,
        };
        Ok(state_space)
    }
}
