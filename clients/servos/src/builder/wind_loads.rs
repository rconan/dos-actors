use gmt_dos_clients_fem::{solvers::ExponentialMatrix, DiscreteStateSpace, StateSpaceError};

use super::Include;

#[derive(Debug, Clone)]
pub enum WindLoaded {
    Mount,
    M1,
    M2,
    None,
}

/**
Wind loads builder

The wind loads builder adds the following inputs to the FEM:
 * [`CFDMountWindLoads`](gmt_dos_clients_io::cfd_wind_loads::CFDMountWindLoads)
 * [`CFDM1WindLoads`](gmt_dos_clients_io::cfd_wind_loads::CFDM1WindLoads)
 * [`CFDM2WindLoads`](gmt_dos_clients_io::cfd_wind_loads::CFDM2WindLoads)

```no_run
use gmt_dos_clients_servos::{GmtServoMechanisms, WindLoads};
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80; // 100Hz

let frequency = 8000_f64; // Hz
let fem = FEM::from_env()?;

let gmt_servos =
    GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
        .wind_loads(WindLoads::new())
        .build()?;
# Ok::<(), Box<dyn std::error::Error>>(())
```
 */
#[derive(Debug, Clone)]
pub struct WindLoads {
    mount: WindLoaded,
    m1: WindLoaded,
    m2: WindLoaded,
}

impl Default for WindLoads {
    fn default() -> Self {
        Self {
            mount: WindLoaded::Mount,
            m1: WindLoaded::M1,
            m2: WindLoaded::M2,
        }
    }
}

impl WindLoads {
    /// Creates a new [WindLoads] builder
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, WindLoads};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .wind_loads(WindLoads::new())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn new() -> Self {
        Default::default()
    }
    /// Disable the mount wind loads
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, WindLoads};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .wind_loads(WindLoads::new().no_mount())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn no_mount(mut self) -> Self {
        self.mount = WindLoaded::None;
        self
    }
    /// Disable the M1 wind loads
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, WindLoads};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .wind_loads(WindLoads::new().no_m1())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn no_m1(mut self) -> Self {
        self.m1 = WindLoaded::None;
        self
    }
    /// Disable the M2 wind loads
    /// ```no_run
    /// # use gmt_dos_clients_servos::{GmtServoMechanisms, WindLoads};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .wind_loads(WindLoads::new().no_m2())
    ///         .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn no_m2(mut self) -> Self {
        self.m2 = WindLoaded::None;
        self
    }
}

impl<'a> Include<'a, WindLoads> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(self, wind_loads: Option<&'a mut WindLoads>) -> Result<Self, StateSpaceError> {
        let Some(wind_loads) = wind_loads else {
            return Ok(self);
        };
        let mut state_space = self;
        #[cfg(cfd2021)]
        if let WindLoaded::Mount = wind_loads.mount {
            state_space = state_space.ins::<gmt_dos_clients_io::gmt_fem::inputs::CFD2021106F>();
        }
        #[cfg(cfd2025)]
        if let WindLoaded::Mount = wind_loads.mount {
            state_space = state_space.ins::<gmt_dos_clients_io::gmt_fem::inputs::CFD2025046F>();
        }
        if let WindLoaded::M1 = wind_loads.m1 {
            state_space = state_space.ins::<gmt_dos_clients_io::gmt_fem::inputs::OSSM1Lcl6F>();
        }
        if let WindLoaded::M2 = wind_loads.m2 {
            state_space = state_space.ins::<gmt_dos_clients_io::gmt_fem::inputs::MCM2Lcl6F>();
        }
        Ok(state_space)
    }
}
