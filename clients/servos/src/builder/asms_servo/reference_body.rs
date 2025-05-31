use gmt_dos_clients_fem::{DiscreteStateSpace, StateSpaceError, solvers::ExponentialMatrix};

use crate::builder::Include;

/**
ASMS reference body builder

The reference body builder adds the following outputs to the FEM:
 * [`M2ASMReferenceBodyNodes`](gmt_dos_clients_io::gmt_m2::asm::M2ASMReferenceBodyNodes)

```no_run
use gmt_dos_clients_servos::{asms_servo, AsmsServo, GmtServoMechanisms};
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80; // 100Hz

let frequency = 8000_f64; // Hz
let fem = FEM::from_env()?;

let gmt_servos =
    GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
        .asms_servo(
            AsmsServo::new().reference_body(
                asms_servo::ReferenceBody::new()
            ),
        )
        .build()?;
# Ok::<(), Box<dyn std::error::Error>>(())
```
 */
#[derive(Debug, Clone, Default)]
pub struct ReferenceBody();

impl ReferenceBody {
    /// Creates a new [ReferenceBody] builder
    /// ```no_run
    /// # use gmt_dos_clients_servos::{asms_servo, AsmsServo, GmtServoMechanisms};
    /// # use gmt_fem::FEM;
    /// # const ACTUATOR_RATE: usize = 80; // 100Hz
    /// # let frequency = 8000_f64; // Hz
    /// # let fem = FEM::from_env()?;
    /// let gmt_servos =
    ///     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
    ///         .asms_servo(
    ///             AsmsServo::new().reference_body(
    ///                 asms_servo::ReferenceBody::new()
    ///             ),
    ///         )
    ///     .build()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn new() -> Self {
        Self()
    }
}

impl<'a> Include<'a, ReferenceBody> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(
        self,
        reference_body: Option<&'a mut ReferenceBody>,
    ) -> Result<Self, StateSpaceError> {
        let Some(_) = reference_body else {
            return Ok(self);
        };
        Ok(self.outs::<gmt_dos_clients_io::gmt_fem::outputs::MCM2RB6D>())
    }
}
