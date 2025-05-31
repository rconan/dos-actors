/*!
Repository for the ASMS component builders

## Example

```no_run
use gmt_dos_clients_servos::{asms_servo, AsmsServo, GmtServoMechanisms};
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80; // 100Hz

let frequency = 8000_f64; // Hz
let fem = FEM::from_env()?;

let gmt_servos =
    GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
        .asms_servo(
            AsmsServo::new()
                .facesheet(
                    asms_servo::Facesheet::new()
                        .filter_piston_tip_tilt()
                        .transforms("KLmodesGS36p90.mat", "KL"),
                )
                .reference_body(
                    asms_servo::ReferenceBody::new()
                ),
        )
        .build()?;
# Ok::<(), Box<dyn std::error::Error>>(())
```
*/

use gmt_dos_clients_fem::{DiscreteStateSpace, StateSpaceError, solvers::ExponentialMatrix};

mod facesheet;
mod reference_body;
mod voice_coils;
pub use facesheet::{Facesheet, FacesheetOptions};
pub use reference_body::ReferenceBody;
pub use voice_coils::VoiceCoils;

use facesheet::FacesheetError;

use super::Include;

#[derive(Debug, thiserror::Error)]
pub enum AsmsServoError {
    #[error("Failed to build the ASMS facesheets")]
    Facesheet(#[from] FacesheetError),
}

/// ASMS builder
///
///
/// The rigid body motions of the facesheet are removed per default.
/// If is not desirable to remove the rigid body motions of the facesheet,
/// the type parameter `F` can be set to `false`.
#[derive(Debug, Clone, Default)]
pub struct AsmsServo {
    pub(crate) voice_coils: Option<VoiceCoils>,
    facesheet: Option<Facesheet>,
    reference_body: Option<ReferenceBody>,
}

impl AsmsServo {
    /// Creates a new ASMS builder
    pub fn new() -> Self {
        Default::default()
    }
    /// Sets the ASMS [Facesheet] builder
    pub fn facesheet(mut self, facesheet: Facesheet) -> Self {
        self.facesheet = Some(facesheet);
        self
    }
    /// Sets the ASMS [ReferenceBody] builder
    pub fn reference_body(mut self, reference_body: ReferenceBody) -> Self {
        self.reference_body = Some(reference_body);
        self
    }
    pub fn voice_coils(mut self, voice_coils: VoiceCoils) -> Self {
        self.voice_coils = Some(voice_coils);
        self
    }
    pub fn build(&mut self, fem: &gmt_fem::FEM) -> Result<(), AsmsServoError> {
        if let Some(facesheet) = self.facesheet.as_mut() {
            if facesheet.options.remove_rigid_body_motions() && self.reference_body.is_none() {
                self.reference_body = Some(ReferenceBody::new());
            }
            facesheet.build(fem)?;
        }
        Ok(())
    }
    pub fn n_modes(&self) -> Option<Vec<usize>> {
        self.voice_coils.as_ref().and_then(VoiceCoils::n_modes)
    }
}

impl<'a> Include<'a, AsmsServo> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(self, asms_servo: Option<&'a mut AsmsServo>) -> Result<Self, StateSpaceError> {
        let Some(asms_servo) = asms_servo else {
            return Ok(self.including_asms(Some(vec![1, 2, 3, 4, 5, 6, 7]), None, None)?);
        };
        self.including(asms_servo.voice_coils.as_mut())?
            .including(asms_servo.facesheet.as_mut())?
            .including(asms_servo.reference_body.as_mut())
    }
}
