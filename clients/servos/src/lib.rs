/*!
# GMT Servo-Mechanisms

A dos-actors [system] that combines together a few clients:
  * the GMT [FEM]
  * the GMT [mount] control system
  * the GMT [M1] control system
  * the GMT [M2] control system

Per default, only a few inputs and outputs of the FEM are made available:
 * FEM inputs:
   * [`MountTorques`](gmt_dos_clients_io::mount::MountTorques)
   * [`M1HardpointsForces`](gmt_dos_clients_io::gmt_m1::assembly::M1HardpointsForces)
   * [`HardpointsForces<ID>`](gmt_dos_clients_io::gmt_m1::segment::HardpointsForces)
   * [`M1ActuatorAppliedForces`](gmt_dos_clients_io::gmt_m1::assembly::M1ActuatorAppliedForces)
   * [`ActuatorAppliedForces<ID>`](gmt_dos_clients_io::gmt_m1::segment::ActuatorAppliedForces)
   * [`M2ASMVoiceCoilsForces`](gmt_dos_clients_io::gmt_m2::asm::M2ASMVoiceCoilsForces)
   * [`VoiceCoilsForces<ID>`](gmt_dos_clients_io::gmt_m2::asm::segment::VoiceCoilsForces)
   * [`M2ASMFluidDampingForces`](gmt_dos_clients_io::gmt_m2::asm::M2ASMFluidDampingForces)
   * [`FluidDampingForces<ID>`](gmt_dos_clients_io::gmt_m2::asm::segment::FluidDampingForces)
   * [`M2PositionerForces`](gmt_dos_clients_io::gmt_m2::M2PositionerForces)
 * FEM outputs
   * [`MountEncoders`](gmt_dos_clients_io::mount::MountEncoders)
   * [`M1HardpointsMotion`](gmt_dos_clients_io::gmt_m1::assembly::M1HardpointsMotion)
   * [`HardpointsMotion<ID>`](gmt_dos_clients_io::gmt_m1::segment::HardpointsMotion)
   * [`M1RigidBodyMotions`](gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions)
   * [`M2ASMVoiceCoilsMotion`](gmt_dos_clients_io::gmt_m2::asm::M2ASMVoiceCoilsMotion)
   * [`VoiceCoilsMotion<ID>`](gmt_dos_clients_io::gmt_m2::asm::segment::VoiceCoilsMotion)
   * [`M2RigidBodyMotions`](gmt_dos_clients_io::gmt_m2::M2RigidBodyMotions)
   * [`M2PositionerNodes`](gmt_dos_clients_io::gmt_m2::M2PositionerNodes)

Other builders will add extra inputs and outputs to the FEM.
These builders are:
  * [`AsmsServo`]
     * [`Facesheet`][asms_servo::Facesheet]
     * [`ReferenceBody`][asms_servo::ReferenceBody]
  * [`WindLoads`]

## Example

```no_run
use gmt_dos_clients_servos::{asms_servo, AsmsServo, GmtServoMechanisms};
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80; // 100Hz

let frequency = 8000_f64; // Hz
let fem = FEM::from_env()?;

let gmt_servos =
    GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem).build()?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

[system]: https://docs.rs/gmt_dos-actors/latest/gmt_dos_actors/system
[FEM]: https://docs.rs/gmt_dos-clients_fem/latest/gmt_dos_clients_fem/
[mount]: https://docs.rs/gmt_dos-clients_mount/latest/gmt_dos_clients_mount/
[M1]: https://docs.rs/gmt_dos-clients_m1-ctrl/latest/gmt_dos_clients_m1_ctrl/
[M2]: https://docs.rs/gmt_dos-clients_m2-ctrl/latest/gmt_dos_clients_m2_ctrl/
*/

#[cfg(fem)]
mod builder;
#[cfg(fem)]
mod servos;
#[cfg(fem)]
mod fem {
    pub use crate::builder::{asms_servo, AsmsServo, ServosBuilder, WindLoads};
    use crate::servos;
    use gmt_dos_actors::system::Sys;

    /// GMT servo-mechanisms system
    pub enum GmtServoMechanisms<const M1_RATE: usize, const M2_RATE: usize = 1> {}

    impl<const M1_RATE: usize, const M2_RATE: usize> GmtServoMechanisms<M1_RATE, M2_RATE> {
        /// Create a new [builder](ServosBuilder)
        pub fn new(
            sim_sampling_frequency: f64,
            fem: gmt_fem::FEM,
        ) -> ServosBuilder<M1_RATE, M2_RATE> {
            ServosBuilder {
                sim_sampling_frequency,
                fem,
                ..Default::default()
            }
        }
    }

    impl<const M1_RATE: usize, const M2_RATE: usize> ServosBuilder<M1_RATE, M2_RATE> {
        /// Build the system
        pub fn build(
            self,
        ) -> anyhow::Result<Sys<servos::GmtServoMechanisms<'static, M1_RATE, M2_RATE>>> {
            Ok(
                Sys::new(servos::GmtServoMechanisms::<'static, M1_RATE, M2_RATE>::try_from(self)?)
                    .build()?,
            )
        }
    }

    /// GMT FEM client
    pub type GmtFem =
        gmt_dos_clients_fem::DiscreteModalSolver<gmt_dos_clients_fem::ExponentialMatrix>;
    /// GMT M1 client
    pub type GmtM1 = gmt_dos_clients_m1_ctrl::assembly::DispatchIn;
    /// GMT mount client
    pub type GmtMount<'a> = gmt_dos_clients_mount::Mount<'a>;
    /// GMT M2 positioners client
    pub type GmtM2Hex = gmt_dos_clients_m2_ctrl::positioner::AsmsPositioners;
    /// GMT M2 mirror client
    pub type GmtM2 = gmt_dos_clients_m2_ctrl::assembly::DispatchIn;
}
#[cfg(fem)]
pub use fem::*;
