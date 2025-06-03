use crate::Mount;
use gmt_dos_actors::{
    actor::Actor,
    framework::network::{AddActorOutput, AddOuput, TryIntoInputs},
};
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::mount::{MountEncoders, MountSetPoint, MountTorques};
use interface::{Update, Write};

/// Builder for [Mount] actors
pub struct Builder<'a, C, const N: usize>
where
    C: Update + Write<MountSetPoint> + Send + 'static,
{
    setpoint_actor: &'a mut Actor<C, N, 1>,
}
impl<'a, C, const N: usize> Builder<'a, C, N>
where
    C: Update + Write<MountSetPoint> + Send + 'static,
{
    /// Returns a mount actor
    ///
    ///  The `MountEncoders` input and `MountTorques` output of the mount actor are linked to the plant
    pub fn build(
        self,
        plant: &mut Actor<DiscreteModalSolver<ExponentialMatrix>>,
    ) -> anyhow::Result<Actor<Mount>> {
        let mut mount: Actor<_> = (
            Mount::new(),
            "Mount
Control",
        )
            .into();
        self.setpoint_actor
            .add_output()
            .build::<MountSetPoint>()
            .into_input(&mut mount)?;
        mount
            .add_output()
            .build::<MountTorques>()
            .into_input(plant)?;
        plant
            .add_output()
            .bootstrap()
            .build::<MountEncoders>()
            .into_input(&mut mount)?;
        Ok(mount)
    }
}

impl Mount {
    /// Returns a mount actor [Builder]
    ///
    /// The mount will be driven to the `MountSetPoint` output signal from the setpoint actor
    pub fn builder<'a, C, const N: usize>(
        setpoint_actor: &'a mut Actor<C, N, 1>,
    ) -> Builder<'a, C, N>
    where
        C: Update + Write<MountSetPoint> + Send + 'static,
    {
        Builder { setpoint_actor }
    }
}
