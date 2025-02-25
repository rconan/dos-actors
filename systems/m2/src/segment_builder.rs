use gmt_dos_actors::prelude::{Actor, AddActorOutput, AddOuput, TryIntoInputs};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::gmt_m2::asm::segment::{
    AsmCommand, FluidDampingForces, VoiceCoilsForces, VoiceCoilsMotion,
};
use interface::{Update, Write};

use crate::{AsmSegmentInnerController, Segment};

pub struct SegmentBuilder<'a, const ID: u8, C, const N: usize>
where
    C: Update + Write<AsmCommand<ID>> + Send + 'static,
{
    stiffness: &'a [f64],
    n_mode: usize,
    setpoint_actor: &'a mut Actor<C, N, 1>,
}

impl<'a, const ID: u8, C, const N: usize> SegmentBuilder<'a, ID, C, N>
where
    C: Update + Write<AsmCommand<ID>> + Send + 'static,
{
    /// Returns a mount actor
    ///
    ///  The `MountEncoders` input and `MountTorques` output of the mount actor are linked to the plant
    pub fn build(
        self,
        plant: &mut Actor<DiscreteModalSolver<ExponentialMatrix>>,
    ) -> anyhow::Result<Actor<AsmSegmentInnerController<ID>>> {
        let mut asm: Actor<_> = (
            AsmSegmentInnerController::<ID>::new(self.n_mode, Some(self.stiffness.to_vec())),
            format!(
                "ASMS
     Segment #{ID}"
            ),
        )
            .into();
        self.setpoint_actor
            .add_output()
            .build::<AsmCommand<ID>>()
            .into_input(&mut asm)?;
        asm.add_output()
            .build::<VoiceCoilsForces<ID>>()
            .into_input(plant)?;
        asm.add_output()
            .build::<FluidDampingForces<ID>>()
            .into_input(plant)?;
        plant
            .add_output()
            .bootstrap()
            .build::<VoiceCoilsMotion<ID>>()
            .into_input(&mut asm)?;
        Ok(asm)
    }
}

impl<'a, const ID: u8> Segment<ID> {
    pub fn builder<C, const N: usize>(
        n_mode: usize,
        stiffness: &'a [f64],
        setpoint_actor: &'a mut Actor<C, N, 1>,
    ) -> SegmentBuilder<'a, ID, C, N>
    where
        C: Update + Write<AsmCommand<ID>> + Send + 'static,
    {
        SegmentBuilder {
            stiffness,
            n_mode,
            setpoint_actor,
        }
    }
}
