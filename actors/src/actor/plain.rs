use interface::{TryUpdate, Who};

use super::{Actor, PlainActor};

impl<C, const NI: usize, const NO: usize> From<&Actor<C, NI, NO>> for PlainActor
where
    C: TryUpdate,
{
    fn from(actor: &Actor<C, NI, NO>) -> Self {
        PlainActor::new(actor.name.as_ref().unwrap_or(&actor.who()))
            .maybe_inputs(
                actor
                    .inputs
                    .as_ref()
                    .map(|inputs| inputs.iter().map(|o| o.as_io(NI)).collect()),
            )
            .maybe_outputs(
                actor
                    .outputs
                    .as_ref()
                    .map(|outputs| outputs.iter().map(|o| o.as_io(NO)).collect()),
            )
            .maybe_image(actor.image.as_ref().cloned())
            .uid(super::replay::uid(actor))
            .build()
    }
}
