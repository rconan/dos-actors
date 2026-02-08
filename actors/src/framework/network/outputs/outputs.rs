use interface::{TryUpdate, TryWrite, UniqueIdentifier};

use crate::actor::Actor;

use super::{ActorOutputBuilder, AddOuput, OutputBuilder, OutputRx};

/// A combination of an [Actor] with an [ActorOutputBuilder]
///
/// [ActorOutput] implements the trait [AddOuput] for building outputs
pub struct ActorOutput<'a, T> {
    actor: &'a mut T,
    builder: ActorOutputBuilder,
}

impl<'a, T> ActorOutput<'a, T> {
    pub fn new(actor: &'a mut T, builder: ActorOutputBuilder) -> Self {
        Self { actor, builder }
    }
}

impl<T> OutputBuilder for ActorOutput<'_, T> {
    fn get_output_builder(&mut self) -> &mut ActorOutputBuilder {
        &mut self.builder
    }
}

impl<'a, C, const NI: usize, const NO: usize> AddOuput<'a, C, NI, NO>
    for ActorOutput<'a, Actor<C, NI, NO>>
where
    C: 'static + TryUpdate,
{
    fn build<U>(self) -> std::result::Result<(), OutputRx<U, C, NI, NO>>
    where
        C: TryWrite<U>,
        U: 'static + UniqueIdentifier,
    {
        let Self { actor, builder } = self;
        ActorOutput::build_output(actor, builder)
    }
}
