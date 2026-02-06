use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
};

use crate::actor::{
    io::{Output, OutputObject, S},
    Actor,
};

pub mod builder;
pub use builder::ActorOutputBuilder;

mod outputs;
use interface::{Quote, TryUpdate, UniqueIdentifier, Who, Write};
pub use outputs::ActorOutput;

use super::OutputRx;

/// Assign ouputs to actors
pub trait AddActorOutput<'a, C, const NI: usize, const NO: usize>
where
    C: TryUpdate,
{
    /// Adds a new output to an actor
    fn add_output(&'a mut self) -> ActorOutput<'a, Actor<C, NI, NO>>;
}

pub trait OutputBuilder {
    fn get_output_builder(&mut self) -> &mut ActorOutputBuilder;
}

/// Actor output construction interface
pub trait AddOuput<'a, C, const NI: usize, const NO: usize>
where
    C: 'static + TryUpdate,
{
    /// Sets the channel to unbounded
    fn unbounded(mut self) -> Self
    where
        Self: Sized + OutputBuilder,
    {
        self.get_output_builder().unbounded();
        self
    }
    /// Flags the output to be bootstrapped
    fn bootstrap(mut self) -> Self
    where
        Self: Sized + OutputBuilder,
    {
        self.get_output_builder().bootstrap();
        self
    }
    /// Multiplexes the output `n` times
    fn multiplex(mut self, n: usize) -> Self
    where
        Self: Sized + OutputBuilder,
    {
        self.get_output_builder().multiplex(n);
        self
    }
    /// Try to build a new output where you must fail to succeed
    fn build<U>(self) -> std::result::Result<(), OutputRx<U, C, NI, NO>>
    where
        C: Write<U>,
        U: 'static + UniqueIdentifier;
    fn build_output<U>(
        actor: &'a mut Actor<C, NI, NO>,
        builder: ActorOutputBuilder,
    ) -> std::result::Result<(), OutputRx<U, C, NI, NO>>
    where
        C: Write<U>,
        U: 'static + UniqueIdentifier,
    {
        let mut txs = vec![];
        let mut rxs = vec![];
        for &cap in builder.capacity() {
            let (tx, rx) = if cap == usize::MAX {
                flume::unbounded::<S<U>>()
            } else {
                flume::bounded::<S<U>>(cap)
            };
            txs.push(tx);
            rxs.push(rx);
        }

        // Check if this output already exists
        if let Some(outputs) = &mut actor.outputs {
            if let Some(output) = outputs
                .iter_mut()
                .find_map(|output| output.as_mut_any().downcast_mut::<Output<C, U, NO>>())
            {
                output.tx_push(txs);
                let output_name = Who::who(output);
                return Err(OutputRx {
                    hash: output.get_hash(),
                    rxs,
                    client: std::sync::Arc::clone(&actor.client),
                    actor: actor.who(),
                    output: output_name,
                });
            }
        }

        let mut output: Output<C, U, NO> = Output::builder(actor.client.clone())
            .bootstrap(builder.is_bootstrap())
            .senders(txs)
            .build();

        let mut hasher = DefaultHasher::new();
        actor.who().hash(&mut hasher);
        let output_name = Who::who(&output);
        <U as Quote>::quote().hash(&mut hasher);
        let hash = hasher.finish();
        <Output<C, U, NO> as OutputObject>::set_hash(&mut output, hash);

        if let Some(ref mut outputs) = actor.outputs {
            outputs.push(Box::new(output));
        } else {
            actor.outputs = Some(vec![Box::new(output)]);
        }

        Err(OutputRx {
            hash,
            rxs,
            client: std::sync::Arc::clone(&actor.client),
            actor: actor.who(),
            output: output_name,
        })
    }
}

/* impl<'a, C, const NI: usize, const NO: usize> AddOuput<'a, C, NI, NO>
    for (&'a mut Actor<C, NI, NO>, ActorOutputBuilder)
where
    C: 'static + TryUpdate + Send,
{
    fn unbounded(self) -> Self {
        let n = self.1.capacity.len();
        (
            self.0,
            ActorOutputBuilder {
                capacity: vec![usize::MAX; n],
                ..self.1
            },
        )
    }
    fn bootstrap(self) -> Self {
        (
            self.0,
            ActorOutputBuilder {
                bootstrap: true,
                ..self.1
            },
        )
    }
    fn multiplex(self, n: usize) -> Self {
        (
            self.0,
            ActorOutputBuilder {
                capacity: vec![self.1.capacity[0]; n],
                ..self.1
            },
        )
    }
    fn legacy_build<U>(self) -> (&'a mut Actor<C, NI, NO>, Vec<Rx<U>>)
    where
        C: 'static + TryUpdate + Send + io::Write<U>,
        U: 'static + Send + Sync + UniqueIdentifier,
        Assoc<U>: Send + Sync,
    {
        let (actor, builder) = self;
        let mut txs = vec![];
        let mut rxs = vec![];
        for &cap in &builder.capacity {
            let (tx, rx) = if cap == usize::MAX {
                flume::unbounded::<S<U>>()
            } else {
                flume::bounded::<S<U>>(cap)
            };
            txs.push(tx);
            rxs.push(rx);
        }

        let output: Output<C, Assoc<U>, U, NO> = Output::builder(actor.client.clone())
            .bootstrap(builder.bootstrap)
            .senders(txs)
            .build();

        if let Some(ref mut outputs) = actor.outputs {
            outputs.push(Box::new(output));
        } else {
            actor.outputs = Some(vec![Box::new(output)]);
        }

        (actor, rxs)
    }
    fn build<U>(self) -> std::result::Result<(), OutputRx<U, C, NI, NO>>
    where
        C: 'static + TryUpdate + Send + io::Write<U>,
        U: 'static + Send + Sync + UniqueIdentifier,
        Assoc<U>: Send + Sync,
    {
        let (actor, builder) = self;
        let mut txs = vec![];
        let mut rxs = vec![];
        for &cap in &builder.capacity {
            let (tx, rx) = if cap == usize::MAX {
                flume::unbounded::<S<U>>()
            } else {
                flume::bounded::<S<U>>(cap)
            };
            txs.push(tx);
            rxs.push(rx);
        }

        // Check if this output already exists
        if let Some(outputs) = &mut actor.outputs {
            if let Some(output) = outputs.iter_mut().find_map(|output| {
                output
                    .as_mut_any()
                    .downcast_mut::<Output<C, Assoc<U>, U, NO>>()
            }) {
                output.tx_push(txs);
                let output_name = Who::who(output);
                return Err(OutputRx {
                    hash: output.get_hash(),
                    rxs,
                    client: std::sync::Arc::clone(&actor.client),
                    actor: actor.who(),
                    output: output_name,
                });
            }
        }

        let mut output: Output<C, Assoc<U>, U, NO> = Output::builder(actor.client.clone())
            .bootstrap(builder.bootstrap)
            .senders(txs)
            .build();

        let mut hasher = DefaultHasher::new();
        actor.who().hash(&mut hasher);
        let output_name = Who::who(&output);
        output_name
            .split("::")
            .last()
            .unwrap()
            .to_owned()
            .hash(&mut hasher);
        let hash = hasher.finish();
        <Output<C, Assoc<U>, U, NO> as OutputObject>::set_hash(&mut output, hash);

        if let Some(ref mut outputs) = actor.outputs {
            outputs.push(Box::new(output));
        } else {
            actor.outputs = Some(vec![Box::new(output)]);
        }

        Err(OutputRx {
            hash,
            rxs,
            client: std::sync::Arc::clone(&actor.client),
            actor: actor.who(),
            output: output_name,
        })
    }
}
 */
