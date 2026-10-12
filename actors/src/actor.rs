/*! # Actor

The module provides an implementation of the [actor model](https://youtu.be/ELwEdb_pD0k) for the GMT Integrated Model.

Actors provide the functionalities for clients to exchange information and to update clients state,
based on the data received from other clients through these clients own Actors.

An [Actor] is build first from a client, then outputs are added one by one and for each output a corresponding input is created that is added to another [Actor].
If the output and input rates of an [Actor] are not specified, they are set to 1.

A model will always have a least one [Actor] without inputs, the [Initiator], and one [Actor] without outputs, the [Terminator].

An [Actor] runs a loop inside a dedicated thread.
The loop starts waiting for new inputs, upon reception the client reads the inputs, update its state and write to the outputs.
The [Actor] will either send the outputs immediately into the buffer of the output/input channel or, if the buffer is full, it will wait until the buffer has been read by the receiving input.

An actor can simply be derived from a client with the [From] trait.
Note that the client is consumed and no longer available.
```
use gmt_dos_actors::prelude::*;
use gmt_dos_clients::signals::Signals;
let source: Initiator<_> = Signals::new(1, 100).into();
```
A name can be given to the Actor with:
```
use gmt_dos_actors::prelude::*;
use gmt_dos_clients::signals::Signals;
let source: Initiator<_> = (Signals::new(1, 100), "My Signal").into();
```

If the client must remain available for later use, it must be wrapped inside a [Mutex] within an [Arc].
This can be easily done with the [into_arcx] method of the [ArcMutex] trait that has a blanket implementation for all type that implements the [Update](crate::Update) trait.
```
use gmt_dos_actors::prelude::*;
use gmt_dos_clients::logging::Logging;

let logging = Logging::<f64>::default().into_arcx();
let sink = Terminator::<_>::new(logging.clone());
```

[Mutex]: tokio::sync::Mutex
[Arc]: std::sync::Arc
[Arcmutex]: crate::ArcMutex
[into_arcx]: crate::ArcMutex::into_arcx
[Signals]: crate::clients::Signals
[Sampler]: crate::clients::Sampler
[Logging]: crate::clients::Logging
*/

pub(crate) mod actor;
#[cfg(feature = "serde")]
mod serde_with;

pub use actor::Actor;
mod plain;

pub use gmt_dos_actors_flowchart::{IOData, PlainActor, PlainActorBuilder, IO};

mod check;
pub mod io;
pub mod task;

/// Type alias for an actor without outputs
pub type Terminator<C, const NI: usize = 1> = Actor<C, NI, 0>;
/// Type alias for an actor without inputs
pub type Initiator<C, const NO: usize = 1> = Actor<C, 0, NO>;
/// Type alias for an actor without inputs and outputs
pub type NoNo<C> = Actor<C, 0, 0>;
