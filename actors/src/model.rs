/*!
# Actors model

The module implements the high-level integrated model interface.
The model is build from a collection of [actor]s.

The model has 4 states:
 1. [Unknown]: model state at its creation
 2. [Ready]: model state after succesfully performing runtime checks on inputs and outputs on all the actors, the model can move to the [Ready] state only from the [Unknown] state
 3. [Running]: model state while all the actors are performing their respective tasks, the model can move to the [Running] state only from the [Ready] state
 4. [Completed]: model state after the succesful completion of the tasks of all the actors, the model can move to the [Completed] state only from the [Running] state

# Example

A 3 actors model with [Signals], [Sampler] and [Logging] clients is build with:
```
use gmt_dos_actors::prelude::*;
use gmt_dos_clients::signals::Signals;
use gmt_dos_clients::sampler::Sampler;
use gmt_dos_clients::logging::Logging;
use interface::UID;

let mut source: Initiator<_> = Signals::new(1, 100).into();
#[derive(UID)]
enum Source {};
let mut sampler: Actor<_, 1, 10> = Sampler::<Vec<f64>, Source>::default().into();
let logging = Logging::<f64>::default().into_arcx();
let mut sink = Terminator::<_, 10>::new(logging);
```
`sampler` decimates `source` with a 1:10 ratio.
The `source` connects to the `sampler` using the empty enum type `Source` as the data identifier.
The source data is then logged into the client of the `sink` actor.
```
# use gmt_dos_actors::prelude::*;
# use gmt_dos_clients::signals::Signals;
# use gmt_dos_clients::sampler::Sampler;
# use gmt_dos_clients::logging::Logging;
# use interface::UID;
# let mut source: Initiator<_> = Signals::new(1, 100).into();
# #[derive(UID)]
# enum Source {};
# let mut sampler: Actor<_> = Sampler::<Vec<f64>, Source>::default().into();
# let logging = Logging::<f64>::default().into_arcx();
# let mut sink = Terminator::<_>::new(logging);
source.add_output().build::<Source>().into_input(&mut sampler);
sampler.add_output().build::<Source>().into_input(&mut sink);
```
A [model](mod@crate::model) is build from the set of actors:
```
# use gmt_dos_actors::prelude::*;
# use gmt_dos_clients::signals::Signals;
# use gmt_dos_clients::sampler::Sampler;
# use gmt_dos_clients::logging::Logging;
# use interface::UID;
# let mut source: Initiator<_> = Signals::new(1, 100).into();
# #[derive(UID)]
# enum Source {};
# let mut sampler: Actor<_> = Sampler::<Vec<f64>, Source>::default().into();
# let logging = Logging::<f64>::default().into_arcx();
# let mut sink = Terminator::<_>::new(logging.clone());
# source.add_output().build::<Source>().into_input(&mut sampler);
# sampler.add_output().build::<Source>().into_input(&mut sink);
Model::new(vec![Box::new(source), Box::new(sampler), Box::new(sink)]);
```
Actors are checked for inputs/outputs consistencies:
```
# use gmt_dos_actors::prelude::*;
# use gmt_dos_clients::signals::Signals;
# use gmt_dos_clients::sampler::Sampler;
# use gmt_dos_clients::logging::Logging;
# use interface::UID;
# let mut source: Initiator<_> = Signals::new(1, 100).into();
# #[derive(UID)]
# enum Source {};
# let mut sampler: Actor<_> = Sampler::<Vec<f64>, Source>::default().into();
# let logging = Logging::<f64>::default().into_arcx();
# let mut sink = Terminator::<_>::new(logging.clone());
# source.add_output().build::<Source>().into_input(&mut sampler);
# sampler.add_output().build::<Source>().into_input(&mut sink);
Model::new(vec![Box::new(source), Box::new(sampler), Box::new(sink)])
       .check()?;
# Ok::<(), gmt_dos_actors::model::ModelError>(())
```
The model run the actor tasks:
```
# tokio_test::block_on(async {
# use gmt_dos_actors::prelude::*;
# use gmt_dos_clients::signals::Signals;
# use gmt_dos_clients::sampler::Sampler;
# use gmt_dos_clients::logging::Logging;
# use interface::UID;
# let mut source: Initiator<_> = Signals::new(1, 100).into();
# #[derive(UID)]
# enum Source {};
# let mut sampler: Actor<_> = Sampler::<Vec<f64>, Source>::default().into();
# let logging = Logging::<f64>::default().into_arcx();
# let mut sink = Terminator::<_>::new(logging.clone());
# source.add_output().build::<Source>().into_input(&mut sampler);
# sampler.add_output().build::<Source>().into_input(&mut sink);
Model::new(vec![Box::new(source), Box::new(sampler), Box::new(sink)])
       .check()?
       .run();
# Ok::<(), gmt_dos_actors::model::ModelError>(())
# });
```
and wait for the tasks to finish:
```
# tokio_test::block_on(async {
# use gmt_dos_actors::prelude::*;
# use gmt_dos_clients::signals::Signals;
# use gmt_dos_clients::sampler::Sampler;
# use gmt_dos_clients::logging::Logging;
# use interface::UID;
# let mut source: Initiator<_> = Signals::new(1, 100).into();
# #[derive(UID)]
# enum Source {};
# let mut sampler: Actor<_> = Sampler::<Vec<f64>, Source>::default().into();
# let logging = Logging::<f64>::default().into_arcx();
# let mut sink = Terminator::<_>::new(logging.clone());
# source.add_output().build::<Source>().into_input(&mut sampler);
# sampler.add_output().build::<Source>().into_input(&mut sink);
Model::new(vec![Box::new(source), Box::new(sampler), Box::new(sink)])
       .check()?
       .run()
       .wait()
       .await?;
# Ok::<(), gmt_dos_actors::model::ModelError>(())
# });
```
Once the model run to completion, the data from `logging` is read with:
```
# tokio_test::block_on(async {
# use gmt_dos_actors::prelude::*;
# use gmt_dos_clients::signals::Signals;
# use gmt_dos_clients::sampler::Sampler;
# use gmt_dos_clients::logging::Logging;
# use interface::UID;
# let mut source: Initiator<_> = Signals::new(1, 100).into();
# #[derive(UID)]
# enum Source {};
# let mut sampler: Actor<_> = Sampler::<Vec<f64>, Source>::default().into();
# let logging = Logging::<f64>::default().into_arcx();
# let mut sink = Terminator::<_>::new(logging.clone());
# source.add_output().build::<Source>().into_input(&mut sampler);
# sampler.add_output().build::<Source>().into_input(&mut sink);
# Model::new(vec![Box::new(source), Box::new(sampler), Box::new(sink)])
#       .check()?
#       .run()
#       .wait()
#       .await?;
let data: &[f64]  = &logging.lock().await;
# Ok::<(), gmt_dos_actors::model::ModelError>(())
# });
```

[Actor]: crate::actor::Actor
[Write]: interface::Write
[Read]: interface::Read
[Update]: interface::Update
[Model]: crate::model::Model
[Mutex]: tokio::sync::Mutex
[Arc]: std::sync::Arc
[Arcmutex]: crate::ArcMutex
[into_arcx]: crate::ArcMutex::into_arcx
[Signals]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/logging/struct.Signals.html
[Sampler]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/logging/struct.Sampler.html
[Logging]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/logging/struct.Logging.html
*/

use crate::framework::model::{CheckError, Task, TaskError};
use std::{fmt::Display, marker::PhantomData, time::Instant};

mod flowchart;
use tokio::task::JoinSet;

#[derive(thiserror::Error, Debug)]
pub enum ModelError {
    #[error("no actors found in the model")]
    NoActors,
    #[error("failed to join the task")]
    TaskError(#[from] tokio::task::JoinError),
    #[error("Actor IO inconsistency")]
    ActorIO(#[from] crate::ActorError),
    #[error("error in Task implementation")]
    Task(#[from] Box<TaskError>),
    #[error("error in Check implementation")]
    Check(#[from] Box<CheckError>),
}

type Result<T> = std::result::Result<T, ModelError>;

/// [Model] initial state
pub enum Unknown {}
/// Valid [Model] state
pub enum Ready {}
/// [Model]ing in-progress state
pub enum Running {}
/// [Model] final state
pub enum Completed {}

type Actors = Vec<Box<dyn Task>>;

/// Actor model
pub struct Model<State> {
    pub(crate) name: Option<String>,
    pub(crate) actors: Option<Actors>,
    pub(crate) task_set: Option<JoinSet<std::result::Result<(), TaskError>>>,
    pub(crate) state: PhantomData<State>,
    pub(crate) start: Instant,
    pub(crate) verbose: bool,
    pub(crate) elapsed_time: f64,
}

impl<S> Display for Model<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "{} [{},{:?}]",
            self.name.as_ref().unwrap_or(&"ACTOR MODEL".to_string()),
            self.n_actors(),
            self.n_io()
        )?;
        if let Some(actors) = &self.actors {
            for actor in actors {
                write!(f, " {}", actor)?;
            }
        }
        Ok(())
    }
}

impl<S> Model<S> {
    /// Prints some informations about the model and the actors within
    pub fn inspect(self) -> Self {
        println!("{self}");
        self
    }
    /// Returns the total number of inputs and the total number of outputs
    ///
    /// Both numbers should be the same
    pub fn n_io(&self) -> (usize, usize) {
        if let Some(ref actors) = self.actors {
            actors
                .iter()
                .fold((0usize, 0usize), |(mut i, mut o), actor| {
                    i += actor.n_inputs();
                    o += actor.n_outputs();
                    (i, o)
                })
        } else {
            (0, 0)
        }
    }
    /// Returns the number of actors
    pub fn n_actors(&self) -> usize {
        self.actors.as_ref().map_or(0, |actors| actors.len())
    }
    pub fn get_name(&self) -> String {
        self.name.clone().unwrap_or("model".to_string())
    }
    pub fn elapsed_time(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.elapsed_time)
    }
}

#[doc(hidden)]
pub trait UnknownOrReady {}
impl UnknownOrReady for Unknown {}
impl UnknownOrReady for Ready {}

mod plain;
pub mod ready;
pub mod running;
pub mod unknown;
pub use plain::PlainModel;

// mod task;
