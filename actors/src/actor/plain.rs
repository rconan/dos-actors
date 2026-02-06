use interface::{TryUpdate, Who};

use crate::{
    actor::io::{InputObject, OutputObject},
    graph::Graph,
    trim,
};

use super::Actor;

/// Actor input or output content
#[derive(Debug, Hash, Clone)]
pub struct IOData {
    pub(crate) name: String,
    pub(crate) hash: u64,
    pub(crate) n: usize,
    pub(crate) rate: usize,
}
impl IOData {
    /// Creates a plain input or output instance
    pub fn new(name: String, hash: u64, n: usize, rate: usize) -> Self {
        Self {
            name,
            hash,
            n,
            rate,
        }
    }
    /// Returns the I/O hash #
    pub fn hash(&self) -> u64 {
        self.hash
    }
}
impl PartialEq<u64> for IOData {
    fn eq(&self, other: &u64) -> bool {
        self.hash == *other
    }
}
/// Actor input or output per kind
#[derive(Debug, Hash, Clone)]
pub enum IO {
    Bootstrap(IOData),
    Regular(IOData),
    Unbounded(IOData),
}
impl IO {
    /// Selects or not this [IO] based on predicate outcome
    pub fn filter<F>(&self, pred: F) -> bool
    where
        F: Fn(&IOData) -> bool,
    {
        if self.len() > 1 {
            return true;
        }
        pred(match self {
            IO::Bootstrap(data) => data,
            IO::Regular(data) => data,
            IO::Unbounded(data) => data,
        })
    }
    /// Selects this [IO] if its name does not match any given names
    pub fn filter_by_name(&self, names: &[&str]) -> bool {
        if self.len() > 1 {
            return true;
        }
        let io_name = self.name();
        !names.iter().any(|name| io_name.contains(name))
    }
    /// Returns the [IO] multiplexing factor
    pub fn len(&self) -> usize {
        match self {
            IO::Bootstrap(data) => data.n,
            IO::Regular(data) => data.n,
            IO::Unbounded(data) => data.n,
        }
    }
    /// Returns the [IO] name
    pub fn name(&self) -> &str {
        match self {
            IO::Bootstrap(data) => &data.name,
            IO::Regular(data) => &data.name,
            IO::Unbounded(data) => &data.name,
        }
    }
    /// Returns the [IO] rate
    pub fn rate(&self) -> usize {
        match self {
            IO::Bootstrap(data) => data.rate,
            IO::Regular(data) => data.rate,
            IO::Unbounded(data) => data.rate,
        }
    }
}
impl IO {
    pub fn hash(&self) -> u64 {
        match self {
            IO::Bootstrap(data) => data.hash(),
            IO::Regular(data) => data.hash(),
            IO::Unbounded(data) => data.hash(),
        }
    }
}
impl PartialEq<u64> for &IO {
    fn eq(&self, other: &u64) -> bool {
        self.hash() == *other
    }
}
impl PartialEq<u64> for IO {
    fn eq(&self, other: &u64) -> bool {
        self.hash() == *other
    }
}

/// [PlainActor] builder
#[derive(Debug, Default, Clone)]
pub struct PlainActorBuilder {
    pub(crate) client: String,
    pub(crate) inputs: Option<Vec<IO>>,
    pub(crate) outputs: Option<Vec<IO>>,
    pub(crate) hash: u64,
    pub(crate) image: Option<String>,
    pub(crate) graph: Option<Graph>,
}

impl PlainActorBuilder {
    /// Sets the inputs
    pub fn inputs(mut self, inputs: Vec<IO>) -> Self {
        self.inputs = Some(inputs);
        self
    }
    /// Sets the outputs
    pub fn outputs(mut self, outputs: Vec<IO>) -> Self {
        self.outputs = Some(outputs);
        self
    }
    /// Sets the [FlowChart] graph for a [System]
    pub fn graph(mut self, graph: Option<Graph>) -> Self {
        self.graph = graph;
        self
    }
    /// Sets the actor or system image
    pub fn image(mut self, image: impl ToString) -> Self {
        self.image = Some(image.to_string());
        self
    }
    /// Builds a [PlainActor]
    pub fn build(self) -> PlainActor {
        PlainActor {
            client: self.client,
            inputs: self.inputs,
            outputs: self.outputs,
            graph: self.graph,
            hash: self.hash,
            image: self.image,
        }
    }
}
/// [Actor] free of generic types and constants
#[derive(Debug, Hash, Default, Clone)]
pub struct PlainActor {
    pub(crate) client: String,
    pub(crate) inputs: Option<Vec<IO>>,
    pub(crate) outputs: Option<Vec<IO>>,
    pub(crate) hash: u64,
    pub(crate) image: Option<String>,
    pub(crate) graph: Option<Graph>,
}
impl PlainActor {
    /// Creates a new [PlainActorBuilder]
    pub fn new(client: impl ToString) -> PlainActorBuilder {
        PlainActorBuilder {
            client: client.to_string(),
            ..Default::default()
        }
    }
    /// Selects the inputs which name does not match any given names
    pub fn filter_inputs_by_name(mut self, names: &[&str]) -> Option<Vec<IO>> {
        self.inputs.take().map(|ios| {
            ios.into_iter()
                .filter(|io| io.filter_by_name(names))
                .collect::<Vec<_>>()
        })
    }
    /// Selects the outputs which name does not match any given names
    pub fn filter_outputs_by_name(mut self, names: &[&str]) -> Option<Vec<IO>> {
        self.outputs.take().map(|ios| {
            ios.into_iter()
                .filter(|io| io.filter_by_name(names))
                .collect::<Vec<_>>()
        })
    }
    /// Takes the inputs out
    pub fn inputs(&mut self) -> Option<Vec<IO>> {
        self.inputs.take()
    }
    /// Takes the outputs out
    pub fn outputs(&mut self) -> Option<Vec<IO>> {
        self.outputs.take()
    }
}

impl<C, const NI: usize, const NO: usize> From<&Actor<C, NI, NO>> for PlainActor
where
    C: TryUpdate,
{
    fn from(actor: &Actor<C, NI, NO>) -> Self {
        Self {
            client: actor.name.as_ref().unwrap_or(&actor.who()).to_owned(),
            // inputs_rate: NI,
            // outputs_rate: NO,
            inputs: actor
                .inputs
                .as_ref()
                .map(|inputs| inputs.iter().map(|o| IO::from((o, NI))).collect()),
            outputs: actor
                .outputs
                .as_ref()
                .map(|outputs| outputs.iter().map(|o| IO::from((o, NO))).collect()),
            hash: 0,
            image: actor.image.as_ref().cloned(),
            graph: None,
        }
    }
}

impl From<(&Box<dyn InputObject>, usize)> for IO {
    fn from((value, r): (&Box<dyn InputObject>, usize)) -> Self {
        if value.capacity().is_some() {
            IO::Regular(IOData::new(value.who(), value.get_hash(), 1, r))
        } else {
            IO::Unbounded(IOData::new(value.who(), value.get_hash(), 1, r))
        }
    }
}

impl From<(&Box<dyn OutputObject>, usize)> for IO {
    fn from((value, r): (&Box<dyn OutputObject>, usize)) -> Self {
        if value.bootstrap() {
            IO::Bootstrap(IOData::new(value.who(), value.get_hash(), value.len(), r))
        } else {
            IO::Regular(IOData::new(value.who(), value.get_hash(), value.len(), r))
        }
    }
}

impl IO {
    pub fn as_formatted_input(&self, actor_hash: u64, color: usize) -> String {
        match self {
            IO::Bootstrap(input) => format!(
                r#"{0} -> {1} [label="{2}", color={3}, style=bold];"#,
                input.hash,
                actor_hash,
                trim(&input.name),
                color
            ),
            IO::Regular(input) => format!(
                r#"{0} -> {1} [label="{2}", color={3}];"#,
                input.hash,
                actor_hash,
                trim(&input.name),
                color
            ),
            IO::Unbounded(input) => format!(
                r#"{0} -> {1} [label="{2}", color={3}, style=dashed];"#,
                input.hash,
                actor_hash,
                trim(&input.name),
                color
            ),
        }
    }
    pub fn as_formatted_output(&self, actor_hash: u64, color: usize) -> String {
        match self {
            IO::Bootstrap(output) => format!(
                r"{0} -> {1} [color={2}, style=bold];",
                actor_hash, output.hash, color
            ),
            IO::Regular(output) => {
                format!("{0} -> {1} [color={2}];", actor_hash, output.hash, color)
            }
            IO::Unbounded(output) => format!(
                r"{0} -> {1} [color={2}, style=dashed];",
                actor_hash, output.hash, color
            ),
        }
    }
}

// pub struct VecIO(Option<Vec<IO>>);

// impl VecIO {
//     pub fn filter(self, tags: Vec<&str>) -> impl Iterator<Item = Option<IO>> + '_ {
//         tags.into_iter().map(move |tag| self.find(tag))
//     }
//     pub fn find(&self, tag: &str) -> Option<IO> {
//         if let Some(io) = &self.0 {
//             io.into_iter()
//                 .find(|&input| input.filter(|x| x.name.contains(tag)))
//                 .cloned()
//         } else {
//             None
//         }
//     }
//     pub fn into_iter(self) -> Box<dyn Iterator<Item = IO>> {
//         match self.0 {
//             Some(data) => Box::new(data.into_iter()),
//             None => Box::new(empty::<IO>()),
//         }
//     }
// }
