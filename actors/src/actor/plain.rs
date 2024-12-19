use std::{iter::empty, vec};

use interface::{Update, Who};

use crate::{
    actor::io::{InputObject, OutputObject},
    graph::Graph,
    trim,
};

use super::Actor;

#[derive(Debug, Hash, Clone)]
#[doc(hidden)]
pub struct IOData {
    pub name: String,
    pub hash: u64,
    pub n: usize,
}
impl IOData {
    pub fn new(name: String, hash: u64, n: usize) -> Self {
        Self { name, hash, n }
    }
    pub fn hash(&self) -> u64 {
        self.hash
    }
}
impl PartialEq<u64> for IOData {
    fn eq(&self, other: &u64) -> bool {
        self.hash == *other
    }
}
#[derive(Debug, Hash, Clone)]
#[doc(hidden)]
pub enum IO {
    Bootstrap(IOData),
    Regular(IOData),
    Unbounded(IOData),
}
impl IO {
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
    pub fn filter_by_name(&self, names: &[&str]) -> bool {
        if self.len() > 1 {
            return true;
        }
        let io_name = self.name();
        !names.into_iter().any(|name| io_name.contains(name))
    }
    pub fn len(&self) -> usize {
        match self {
            IO::Bootstrap(data) => data.n,
            IO::Regular(data) => data.n,
            IO::Unbounded(data) => data.n,
        }
    }
    pub fn name(&self) -> &str {
        match self {
            IO::Bootstrap(data) => &data.name,
            IO::Regular(data) => &data.name,
            IO::Unbounded(data) => &data.name,
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

#[derive(Debug, Hash, Default, Clone)]
#[doc(hidden)]
pub struct PlainActor {
    pub client: String,
    pub inputs_rate: usize,
    pub outputs_rate: usize,
    pub inputs: Option<Vec<IO>>,
    pub outputs: Option<Vec<IO>>,
    pub hash: u64,
    pub image: Option<String>,
    pub graph: Option<Graph>,
}
impl PlainActor {
    pub fn filter_inputs_by_name(mut self, names: &[&str]) -> Option<Vec<IO>> {
        self.inputs
            .take()
            .map(|ios| {
                ios.into_iter()
                    .filter(|io| io.filter_by_name(names))
                    .collect::<Vec<_>>()
            })
            .and_then(|ios| if ios.is_empty() { None } else { Some(ios) })
    }
    pub fn inputs(&mut self) -> VecIO {
        VecIO(self.inputs.take())
    }
    pub fn outputs(&mut self) -> VecIO {
        VecIO(self.outputs.take())
    }
}

impl<C, const NI: usize, const NO: usize> From<&Actor<C, NI, NO>> for PlainActor
where
    C: Update,
{
    fn from(actor: &Actor<C, NI, NO>) -> Self {
        Self {
            client: actor.name.as_ref().unwrap_or(&actor.who()).to_owned(),
            inputs_rate: NI,
            outputs_rate: NO,
            inputs: actor
                .inputs
                .as_ref()
                .map(|inputs| inputs.iter().map(|o| IO::from(o)).collect()),
            outputs: actor
                .outputs
                .as_ref()
                .map(|outputs| outputs.iter().map(|o| IO::from(o)).collect()),
            hash: 0,
            image: actor.image.as_ref().cloned(),
            graph: None,
        }
    }
}

impl From<&Box<dyn InputObject>> for IO {
    fn from(value: &Box<dyn InputObject>) -> Self {
        if let Some(_) = value.capacity() {
            IO::Regular(IOData::new(value.who(), value.get_hash(), 1))
        } else {
            IO::Unbounded(IOData::new(value.who(), value.get_hash(), 1))
        }
    }
}

impl From<&Box<dyn OutputObject>> for IO {
    fn from(value: &Box<dyn OutputObject>) -> Self {
        if value.bootstrap() {
            IO::Bootstrap(IOData::new(value.who(), value.get_hash(), value.len()))
        } else {
            IO::Regular(IOData::new(value.who(), value.get_hash(), value.len()))
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

pub struct VecIO(Option<Vec<IO>>);

impl VecIO {
    pub fn filter(self, tags: Vec<&str>) -> impl Iterator<Item = Option<IO>> + '_ {
        tags.into_iter().map(move |tag| self.find(tag))
    }
    pub fn find(&self, tag: &str) -> Option<IO> {
        if let Some(io) = &self.0 {
            io.into_iter()
                .find(|&input| input.filter(|x| x.name.contains(tag)))
                .cloned()
        } else {
            None
        }
    }
    pub fn into_iter(self) -> Box<dyn Iterator<Item = IO>> {
        match self.0 {
            Some(data) => Box::new(data.into_iter()),
            None => Box::new(empty::<IO>()),
        }
    }
}
