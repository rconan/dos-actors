//! # Model framework
//!
//! The model module define the interface to build a [Model].
//!
//! Any structure that implements [Task], and its super trait [Check], can be part of an actors [Model].
//!
//! [Model]: crate::model::Model

use std::path::PathBuf;

use crate::graph::GraphError;
use crate::model::{Model, UnknownOrReady};
use crate::system::System;
use crate::{
    actor::PlainActor,
    graph::{self, Graph},
    model::{self, PlainModel},
    ActorError,
};

#[derive(Debug, thiserror::Error)]
pub enum CheckError {
    #[error("error in Task from Actor")]
    FromActor(#[from] ActorError),
    #[error("error in Task from Model")]
    FromModel(#[from] model::ModelError),
}

/// Interface for model verification routines
///
pub trait Check {
    /// Validates the inputs
    ///
    /// Returns en error if there are some inputs but the inputs rate is zero
    /// or if there are no inputs and the inputs rate is positive
    fn check_inputs(&self) -> std::result::Result<(), CheckError>;
    /// Validates the outputs
    ///
    /// Returns en error if there are some outputs but the outputs rate is zero
    /// or if there are no outputs and the outputs rate is positive
    fn check_outputs(&self) -> std::result::Result<(), CheckError>;
    /// Return the number of inputs
    fn n_inputs(&self) -> usize;
    /// Return the number of outputs
    fn n_outputs(&self) -> usize;
    /// Return the hash # of inputs
    fn inputs_hashes(&self) -> Vec<u64>;
    /// Return the hash # of outputs
    fn outputs_hashes(&self) -> Vec<u64>;
    fn _as_plain(&self) -> PlainActor;
    fn is_system(&self) -> bool {
        false
    }
}

#[derive(Debug, thiserror::Error)]
pub enum TaskError {
    #[error("error in Task from Actor")]
    FromActor(#[from] ActorError),
    #[error("error in Task from Model")]
    FromModel(#[from] model::ModelError),
    #[error(transparent)]
    Other(#[from] Box<dyn std::error::Error + Send + Sync>),
}

/// Interface for running model components
#[async_trait::async_trait]
pub trait Task: Check + std::fmt::Display + Send + Sync {
    /// Runs the [Actor](crate::actor::Actor) infinite loop
    ///
    /// The loop ends when the client data is [None] or when either the sending of receiving
    /// end of a channel is dropped
    async fn async_run(&mut self) -> std::result::Result<(), TaskError>;
    /// Run the actor loop in a dedicated thread
    fn spawn(self) -> tokio::task::JoinHandle<std::result::Result<(), TaskError>>
    where
        Self: Sized + 'static,
    {
        tokio::spawn(async move { Box::new(self).task().await })
    }
    /// Run the actor loop
    async fn task(self: Box<Self>) -> std::result::Result<(), TaskError>;
    fn as_plain(&self) -> PlainActor;
}

/// Flowchart name
pub trait GetName {
    /// Returns the flowchart name
    fn get_name(&self) -> String {
        "integrated_model".into()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum FlowChartError {
    #[error("no graph to walk, may be there is no actors!")]
    NoGraph,
    #[error("failed to write SVG charts")]
    Rendering(#[from] graph::RenderError),
    #[error("failed to process graph")]
    Graph(#[from] GraphError),
}

/// Actors flowchart interface
pub trait FlowChart: GetName {
    /// Returns the actors network graph
    fn graph(&self) -> Option<Graph>;
    /// Writes the flowchart to an HTML file
    ///
    /// Optionnaly, one can get the [Graphviz](https://www.graphviz.org/) dot files to
    /// be written as well by setting the environment variable `TO_DOT` to 1.
    fn to_html(&self) -> std::result::Result<PathBuf, FlowChartError> {
        Ok(self
            .graph()
            .ok_or(FlowChartError::NoGraph)?
            .to_dot()?
            .walk()
            .into_svg()?
            .to_html()?)
    }

    /// Writes the actors flowchart to an HTML file
    ///
    /// The flowchart file is written either in the current directory
    /// or in the directory give by the environment variable `DATA_REPO`.
    /// The flowchart is created with [Graphviz](https://www.graphviz.org/) neato filter,
    /// other filters can be specified with the environment variable `FLOWCHART`
    fn flowchart(self) -> Self
    where
        Self: Sized,
    {
        if let Err(e) = self.to_html() {
            println!("failed to write flowchart Web page caused by:\n {e:?}");
        }
        self
    }
    /// Writes the actors flowchart to an HTML file and open it in the default browser
    fn flowchart_open(self) -> Self
    where
        Self: Sized,
    {
        match self.to_html() {
            Ok(path) => {
                if let Err(e) = open::that(path) {
                    println!("failed to open flowchart Web page caused by:\n {e:?}");
                }
            }
            Err(e) => println!("failed to write flowchart Web page caused by:\n {e:?}"),
        };
        self
    }
}
impl<S: UnknownOrReady> FlowChart for Model<S>
// where
//     for<'a> &'a T: IntoIterator<Item = PlainActor>,
{
    fn graph(&self) -> Option<Graph> {
        // let actors: Vec<_> = self.into_iter().collect();
        let actors = PlainModel::from_iter(self);
        if actors.is_empty() {
            None
        } else {
            Some(Graph::new(self.get_name(), actors))
        }
    }
    /*
    fn flowchart(self) -> Self {
        match self.graph() {
            None => println!("no graph to make, may be there is no actors!"),
            Some(graph) => match graph.walk().into_svg() {
                Ok(r) => {
                    if let Err(e) = r.to_html() {
                        println!("failed to write flowchart Web page caused by:\n {e}");
                    }
                }
                Err(e) => println!("failed to write SVG charts caused by:\n {e}"),
            },
        }
        self
    } */

    /*     fn flowchart_open(self) -> Self {
        match self.graph() {
            None => println!("no graph to make, may be there is no actors!"),
            Some(graph) => match graph.walk().into_svg() {
                Ok(r) => match r.to_html() {
                    Ok(path) => {
                        if let Err(e) = open::that(path) {
                            println!("failed to open flowchart Web page caused by:\n {e}");
                        }
                    }
                    Err(e) => println!("failed to write flowchart Web page caused by:\n {e}"),
                },
                Err(e) => println!("failed to write SVG charts caused by:\n {e}"),
            },
        }
        self
    }*/
}

// pub trait SystemFlowChart {
//     fn graph(&self) -> Option<Graph>;
//     // fn flowchart(&self) -> &Self;
// }
impl<T: System> FlowChart for T
where
    for<'a> &'a T: IntoIterator<Item = Box<&'a dyn Check>>,
{
    fn graph(&self) -> Option<Graph> {
        // let actors: Vec<_> = self.into_iter().map(|x| x._as_plain()).collect();
        let actors = PlainModel::from_iter(self);
        if actors.is_empty() {
            None
        } else {
            Some(Graph::new(self.get_name(), actors))
        }
    }

    /*     fn flowchart(&self) -> &Self {
        match self.graph() {
            None => println!("no graph to make, may be there is no actors!"),
            Some(graph) => match graph.walk().into_svg() {
                Ok(r) => {
                    if let Err(e) = r.to_html() {
                        println!("failed to write flowchart Web page caused by:\n {e}");
                    }
                }
                Err(e) => println!("failed to write SVG charts caused by:\n {e}"),
            },
        }
        &self
    } */
}

#[cfg(test)]
mod tests {
    use std::process::{Command, Stdio};
    #[test]
    fn pipe() {
        let graph = Command::new("echo")
            .arg(r#"digraph G { a -> b }"#)
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        let svg = Command::new("dot")
            .arg("-Tsvg")
            .stdin(Stdio::from(graph.stdout.unwrap()))
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        let output = svg.wait_with_output().unwrap();
        let result = std::str::from_utf8(&output.stdout).unwrap();
        let svg = result.lines().skip(6).collect::<Vec<_>>().join("");
        println!("{:#}", &svg);
    }
}
