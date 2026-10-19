/*!
# GMT DOS Actors flowchart

Flowcharts of GMT DOS actors models.

A model is turned into a [PlainModel], a collection of [PlainActor]s free of generic types and constants,
from which a [Graph] is built and rendered into a Web page with one of the backends:
 * `graphviz` (default feature): the graph is laid out with [Graphviz](https://www.graphviz.org/)
   and written as a collection of SVG diagrams
 * `elk`: the graph is written as JSON and laid out in the browser with [elkjs](https://github.com/kieler/elkjs)

The backend is selected with the environment variable `FLOWCHART`:
 * `dot`, `neato` or `fdp`: Graphviz with the given layout
 * `elk`: ELK
 * empty or `none`: the flowchart is disabled
 * unset: Graphviz with the `neato` layout if the `graphviz` feature is enabled, otherwise ELK

With the `replay` feature, a model run is recorded and written into the ELK flowchart
to be played back step by step in the browser (see the `replay` module).
With the `scope` feature, the signals saved in parquet files during the run are embedded into the replay page
with the `flowchart-scope` command and displayed as time series or images in sync with the replay (see the `scope` module).
*/

mod graph;
mod model;
mod plain;
mod render;

#[cfg(feature = "elk")]
mod elk;
#[cfg(feature = "replay")]
pub mod replay;
#[cfg(feature = "scope")]
pub mod scope;

pub use graph::{COLORMAP, ColorMap, Graph, GraphError};
pub use model::PlainModel;
pub use plain::{IO, IOData, PlainActor, PlainActorBuilder};
#[cfg(feature = "graphviz")]
pub use render::Render;
pub use render::{RenderError, flowchart_enabled};

#[cfg(not(any(feature = "graphviz", feature = "elk")))]
compile_error!(
    "gmt_dos-actors_flowchart requires at least one of the features `graphviz` or `elk`"
);

/// Strips the paths from a type name, including the paths of the generic parameters
pub fn trim(name: &str) -> String {
    if let Some((prefix, suffix)) = name.split_once('<') {
        let generics: Vec<_> = suffix.split(',').map(trim).collect();
        format!("{}<{}", trim(prefix), generics.join(","))
    } else if let Some((_, suffix)) = name.rsplit_once("::") {
        suffix.into()
    } else {
        name.into()
    }
}
