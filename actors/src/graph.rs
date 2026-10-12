//! Actors graph
//!
//! The flowchart types are defined in the crate [gmt_dos_actors_flowchart] and re-exported here.

#[cfg(feature = "flowchart-graphviz")]
pub use gmt_dos_actors_flowchart::Render;
pub use gmt_dos_actors_flowchart::{
    flowchart_enabled, ColorMap, Graph, GraphError, RenderError, COLORMAP,
};
