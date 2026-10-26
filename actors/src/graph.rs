//! Actors graph
//!
//! The flowchart types are defined in the crate [gmt_dos_actors_flowchart] and re-exported here.

#[cfg(feature = "flowchart-graphviz")]
pub use gmt_dos_actors_flowchart::Render;
#[cfg(feature = "flowchart-replay")]
pub use gmt_dos_actors_flowchart::replay;
pub use gmt_dos_actors_flowchart::{
    COLORMAP, ColorMap, Graph, GraphError, RenderError, flowchart_enabled,
};

/// Chains the runs of the next models into a single flowchart replay
///
/// The models that run one after the other are recorded into the replay of the last one,
/// the clients shared by the models being the same actors (see `replay::chain`).
/// It does nothing without the feature `flowchart-replay`.
pub fn chain_replay() {
    #[cfg(feature = "flowchart-replay")]
    gmt_dos_actors_flowchart::replay::chain();
}
