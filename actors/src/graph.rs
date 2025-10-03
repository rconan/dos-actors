//! Actors graph

use std::{
    collections::{hash_map::DefaultHasher, HashMap},
    env,
    fs::File,
    hash::{Hash, Hasher},
    io::{self, Write},
    path::Path,
    sync::{LazyLock, Mutex},
};

use crate::{model::PlainModel, trim};
mod render;
pub use render::{Render, RenderError};

#[derive(Debug)]
pub struct ColorMap {
    lookup: HashMap<usize, usize>,
    colors: Vec<usize>,
}
impl Default for ColorMap {
    fn default() -> Self {
        Self::new()
    }
}

impl ColorMap {
    pub fn new() -> Self {
        Self {
            lookup: HashMap::new(),
            colors: (1usize..=8).collect(),
        }
    }
    pub fn get(&mut self, rate: usize) -> usize {
        *self.lookup.entry(rate).or_insert_with(|| {
            let color = self.colors[0];
            self.colors.rotate_left(1);
            color
        })
    }
}
pub static COLORMAP: LazyLock<Mutex<ColorMap>> = LazyLock::new(|| Mutex::new(ColorMap::new()));

#[derive(Debug, thiserror::Error)]
pub enum GraphError {
    #[error("failed to write Graphviz file")]
    ToDot(#[from] io::Error),
}

#[derive(Debug, Default, Clone)]
enum GraphTheme {
    #[default]
    Screen,
    Paper,
}
impl GraphTheme {
    pub fn new() -> Self {
        match env::var("FLOWCHART_THEME") {
            Ok(var) => match var.to_lowercase().as_str() {
                "screen" => Self::Screen,
                "paper" => Self::Paper,
                _ => Self::default(),
            },
            Err(_) => Self::default(),
        }
    }
    pub fn into_string(self, actors: String, inputs: String, outputs: String) -> String {
        match self {
            Self::Screen => format!(
                r#"
    digraph  G {{
      overlap = scale;
      splines = true;
      bgcolor = gray24;
      {{node [shape=box, width=1.5, style="rounded,filled", fillcolor=lightgray]; {};}}
      node [shape=point, fillcolor=gray24, color=lightgray];
    
      /* Outputs */
    {{
      edge [arrowhead=none,colorscheme=dark28,fontsize=9, fontcolor=lightgray,fontname="times:italic"];
      {}
    }}
      /* Inputs */
    {{
      edge [arrowhead=vee, colorscheme=dark28]
      {}
    }}
    }}
    "#,
                actors, outputs, inputs,
            ),
            Self::Paper => format!(
                r#"
    digraph  G {{
      overlap = scale;
      splines = true;
      {{node [shape=box, width=1.5, style="rounded,filled"]; {};}}
      node [shape=point, fillcolor=gray24, color=lightgray];
    
      /* Outputs */
    {{
      edge [arrowhead=none,colorscheme=dark28,fontsize=9,fontname="times:italic"];
      {}
    }}
      /* Inputs */
    {{
      edge [arrowhead=vee, colorscheme=dark28]
      {}
    }}
    }}
    "#,
                actors, outputs, inputs,
            ),
        }
    }
}

/// [Model](crate::model::Model) network mapping
///
/// The structure is used to build a [Graphviz](https://www.graphviz.org/) diagram of a [Model](crate::model::Model).
/// A new [Graph] is created with `Model::graph()`.
///
/// There are 2 themes for the flowcharts: `Screen` with a dark background and `Paper` with a light background.
/// The theme can be set with the environment variable `FLOWCHART_THEME` with the values `Screen` and `Paper`.
///
/// The graph layout is either
///  [dot](https://www.graphviz.org/docs/layouts/dot/),
///  [neato](https://www.graphviz.org/docs/layouts/neato/),
/// or [fdp](https://www.graphviz.org/docs/layouts/fdp/).
/// The default layout is `neato` and it can be change by setting the environment variable `FLOWCHART`
/// to `dot`, `neato` or `fdp`.

#[derive(Debug, Hash, Default, Clone)]
pub struct Graph {
    pub(crate) name: String,
    actors: PlainModel,
    to_dot: bool,
}
impl Graph {
    pub fn new(name: String, actors: impl Into<PlainModel>) -> Self {
        let mut hasher = DefaultHasher::new();
        let mut actors: PlainModel = actors.into();
        actors.iter_mut().for_each(|actor| {
            actor.client = trim(&actor.client);
            actor.hash(&mut hasher);
            actor.hash = hasher.finish();
        });
        Self {
            name,
            actors,
            to_dot: env::var("TO_DOT").is_ok(),
        }
    }
    /// Returns the diagram in the [Graphviz](https://www.graphviz.org/) dot language
    pub fn to_string(&self) -> String {
        let color_map = &*COLORMAP;
        let inputs: Vec<_> = self
            .actors
            .iter()
            .filter_map(|actor| {
                actor.inputs.as_ref().map(|inputs| {
                    inputs
                        .iter()
                        .map(|input| {
                            let color = color_map.lock().unwrap().get(input.rate());
                            input.as_formatted_input(actor.hash, color)
                        })
                        .collect::<Vec<String>>()
                })
            })
            .flatten()
            .collect();
        let outputs: Vec<_> = self
            .actors
            .iter()
            .filter_map(|actor| {
                actor.outputs.as_ref().map(|outputs| {
                    outputs
                        .iter()
                        .map(|output| {
                            let color = color_map.lock().unwrap().get(output.rate());
                            output.as_formatted_output(actor.hash, color)
                        })
                        .collect::<Vec<String>>()
                })
            })
            .flatten()
            .collect();
        GraphTheme::new().into_string(
            self.actors
                .iter()
                .map(|actor| {
                    if let Some(image) = actor.image.as_ref() {
                        format!(
                            r#"{} [label="{}", labelloc=t, image="{}"]"#,
                            actor.hash, actor.client, image
                        )
                    } else {
                        format!(r#"{} [label="{}"]"#, actor.hash, actor.client)
                    }
                })
                .collect::<Vec<String>>()
                .join("; "),
            outputs.join("\n"),
            inputs.join("\n"),
        )
    }
    /// Writes the output of [Graph::to_string()] to a file
    pub fn to_dot(&self) -> std::result::Result<&Self, GraphError> {
        if self.to_dot {
            let data_repo = env::var("DATA_REPO").unwrap_or(".".into());
            let path = Path::new(&data_repo).join(format!("{}.dot", self.name));
            let mut file = File::create(&path)?;
            write!(&mut file, "{}", self.to_string())?;
            for actor in &self.actors {
                if let Some(graph) = actor.graph.as_ref() {
                    graph.to_dot()?;
                }
            }
        }
        Ok(self)
    }
    pub fn walk(&self) -> Render {
        let mut render = Render::from(self);
        for actor in &self.actors {
            if let Some(graph) = actor.graph.as_ref() {
                render
                    .child
                    .get_or_insert(Vec::new())
                    .push(Box::new(graph.walk()));
            }
        }
        log::debug!("{:}", render);
        render
    }
}

#[cfg(test)]
mod tests {
    use super::trim;

    #[test]
    fn parse_client_name() {
        let a = trim("print");
        dbg!(&a);
        let a = trim("a::b::print");
        dbg!(a);
        let a = trim("a::b::print<w::W,q::s::C>");
        dbg!(a);
        // let a = trim("a::b::print<w::W>");
    }
}
