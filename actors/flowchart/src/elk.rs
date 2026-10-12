//! ELK backend
//!
//! The flowchart is written as an HTML page that embeds the model as JSON
//! and lays it out in the browser with [elkjs](https://github.com/kieler/elkjs)

use std::{
    env,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
};

use serde_json::{Map, Value, json};

use crate::{COLORMAP, IO, graph::Graph, render::RenderError, trim};

const TEMPLATE: &str = include_str!("elk/template.html");
const ELK: &str = include_str!("elk/elk.bundled.js");

/// `dark28` color scheme, the same colors than the Graphviz backend
const DARK28: [&str; 8] = [
    "#1b9e77", "#d95f02", "#7570b3", "#e7298a", "#66a61e", "#e6ab02", "#a6761d", "#666666",
];

fn style(io: &IO) -> &'static str {
    match io {
        IO::Bootstrap(_) => "bold",
        IO::Regular(_) => "solid",
        IO::Unbounded(_) => "dashed",
    }
}

/// Adds the actors of a graph, and recursively of the graphs of its systems, to the model
fn walk(
    graph: &Graph,
    parent: Option<&str>,
    actors: &mut Vec<Value>,
    channels: &mut Map<String, Value>,
) {
    for actor in graph.actors().iter() {
        let id = match parent {
            Some(parent) => format!("{parent}/{}", actor.hash),
            None => format!("{}/{}", graph.name(), actor.hash),
        };
        let mut hashes = |ios: &Option<Vec<IO>>| -> Vec<String> {
            ios.iter()
                .flatten()
                .map(|io| {
                    let hash = io.hash().to_string();
                    if !channels.contains_key(&hash) {
                        let color = COLORMAP.lock().unwrap().get(io.rate());
                        channels.insert(
                            hash.clone(),
                            json!({
                                "name": trim(io.name()),
                                "color": DARK28[(color - 1) % DARK28.len()],
                                "style": style(io),
                                "rate": io.rate(),
                            }),
                        );
                    }
                    hash
                })
                .collect()
        };
        let inputs = hashes(&actor.inputs);
        let outputs = hashes(&actor.outputs);
        actors.push(json!({
            "id": id,
            "label": actor.client,
            "parent": parent,
            "system": actor.graph.is_some(),
            "inputs": inputs,
            "outputs": outputs,
        }));
        if let Some(graph) = actor.graph.as_ref() {
            walk(graph, Some(&id), actors, channels);
        }
    }
}

/// Returns the model of a [Graph] as JSON
pub fn to_json(graph: &Graph) -> Result<String, RenderError> {
    let mut actors = vec![];
    let mut channels = Map::new();
    walk(graph, None, &mut actors, &mut channels);
    let model = json!({
        "name": graph.name(),
        "actors": actors,
        "channels": channels,
    });
    // escapes `</` so that the JSON can never close the script element
    Ok(serde_json::to_string(&model)?.replace("</", r"<\/"))
}

fn escape_html(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// Fills the template placeholders in a single pass
fn fill(title: &str, model: &str) -> String {
    let mut page = String::with_capacity(TEMPLATE.len() + ELK.len() + model.len());
    let mut rest = TEMPLATE;
    while let Some(i) = rest.find("__") {
        page.push_str(&rest[..i]);
        let tail = &rest[i..];
        let (value, n) = if tail.starts_with("__TITLE__") {
            (title, "__TITLE__".len())
        } else if tail.starts_with("__MODEL__") {
            (model, "__MODEL__".len())
        } else if tail.starts_with("__ELK__") {
            (ELK, "__ELK__".len())
        } else {
            ("__", 2)
        };
        page.push_str(value);
        rest = &tail[n..];
    }
    page.push_str(rest);
    page
}

/// Writes the ELK flowchart of a [Graph] to an HTML file
pub fn to_html(graph: &Graph) -> Result<PathBuf, RenderError> {
    let data_repo = env::var("DATA_REPO").unwrap_or(".".into());
    let path = Path::new(&data_repo).join(format!("{}_flowchart.html", graph.name()));
    let page = fill(&escape_html(&graph.name().to_uppercase()), &to_json(graph)?);
    let mut file = File::create(&path)?;
    file.write_all(page.as_bytes())?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{IOData, PlainActor};

    fn graph() -> Graph {
        let io = |name: &str, hash: u64, rate: usize| {
            IO::Regular(IOData::new(name.into(), hash, 1, rate))
        };
        let child = Graph::new(
            "sys".into(),
            vec![
                PlainActor::new("a::B")
                    .inputs(vec![io("x::In", 1, 1)])
                    .build(),
                PlainActor::new("C")
                    .outputs(vec![io("x::Out</script>", 2, 10)])
                    .build(),
            ],
        );
        Graph::new(
            "model".into(),
            vec![
                PlainActor::new("Source")
                    .outputs(vec![io("x::In", 1, 1)])
                    .build(),
                PlainActor::new("System")
                    .inputs(vec![io("x::In", 1, 1)])
                    .outputs(vec![io("x::Out</script>", 2, 10)])
                    .graph(Some(child))
                    .build(),
            ],
        )
    }

    #[test]
    fn json() {
        let json = to_json(&graph()).unwrap();
        assert!(!json.contains("</"));
        let model: Value = serde_json::from_str(&json).unwrap();
        let actors = model["actors"].as_array().unwrap();
        assert_eq!(actors.len(), 4);
        assert_eq!(actors[1]["system"], true);
        assert_eq!(actors[2]["parent"], actors[1]["id"]);
        assert_eq!(model["channels"]["2"]["name"], "Out</script>");
        assert_eq!(model["channels"]["2"]["rate"], 10);
    }

    #[test]
    fn page() {
        let page = fill("MODEL", &to_json(&graph()).unwrap());
        assert!(
            !page.contains("__MODEL__") && !page.contains("__TITLE__") && !page.contains("__ELK__")
        );
        assert!(page.contains("SPDX-License-Identifier: EPL-2.0"));
        assert_eq!(page.matches("</script>").count(), 2);
    }
}
