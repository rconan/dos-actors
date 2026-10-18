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

use base64::{Engine, engine::general_purpose::STANDARD};
use serde_json::{Map, Value, json};

use crate::{COLORMAP, IO, graph::Graph, render::RenderError, trim};

const TEMPLATE: &str = include_str!("elk/template.html");
const ELK: &str = include_str!("elk/elk.bundled.js");

/// `dark28` color scheme, the same colors than the Graphviz backend
const DARK28: [&str; 8] = [
    "#1b9e77", "#d95f02", "#7570b3", "#e7298a", "#66a61e", "#e6ab02", "#a6761d", "#666666",
];

/// Returns the media type of an image from its file extension
fn media_type(path: &Path) -> &'static str {
    match path
        .extension()
        .and_then(|ext| ext.to_str())
        .map(|ext| ext.to_lowercase())
        .as_deref()
    {
        Some("png") => "image/png",
        Some("jpg" | "jpeg") => "image/jpeg",
        Some("gif") => "image/gif",
        Some("svg") => "image/svg+xml",
        Some("webp") => "image/webp",
        Some("bmp") => "image/bmp",
        _ => "application/octet-stream",
    }
}

/// Reads an actor image and returns it as a base64 data URI
///
/// The image path is resolved against the working directory and then against `CARGO_MANIFEST_DIR`, if set.
/// Returns `None`, with a warning, if the image is not found
fn data_uri(image: &str) -> Option<String> {
    let path = Path::new(image);
    let candidates = std::iter::once(path.to_path_buf()).chain(
        env::var("CARGO_MANIFEST_DIR")
            .ok()
            .filter(|_| path.is_relative())
            .map(|dir| Path::new(&dir).join(path)),
    );
    for candidate in candidates {
        if let Ok(bytes) = std::fs::read(&candidate) {
            return Some(format!(
                "data:{};base64,{}",
                media_type(&candidate),
                STANDARD.encode(bytes)
            ));
        }
    }
    log::warn!("flowchart image `{image}` not found, the block is drawn without it");
    None
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
                    let channel = channels.entry(hash.clone()).or_insert_with(|| {
                        let color = COLORMAP.lock().unwrap().get(io.rate());
                        json!({
                            "name": trim(io.name()),
                            "color": DARK28[(color - 1) % DARK28.len()],
                            "rate": io.rate(),
                            "bootstrap": false,
                        })
                    });
                    // an output is bootstrapped, the channel is bootstrapped whichever end is seen first
                    if matches!(io, IO::Bootstrap(_)) {
                        channel["bootstrap"] = json!(true);
                    }
                    hash
                })
                .collect()
        };
        let inputs = hashes(&actor.inputs);
        let outputs = hashes(&actor.outputs);
        // an input is unbounded, a channel can feed both bounded and unbounded inputs
        let unbounded: Vec<_> = actor
            .inputs
            .iter()
            .flatten()
            .filter(|io| matches!(io, IO::Unbounded(_)))
            .map(|io| io.hash().to_string())
            .collect();
        actors.push(json!({
            "id": id,
            "label": actor.client,
            "parent": parent,
            "system": actor.graph.is_some(),
            "inputs": inputs,
            "outputs": outputs,
            "unbounded": unbounded,
            "image": actor.image.as_deref().and_then(data_uri),
            "uid": actor.uid.map(|uid| uid.to_string()),
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
///
/// `events` is the JSON of a recorded model run or `null`
pub(crate) fn fill(title: &str, model: &str, events: &str) -> String {
    let mut page = String::with_capacity(TEMPLATE.len() + ELK.len() + model.len() + events.len());
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
        } else if tail.starts_with("__EVENTS__") {
            (events, "__EVENTS__".len())
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
    let title = escape_html(&graph.name().to_uppercase());
    let model = to_json(graph)?;
    let page = fill(&title, &model, "null");
    let mut file = File::create(&path)?;
    file.write_all(page.as_bytes())?;
    #[cfg(feature = "replay")]
    crate::replay::open(path.clone(), title, model);
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
    fn bootstrap_into_unbounded() {
        // the consumer is seen before the producer
        let graph = Graph::new(
            "model".into(),
            vec![
                PlainActor::new("Logger")
                    .inputs(vec![IO::Unbounded(IOData::new("x::Y".into(), 3, 1, 1))])
                    .build(),
                PlainActor::new("Feedback")
                    .outputs(vec![IO::Bootstrap(IOData::new("x::Y".into(), 3, 1, 1))])
                    .build(),
            ],
        );
        let model: Value = serde_json::from_str(&to_json(&graph).unwrap()).unwrap();
        assert_eq!(model["channels"]["3"]["bootstrap"], true);
        assert_eq!(model["actors"][0]["unbounded"], json!(["3"]));
        assert_eq!(model["actors"][1]["unbounded"], json!([]));
    }

    #[test]
    fn image() {
        let dir = env::temp_dir().join(format!("flowchart-elk-image-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let png = dir.join("block.png");
        std::fs::write(&png, b"\x89PNG\r\n\x1a\n").unwrap();
        let graph = Graph::new(
            "model".into(),
            vec![
                PlainActor::new("A").image(png.to_str().unwrap()).build(),
                PlainActor::new("B")
                    .image(dir.join("missing.png").to_str().unwrap())
                    .build(),
                PlainActor::new("C").build(),
            ],
        );
        let model: Value = serde_json::from_str(&to_json(&graph).unwrap()).unwrap();
        let actors = model["actors"].as_array().unwrap();
        assert_eq!(actors[0]["image"], "data:image/png;base64,iVBORw0KGgo=");
        assert!(actors[1]["image"].is_null());
        assert!(actors[2]["image"].is_null());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn page() {
        let page = fill("MODEL", &to_json(&graph()).unwrap(), "null");
        assert!(
            !page.contains("__MODEL__")
                && !page.contains("__TITLE__")
                && !page.contains("__ELK__")
                && !page.contains("__EVENTS__")
        );
        assert!(page.contains("SPDX-License-Identifier: EPL-2.0"));
        assert_eq!(page.matches("</script>").count(), 2);
    }
}
