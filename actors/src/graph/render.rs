use std::{
    env,
    fmt::Display,
    fs::File,
    io::{ErrorKind, Write},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    thread,
};

use svg::{
    node::{
        element::tag::{self, Type},
        Attributes,
    },
    parser::Event,
    Parser,
};

use crate::graph::Graph;

const HEAD: &str = r#"
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>GRAPH</title>
    <style>
        body {
            background-color: #3d3d3d;
        }

        .svg-container {
            display: flex;
            justify-content: space-around;
            margin-top: 20px;
        }

        .info-container {
            display: flex;
            justify-content: space-around;
            font-family: monospace
        }

        svg {
            width: auto;
            /* Adjust the width as needed */
            height: auto;
        }

        .hidden {
            display: none;
        }

        .highlighted {
            stroke: hsla(348, 83%, 47%, 0.5);
            /* Set the stroke color to yellow */
            /* stroke-width: 2; */
            /* Set the stroke width */
        }
    </style>
</head>
"#;

#[derive(Debug, thiserror::Error)]
pub enum RenderError {
    #[error("failed to write flowchart: {0}")]
    IO(#[from] std::io::Error),
    #[error("failed to convert to string")]
    Utf(#[from] std::string::FromUtf8Error),
    #[error("unknown flowchart layout in FLOWCHART, expected dot, neato, fdp or none")]
    Layout,
    #[error("flowchart is disabled (FLOWCHART is empty or none)")]
    Disabled,
    #[error("Graphviz `{0}` not found, is Graphviz installed?")]
    GraphvizNotFound(String),
    #[error("Graphviz `{layout}` failed on model `{name}`:\n{stderr}")]
    Graphviz {
        layout: String,
        name: String,
        stderr: String,
    },
}
type Result<T> = std::result::Result<T, RenderError>;

#[derive(Debug, Clone)]
pub struct Render {
    name: String,
    render: String,
    pub(crate) child: Option<Vec<Box<Render>>>,
}
impl From<&Graph> for Render {
    fn from(graph: &Graph) -> Self {
        Self {
            name: graph.name.clone(),
            render: graph.to_string(),
            child: None,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
#[allow(dead_code)]
enum GraphLayout {
    Dot,
    #[default]
    Neato,
    Fdp,
}
impl Display for GraphLayout {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GraphLayout::Dot => write!(f, "dot"),
            GraphLayout::Neato => write!(f, "neato"),
            GraphLayout::Fdp => write!(f, "fdp"),
        }
    }
}
impl GraphLayout {
    /// Reads the layout from the environment variable `FLOWCHART`
    ///
    /// Returns `None` if `FLOWCHART` is empty or `none`, the default layout if `FLOWCHART` is not set
    /// and an error if the layout is unknown
    pub fn new() -> Result<Option<Self>> {
        match env::var("FLOWCHART") {
            Ok(var) => match var.trim().to_lowercase().as_str() {
                "dot" => Ok(Some(Self::Dot)),
                "neato" => Ok(Some(Self::Neato)),
                "fdp" => Ok(Some(Self::Fdp)),
                "" | "none" => Ok(None),
                _ => Err(RenderError::Layout),
            },
            Err(_) => Ok(Some(Self::default())),
        }
    }
    /// Runs the Graphviz layout program on the `dot` graph and returns the SVG output
    fn run(&self, name: &str, dot: &str) -> Result<String> {
        let mut graphviz = Command::new(self.to_string())
            .arg("-Tsvg")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| match e.kind() {
                ErrorKind::NotFound => RenderError::GraphvizNotFound(self.to_string()),
                _ => e.into(),
            })?;
        // the graph is written to stdin from another thread so that Graphviz is never blocked
        // writing to a stdout that is not being read
        let mut stdin = graphviz.stdin.take().expect("Graphviz stdin is piped");
        let dot = dot.to_string();
        let writer = thread::spawn(move || stdin.write_all(dot.as_bytes()));
        let output = graphviz.wait_with_output()?;
        // a write error (e.g. broken pipe) shows up as a Graphviz failure below
        let _ = writer.join();
        let stderr = String::from_utf8_lossy(&output.stderr).trim().to_string();
        if output.status.success() {
            if !stderr.is_empty() {
                log::warn!("Graphviz `{self}` on model `{name}`: {stderr}");
            }
            Ok(String::from_utf8(output.stdout)?)
        } else {
            Err(RenderError::Graphviz {
                layout: self.to_string(),
                name: name.to_string(),
                stderr,
            })
        }
    }
}

/// Returns `false` if the flowchart is disabled by setting `FLOWCHART` to an empty value or to `none`
pub fn flowchart_enabled() -> Result<bool> {
    GraphLayout::new().map(|layout| layout.is_some())
}
impl Render {
    fn id(&self) -> String {
        use std::hash::{DefaultHasher, Hash, Hasher};
        let mut hasher = DefaultHasher::new();
        self.name.hash(&mut hasher);
        let sh = hasher.finish();
        format!("id{sh:x}")
    }
    /// Renders flowchart to SVG
    pub fn into_svg(&mut self) -> Result<&mut Self> {
        let graph_layout = GraphLayout::new()?.ok_or(RenderError::Disabled)?;
        let result = match graph_layout.run(&self.name, &self.render) {
            Ok(svg) => svg,
            // falls back to the dot layout if the requested layout fails
            Err(e) if graph_layout != GraphLayout::Dot => {
                log::warn!("{e}, falling back to `dot`");
                GraphLayout::Dot.run(&self.name, &self.render)?
            }
            Err(e) => return Err(e),
        };
        log::debug!("{:}", &result[..result.len().min(64)]);
        self.render = result
            .lines()
            .skip(6)
            .collect::<Vec<_>>()
            .join("\n")
            .replace(r#"g id="node"#, &format!(r#"g id="{}_node"#, self.id()));
        self.child
            .as_mut()
            .map(|child| {
                child
                    .iter_mut()
                    .map(|child| child.into_svg())
                    .collect::<Result<Vec<_>>>()
            })
            .transpose()?;
        log::debug!("{:}", self);
        Ok(self)
    }
    fn hover(&self, child: &str, element: &str) -> String {
        format!(
            r#"
const {0} = document.getElementById('{0}');
{0}.addEventListener('mouseenter', function () {{
    {0}.classList.add('highlighted');
}});
{0}.addEventListener('mouseleave', function () {{
    {0}.classList.remove('highlighted');
}});
{0}.addEventListener('click', function () {{
    // Hide graph1
    {1}.classList.add('hidden');
    // Show graph2 and give it the focus for the Escape key
    {2}.classList.remove('hidden');
    {2}.focus();
}});
{2}.addEventListener('keydown', function (event) {{
    if (event.key === 'Escape') {{
        // Show graph1 and give it the focus for the Escape key
        {1}.classList.remove('hidden');
        {1}.focus();
        // Hide graph2
        {2}.classList.add('hidden');
    }}
}});
        "#,
            element,
            self.id(),
            child
        )
    }
    /// Parses SVG diagram to identify SVG node names of children graph
    fn parse(&self) -> Option<String> {
        let Some(child) = &self.child else {
            return None;
        };
        let parser = Parser::new(&self.render);
        let mut h = vec![];
        let mut attributes = Attributes::new();
        for event in parser {
            match event {
                Event::Tag(tag::Group, Type::Start, a) => {
                    attributes = a;
                }
                Event::Text(text) => {
                    for child in child {
                        if html_escape::decode_html_entities(text) == child.name {
                            log::debug!("{:?}", (&child.name, child.id()));
                            h.push(attributes.get("id").map(|id| self.hover(&child.id(), id)));
                        }
                    }
                }
                _ => {}
            }
        }
        if h.is_empty() {
            None
        } else {
            h.into_iter()
                .collect::<Option<Vec<String>>>()
                .map(|h| h.join("\n"))
        }
    }
    /// Writes highlight script to file
    fn script_child_hover(&self, file: &mut File) -> Result<()> {
        log::debug!("{:?}", (&self.name, self.id()));
        if let Some(h) = self.parse() {
            writeln!(file, "{}", h)?;
        }
        let Some(child) = &self.child else {
            return Ok(());
        };
        for child in child {
            child.script_child_hover(file)?;
        }
        Ok(())
    }
    /// Writes SVG diagram to file
    fn child_svg(&self, file: &mut File, class: Option<&str>) -> Result<()> {
        match class {
            Some(class) => writeln!(
                file,
                "{}",
                self.render.replace(
                    "<svg",
                    &format!(r#"<svg id="{}" tabindex="0" class="{}""#, self.id(), class)
                )
            )?,
            None => writeln!(
                file,
                "{}",
                self.render
                    .replace("<svg", &format!(r#"<svg id="{}" tabindex="0""#, self.id()))
            )?,
        }
        let Some(child) = &self.child else {
            return Ok(());
        };
        for child in child {
            child.child_svg(file, Some("hidden"))?;
        }
        Ok(())
    }
    /// Writes get element by id script to file
    fn script_child_const(&self, file: &mut File) -> Result<()> {
        let Some(child) = &self.child else {
            return Ok(());
        };
        for child in child {
            writeln!(
                file,
                "const {0} = document.getElementById('{0}');",
                child.id()
            )?;
            child.script_child_const(file)?;
        }
        Ok(())
    }
    /*     /// Return the names of all children
    fn get_children_name(&self, names: &mut Vec<String>) {
        let Some(child) = &self.child else {
            return;
        };
        for child in child {
            names.push(child.name.clone());
            child.get_children_name(names);
        }
    } */
    /// Return the ids of all children
    fn get_children_id(&self, ids: &mut Vec<String>) {
        let Some(child) = &self.child else {
            return;
        };
        for child in child {
            ids.push(child.id());
            child.get_children_id(ids);
        }
    }
    /// Homing script
    fn script_home(&self) -> String {
        let mut ids = vec![];
        self.get_children_id(&mut ids);
        format!(
            r#"
document.addEventListener('keydown', function (event) {{
    if (event.key === 'Home') {{
// Show graph1
{0}.classList.remove('hidden');
{0}.focus();
// Hide other graphs
{1}
    }}
}});
        "#,
            self.id(),
            ids.into_iter()
                .map(|id| format!("{0}.classList.add('hidden');", id))
                .collect::<Vec<String>>()
                .join("\n")
        )
    }
    /// Writes the flowchart to an HTML file
    pub fn to_html(&self) -> Result<PathBuf> {
        log::debug!("{:}", self);
        let data_repo = env::var("DATA_REPO").unwrap_or(".".into());
        let path = Path::new(&data_repo).join(format!("{}_flowchart.html", self.name));
        let mut file = File::create(&path).unwrap();
        writeln!(file, "<!DOCTYPE html>")?;
        writeln!(file, r#"<html lang="en">"#)?;
        writeln!(
            file,
            "{}",
            HEAD.replace("GRAPH", &format!("{} Flowchart", self.name.to_uppercase()))
        )?;
        writeln!(file, "<body>")?;
        writeln!(
            file,
            r#"<div class="info-container">Left Click on System: show ; Left Click followed by Escape key: back-up ; Home key: back to root</div>"#
        )?;

        writeln!(file, r#"    <div class="svg-container">"#)?;
        self.child_svg(&mut file, None)?;
        writeln!(file, "      </div>")?;
        writeln!(file, "<script>")?;
        writeln!(
            file,
            "const {0} = document.getElementById('{0}');",
            self.id()
        )?;
        write!(file, "{}", self.script_home())?;
        self.script_child_const(&mut file)?;
        self.script_child_hover(&mut file)?;
        writeln!(file, "</script>")?;
        writeln!(file, "</body>")?;
        writeln!(file, "</html>")?;
        Ok(path)
    }
}

impl Display for Render {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "==>> {}", self.name)?;
        // writeln!(f, "{} ...", &self.render[..self.render.len().min(64)])?;
        if let Some(child) = &self.child {
            for (i, child) in child.iter().enumerate() {
                writeln!(f, "{} child #{i}", self.name)?;
                writeln!(f, "{}", child)?;
            }
        }
        writeln!(f, " <<== {}", self.name)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn hash() {
        use std::hash::{DefaultHasher, Hash, Hasher};
        let mut hasher = DefaultHasher::new();
        let s = String::from("M1@80");
        s.hash(&mut hasher);
        let sh = hasher.finish();
        println!("{s} -> {sh:x}");
        let s = String::from("GMT Servo-Mechanisms (M1@80)");
        s.hash(&mut hasher);
        let sh = hasher.finish();
        println!("{s} -> {sh:x}");
    }
}
