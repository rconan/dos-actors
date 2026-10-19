//! Signals of a model run embedded into the replay page (feature `scope`)
//!
//! The signals saved in parquet files by the Arrow loggers (`gmt_dos-clients_arrow`) are embedded into the
//! ELK flowchart page of the recorded model run, so the page, a single self-contained file, can display them
//! as time series or images in sync with the replay.
//!
//! A parquet file is matched to the logger actor with the same name (`model_data_1.parquet` is logged by
//! `model_data_1`) and a column to the input of the logger with the same name.
//! The rows of a column are the data received by the logger, the k-th row is the k-th data sent on that
//! channel, starting with the bootstrapped data if any, so its sample step is given by the recorded run.
//!
//! The values are embedded as float32. If the page would exceed the size budget,
//! the time series are thinned and the images are downsampled.

use std::{
    collections::HashMap,
    fs::{self, File},
    path::{Path, PathBuf},
};

use apache_arrow::{
    array::{Array, AsArray},
    datatypes::{Float32Type, Float64Type},
};
use base64::{Engine, engine::general_purpose::STANDARD};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use serde_json::{Value, json};

use crate::replay::Runs;

/// Default size budget of the embedded signals [bytes]
pub const MAX_SIZE: usize = 64 * 1024 * 1024;

#[derive(Debug, thiserror::Error)]
pub enum ScopeError {
    #[error("failed to read or write file: {0}")]
    Io(#[from] std::io::Error),
    #[error("failed to read the parquet file: {0}")]
    Parquet(#[from] parquet::errors::ParquetError),
    #[error("failed to read the Arrow data: {0}")]
    Arrow(#[from] apache_arrow::error::ArrowError),
    #[error("invalid JSON in the flowchart page: {0}")]
    Json(#[from] serde_json::Error),
    #[error("{0}")]
    Page(String),
}
type Result<T> = std::result::Result<T, ScopeError>;

/// Selection and size of the embedded signals
#[derive(Debug, Clone)]
pub struct Options {
    /// Signals to embed, as `name` or `file:name` (all the signals if `None`)
    pub signals: Option<Vec<String>>,
    /// Size budget of the embedded signals [bytes], 0 for no limit
    pub max_size: usize,
    /// Steps window `[start, end)` of the embedded data
    pub steps: Option<(usize, usize)>,
    /// Image shapes `(width, height)` by `name` or `file:name`,
    /// a signal with a square number (at least 32²) of values is a square image otherwise
    pub shapes: HashMap<String, (usize, usize)>,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            signals: None,
            max_size: MAX_SIZE,
            steps: None,
            shapes: HashMap::new(),
        }
    }
}

/// A signal read from a parquet file
#[derive(Debug, Clone)]
struct Signal {
    file: String,
    name: String,
    hash: String,
    // values per row
    n: usize,
    shape: Option<(usize, usize)>,
    // sample step of each row, -1 for the bootstrapped data
    steps: Vec<i64>,
    // rows of `n` values
    data: Vec<f32>,
    notes: Vec<String>,
}
impl Signal {
    fn size(&self) -> usize {
        self.data.len() * 4
    }
    /// Keeps one row every `k` rows
    fn thin(&mut self, k: usize) {
        if k < 2 {
            return;
        }
        let n = self.n;
        let rows: Vec<usize> = (0..self.steps.len()).step_by(k).collect();
        self.data = rows
            .iter()
            .flat_map(|&i| self.data[i * n..(i + 1) * n].iter().copied())
            .collect();
        self.steps = rows.iter().map(|&i| self.steps[i]).collect();
        self.notes.push(format!("1 row out of {k} kept"));
    }
    /// Averages the image over `q`×`q` pixels, the masked pixels (0 or NaN) left out
    fn downsample(&mut self, q: usize) {
        let Some((w, h)) = self.shape else { return };
        if q < 2 {
            return;
        }
        let (w2, h2) = (w.div_ceil(q), h.div_ceil(q));
        let mut data = Vec::with_capacity(self.steps.len() * w2 * h2);
        for row in self.data.chunks(self.n) {
            for j2 in 0..h2 {
                for i2 in 0..w2 {
                    let (mut sum, mut count) = (0f64, 0usize);
                    for j in j2 * q..((j2 + 1) * q).min(h) {
                        for i in i2 * q..((i2 + 1) * q).min(w) {
                            let v = row[j * w + i];
                            if v.is_finite() && v != 0. {
                                sum += v as f64;
                                count += 1;
                            }
                        }
                    }
                    data.push(if count > 0 { (sum / count as f64) as f32 } else { 0. });
                }
            }
        }
        self.data = data;
        self.shape = Some((w2, h2));
        self.n = w2 * h2;
        self.notes
            .push(format!("image downsampled from {w}×{h} to {w2}×{h2}"));
    }
    fn to_json(&self) -> Value {
        let bootstrap = self.steps.first() == Some(&-1);
        let mut runs = Runs::default();
        self.steps
            .iter()
            .filter(|&&s| s >= 0)
            .for_each(|&s| runs.push(s as usize));
        let bytes: Vec<u8> = self.data.iter().flat_map(|v| v.to_le_bytes()).collect();
        json!({
            "hash": self.hash,
            "name": self.name,
            "file": self.file,
            "n": self.n,
            "shape": self.shape.map(|(w, h)| [w, h]),
            "rows": self.steps.len(),
            "b": bootstrap,
            "t": runs.to_json(),
            "data": STANDARD.encode(bytes),
            "notes": self.notes,
        })
    }
}

/// Returns the JSON value of the line `const <name> = <json>;` of the page
fn constant(page: &str, name: &str) -> Result<Value> {
    let key = format!("\nconst {name} = ");
    let start = page
        .find(&key)
        .ok_or_else(|| ScopeError::Page(format!("no `{name}` in the flowchart page, is it an ELK flowchart?")))?
        + key.len();
    let end = start
        + page[start..]
            .find(";\n")
            .ok_or_else(|| ScopeError::Page(format!("`{name}` is not terminated")))?;
    Ok(serde_json::from_str(&page[start..end])?)
}

/// Expands runs `[[start, period, count], ...]`
fn expand(runs: &Value) -> Vec<i64> {
    runs.as_array()
        .into_iter()
        .flatten()
        .filter_map(|run| {
            let run: Vec<i64> = run.as_array()?.iter().filter_map(Value::as_i64).collect();
            Some((run[0], run[1], run[2]))
        })
        .flat_map(|(start, period, count)| (0..count).map(move |k| start + k * period))
        .collect()
}

fn selected(options: &Options, file: &str, name: &str) -> bool {
    options.signals.as_ref().is_none_or(|signals| {
        signals
            .iter()
            .any(|s| s == name || *s == format!("{file}:{name}"))
    })
}

/// Reads the selected signals of a parquet file
fn read(
    path: &Path,
    model: &Value,
    events: &Value,
    options: &Options,
    report: &mut Vec<String>,
) -> Result<Vec<Signal>> {
    let file = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or_default()
        .to_string();
    // the logger actor and its inputs by name
    let Some(logger) = model["actors"]
        .as_array()
        .into_iter()
        .flatten()
        .find(|a| a["label"] == file.as_str())
    else {
        report.push(format!("{file}: no logger actor `{file}` in the model, skipped"));
        return Ok(vec![]);
    };
    let inputs: HashMap<&str, &str> = logger["inputs"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .filter_map(|h| Some((model["channels"][h]["name"].as_str()?, h)))
        .collect();
    let bootstrap: Vec<&str> = events["bootstrap"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();

    let batches = ParquetRecordBatchReaderBuilder::try_new(File::open(path)?)?
        .build()?
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let Some(schema) = batches.first().map(|b| b.schema()) else {
        report.push(format!("{file}: empty file, skipped"));
        return Ok(vec![]);
    };
    let mut signals = vec![];
    for field in schema.fields() {
        let name = field.name();
        if !selected(options, &file, name) {
            continue;
        }
        let Some(&hash) = inputs.get(name.as_str()) else {
            report.push(format!("{file}:{name}: not an input of `{file}`, skipped"));
            continue;
        };
        // rows
        let mut data = vec![];
        let mut n = None;
        let mut rows = 0;
        for batch in &batches {
            let column = batch.column_by_name(name).unwrap();
            let Some(list) = column.as_list_opt::<i32>() else {
                report.push(format!("{file}:{name}: not a list column, skipped"));
                break;
            };
            let values = list.values();
            let values: Vec<f32> = if let Some(v) = values.as_primitive_opt::<Float64Type>() {
                v.values().iter().map(|&x| x as f32).collect()
            } else if let Some(v) = values.as_primitive_opt::<Float32Type>() {
                v.values().to_vec()
            } else {
                report.push(format!("{file}:{name}: values are not floats, skipped"));
                break;
            };
            let offsets = list.value_offsets();
            for i in 0..list.len() {
                let (a, b) = (offsets[i] as usize, offsets[i + 1] as usize);
                if *n.get_or_insert(b - a) != b - a {
                    report.push(format!("{file}:{name}: rows of different sizes, skipped"));
                    rows = 0;
                    break;
                }
                data.extend_from_slice(&values[a..b]);
                rows += 1;
            }
        }
        let n = n.unwrap_or(0);
        if rows == 0 || n == 0 {
            continue;
        }
        // sample steps of the data sent on the channel: the bootstrap, then the recorded sends
        let mut messages = if bootstrap.contains(&hash) { vec![-1] } else { vec![] };
        messages.extend(expand(&events["s"][hash]));
        if messages.is_empty() {
            report.push(format!("{file}:{name}: no data sent in the recorded run, skipped"));
            continue;
        }
        // the logger may keep one sample out of `decimation`
        let decimation = ((messages.len() as f64 / rows as f64).round() as usize).max(1);
        let kept = messages.len().div_ceil(decimation);
        if kept.abs_diff(rows) > 1 {
            report.push(format!(
                "{file}:{name}: {rows} rows for {} recorded data, does the file come from the recorded run?",
                messages.len()
            ));
        }
        let rows = rows.min(kept);
        let steps: Vec<i64> = (0..rows).map(|k| messages[k * decimation]).collect();
        data.truncate(rows * n);
        let shape = options
            .shapes
            .get(name.as_str())
            .or_else(|| options.shapes.get(&format!("{file}:{name}")))
            .copied()
            .or_else(|| {
                let w = (n as f64).sqrt().round() as usize;
                (w >= 32 && w * w == n).then_some((w, w))
            });
        let mut signal = Signal {
            file: file.clone(),
            name: name.clone(),
            hash: hash.to_string(),
            n,
            shape,
            steps,
            data,
            notes: vec![],
        };
        if decimation > 1 {
            signal
                .notes
                .push(format!("logged 1 sample out of {decimation}"));
        }
        // steps window
        if let Some((start, end)) = options.steps {
            let keep: Vec<usize> = (0..signal.steps.len())
                .filter(|&i| {
                    let s = signal.steps[i];
                    (s >= start as i64 || (s == -1 && start == 0)) && s < end as i64
                })
                .collect();
            signal.data = keep
                .iter()
                .flat_map(|&i| signal.data[i * n..(i + 1) * n].iter().copied())
                .collect();
            signal.steps = keep.iter().map(|&i| signal.steps[i]).collect();
            if signal.steps.is_empty() {
                report.push(format!("{file}:{name}: no data in the steps window, skipped"));
                continue;
            }
        }
        signals.push(signal);
    }
    Ok(signals)
}

/// Embeds the signals of the parquet `files` into the replay `page`, written to `output`
///
/// Returns a report of the embedded and skipped signals
pub fn embed(
    page: impl AsRef<Path>,
    files: &[PathBuf],
    output: impl AsRef<Path>,
    options: &Options,
) -> Result<Vec<String>> {
    let mut html = fs::read_to_string(page.as_ref())?;
    let model = constant(&html, "MODEL")?;
    let events = constant(&html, "EVENTS")?;
    if events.is_null() {
        return Err(ScopeError::Page(
            "the flowchart page has no recorded run, run the model with the feature `flowchart-replay`".into(),
        ));
    }
    let mut report = vec![];
    let mut signals = vec![];
    for file in files {
        signals.extend(read(file, &model, &events, options, &mut report)?);
    }
    // a channel logged in several files is embedded once
    let mut seen = std::collections::HashSet::new();
    signals.retain(|s| seen.insert(s.hash.clone()));

    // size budget: the series are thinned and the images downsampled by the same factor
    let total: usize = signals.iter().map(Signal::size).sum();
    if options.max_size > 0 && total > options.max_size {
        let factor = total as f64 / options.max_size as f64;
        for signal in signals.iter_mut() {
            if signal.shape.is_some() {
                signal.downsample(factor.sqrt().ceil() as usize);
            } else {
                signal.thin(factor.ceil() as usize);
            }
        }
        report.push(format!(
            "signals reduced from {:.1} MB to {:.1} MB to fit in {:.1} MB",
            total as f64 / 1e6,
            signals.iter().map(Signal::size).sum::<usize>() as f64 / 1e6,
            options.max_size as f64 / 1e6
        ));
    }
    for s in &signals {
        report.push(format!(
            "{}:{}: {} rows of {}{}{}",
            s.file,
            s.name,
            s.steps.len(),
            s.n,
            s.shape
                .map(|(w, h)| format!(" ({w}×{h} image)"))
                .unwrap_or_default(),
            if s.notes.is_empty() {
                String::new()
            } else {
                format!(", {}", s.notes.join(", "))
            }
        ));
    }

    let scope = json!({ "signals": signals.iter().map(Signal::to_json).collect::<Vec<_>>() });
    let scope = serde_json::to_string(&scope)?.replace("</", r"<\/");
    // replaces the `SCOPE` constant of the page
    let key = "\nconst SCOPE = ";
    let start = html.find(key).ok_or_else(|| {
        ScopeError::Page(
            "no `SCOPE` in the flowchart page, it was written before the scope support".into(),
        )
    })? + key.len();
    let end = start + html[start..].find(";\n").unwrap();
    html.replace_range(start..end, &scope);
    fs::write(output.as_ref(), html)?;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn signal(shape: Option<(usize, usize)>, n: usize, rows: usize) -> Signal {
        Signal {
            file: "f".into(),
            name: "x".into(),
            hash: "1".into(),
            n,
            shape,
            steps: (0..rows as i64).collect(),
            data: (0..rows * n).map(|v| v as f32).collect(),
            notes: vec![],
        }
    }

    #[test]
    fn thin() {
        let mut s = signal(None, 2, 5);
        s.thin(2);
        assert_eq!(s.steps, vec![0, 2, 4]);
        assert_eq!(s.data, vec![0., 1., 4., 5., 8., 9.]);
    }

    #[test]
    fn downsample() {
        let mut s = signal(Some((3, 3)), 9, 1);
        // the 0 pixel is masked
        s.downsample(2);
        assert_eq!(s.shape, Some((2, 2)));
        assert_eq!(s.data, vec![(1. + 3. + 4.) / 3., 3.5, 6.5, 8.]);
    }

    #[test]
    fn steps() {
        assert_eq!(expand(&json!([[0, 5, 3], [20, 0, 1]])), vec![0, 5, 10, 20]);
        let s = Signal {
            steps: vec![-1, 0, 1, 2],
            ..signal(None, 1, 4)
        };
        let v = s.to_json();
        assert_eq!(v["b"], true);
        assert_eq!(v["t"], json!([[0, 1, 3]]));
        assert_eq!(v["rows"], 4);
    }
}
