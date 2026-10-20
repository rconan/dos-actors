//! Embeds the signals saved in parquet files into the replay page of a model run
//!
//! ```text
//! flowchart-scope <page.html> <file.parquet>... [--signals NAME,FILE:NAME,...] [--max-size MB]
//!                 [--steps START..END] [--shape NAME=WxH]... [--pixel NAME=SIZE[UNIT]]...
//!                 [--sampling-frequency HZ] [--output OUT.html]
//! ```

use std::{env, path::PathBuf, process::ExitCode};

use gmt_dos_actors_flowchart::scope::{Options, embed};

const USAGE: &str = "usage: flowchart-scope <page.html> <file.parquet>... [--signals NAME,FILE:NAME,...] [--max-size MB (0: no limit, default: 64)] [--steps START..END] [--shape NAME=WxH]... [--pixel NAME=SIZE[UNIT] (e.g. 'Frame<Host>=3.93mas')]... [--sampling-frequency HZ] [--output OUT.html]";

fn parse() -> Result<(PathBuf, Vec<PathBuf>, PathBuf, Options), String> {
    let mut args = env::args().skip(1);
    let mut options = Options::default();
    let mut paths = vec![];
    let mut output = None;
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("missing value for {arg}"));
        match arg.as_str() {
            "-h" | "--help" => return Err(USAGE.into()),
            "--signals" => {
                options.signals = Some(value()?.split(',').map(|s| s.trim().to_string()).collect())
            }
            "--max-size" => {
                let mb: f64 = value()?.parse().map_err(|e| format!("--max-size: {e}"))?;
                options.max_size = (mb * 1024. * 1024.) as usize;
            }
            "--steps" => {
                let v = value()?;
                let (a, b) = v.split_once("..").ok_or("--steps: expected START..END")?;
                let a = a.parse().map_err(|e| format!("--steps: {e}"))?;
                let b = if b.is_empty() { usize::MAX } else { b.parse().map_err(|e| format!("--steps: {e}"))? };
                options.steps = Some((a, b));
            }
            "--shape" => {
                let v = value()?;
                let (name, shape) = v.rsplit_once('=').ok_or("--shape: expected NAME=WxH")?;
                let (w, h) = shape.split_once('x').ok_or("--shape: expected NAME=WxH")?;
                let w = w.parse().map_err(|e| format!("--shape: {e}"))?;
                let h = h.parse().map_err(|e| format!("--shape: {e}"))?;
                options.shapes.insert(name.to_string(), (w, h));
            }
            "--pixel" => {
                let v = value()?;
                let (name, size) = v.rsplit_once('=').ok_or("--pixel: expected NAME=SIZE[UNIT]")?;
                // the size is a number followed by an optional unit
                let split = size
                    .find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E')))
                    .unwrap_or(size.len());
                let (value, unit) = size.split_at(split);
                let value: f64 = value.parse().map_err(|e| format!("--pixel: {e}"))?;
                options
                    .pixels
                    .insert(name.to_string(), (value, unit.trim().to_string()));
            }
            "--sampling-frequency" | "--fs" => {
                let fs: f64 = value()?.parse().map_err(|e| format!("--sampling-frequency: {e}"))?;
                if !(fs > 0.) {
                    return Err("--sampling-frequency: expected a positive frequency [Hz]".into());
                }
                options.sampling_frequency = Some(fs);
            }
            "--output" | "-o" => output = Some(PathBuf::from(value()?)),
            _ if arg.starts_with('-') => return Err(format!("unknown option {arg}\n{USAGE}")),
            _ => paths.push(PathBuf::from(arg)),
        }
    }
    if paths.len() < 2 {
        return Err(USAGE.into());
    }
    let page = paths.remove(0);
    let output = output.unwrap_or_else(|| page.clone());
    Ok((page, paths, output, options))
}

fn main() -> ExitCode {
    let (page, files, output, options) = match parse() {
        Ok(args) => args,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    match embed(&page, &files, &output, &options) {
        Ok(report) => {
            report.iter().for_each(|line| println!("{line}"));
            println!("signals embedded into {output:?}");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("flowchart-scope: {e}");
            ExitCode::FAILURE
        }
    }
}
