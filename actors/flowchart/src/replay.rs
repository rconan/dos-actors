//! Replay of a model run in the ELK flowchart
//!
//! When the ELK flowchart of a model is written, a replay session is opened for that page.
//! While the model runs, the actors record, at each sample step, that they have been updated
//! and which outputs they have sent.
//! Once the model has completed, [write] rewrites the flowchart page with the recorded steps
//! and the run can be played back step by step in the browser.
//!
//! Only the first `FLOWCHART_STEPS` steps (100 by default) are recorded.

use std::{
    collections::BTreeMap,
    env,
    fs::File,
    io::Write,
    path::PathBuf,
    sync::{
        Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
};

use serde_json::{Value, json};

use crate::render::RenderError;

/// Default number of recorded steps
const STEPS: usize = 100;

static ACTIVE: AtomicBool = AtomicBool::new(false);
static MAX_STEPS: AtomicUsize = AtomicUsize::new(STEPS);
static SESSION: Mutex<Option<Session>> = Mutex::new(None);

/// Recording of a model run
#[derive(Debug, Default)]
struct Session {
    path: PathBuf,
    title: String,
    model: String,
    // (step, actor uid)
    updates: Vec<(usize, u64)>,
    // (step, output hash), the step is `None` for a bootstrap
    sends: Vec<(Option<usize>, u64)>,
}

/// Opens a replay session for the flowchart page written at `path`
pub(crate) fn open(path: PathBuf, title: String, model: String) {
    let steps = env::var("FLOWCHART_STEPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(STEPS);
    MAX_STEPS.store(steps, Ordering::Relaxed);
    *SESSION.lock().unwrap() = Some(Session {
        path,
        title,
        model,
        ..Default::default()
    });
    ACTIVE.store(true, Ordering::Release);
}

/// Returns true if a model run is being recorded
#[inline]
pub fn is_recording() -> bool {
    ACTIVE.load(Ordering::Acquire)
}

#[inline]
fn in_range(step: usize) -> bool {
    is_recording() && step < MAX_STEPS.load(Ordering::Relaxed)
}

/// Records the update of the actor `uid` at `step`
pub fn record_update(uid: u64, step: usize) {
    if !in_range(step) {
        return;
    }
    if let Some(session) = SESSION.lock().unwrap().as_mut() {
        session.updates.push((step, uid));
    }
}

/// Records the outputs `hashes` sent at `step`, or as a bootstrap if `step` is `None`
pub fn record_send(step: Option<usize>, hashes: impl IntoIterator<Item = u64>) {
    if !step.map_or(is_recording(), in_range) {
        return;
    }
    if let Some(session) = SESSION.lock().unwrap().as_mut() {
        session.sends.extend(hashes.into_iter().map(|h| (step, h)));
    }
}

/// Returns the recorded steps as JSON
///
/// The steps are written as `{"bootstrap": [hash, ...], "steps": [{"u": [uid, ...], "s": [hash, ...]}, ...]}`,
/// with the actors uids and the output hashes as strings
fn to_json(updates: &[(usize, u64)], sends: &[(Option<usize>, u64)]) -> Value {
    let mut steps: BTreeMap<usize, (Vec<String>, Vec<String>)> = BTreeMap::new();
    for &(step, uid) in updates {
        steps.entry(step).or_default().0.push(uid.to_string());
    }
    let mut bootstrap = vec![];
    for &(step, hash) in sends {
        match step {
            Some(step) => steps.entry(step).or_default().1.push(hash.to_string()),
            None => bootstrap.push(hash.to_string()),
        }
    }
    let n = steps.keys().next_back().map_or(0, |&s| s + 1);
    let steps: Vec<_> = (0..n)
        .map(|i| {
            let (mut u, mut s) = steps.remove(&i).unwrap_or_default();
            u.sort();
            u.dedup();
            s.sort();
            s.dedup();
            json!({"u": u, "s": s})
        })
        .collect();
    bootstrap.sort();
    bootstrap.dedup();
    json!({"bootstrap": bootstrap, "steps": steps})
}

/// Rewrites the flowchart page with the recorded run and closes the replay session
///
/// Returns the path to the page or `None` if there is no replay session
pub fn write() -> Result<Option<PathBuf>, RenderError> {
    ACTIVE.store(false, Ordering::Release);
    let Some(session) = SESSION.lock().unwrap().take() else {
        return Ok(None);
    };
    let events = serde_json::to_string(&to_json(&session.updates, &session.sends))?;
    let page = crate::elk::fill(&session.title, &session.model, &events);
    let mut file = File::create(&session.path)?;
    file.write_all(page.as_bytes())?;
    Ok(Some(session.path))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps() {
        let updates = [(0, 7), (0, 8), (1, 7), (3, 8), (1, 7)];
        let sends = [(None, 5), (Some(0), 1), (Some(0), 1), (Some(3), 2)];
        let events = to_json(&updates, &sends);
        assert_eq!(events["bootstrap"], json!(["5"]));
        let steps = events["steps"].as_array().unwrap();
        assert_eq!(steps.len(), 4);
        assert_eq!(steps[0], json!({"u": ["7", "8"], "s": ["1"]}));
        assert_eq!(steps[1], json!({"u": ["7"], "s": []}));
        assert_eq!(steps[2], json!({"u": [], "s": []}));
        assert_eq!(steps[3], json!({"u": ["8"], "s": ["2"]}));
    }
}
