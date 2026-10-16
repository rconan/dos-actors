//! Replay of a model run in the ELK flowchart
//!
//! When the ELK flowchart of a model is written, a replay session is opened for that page.
//! While the model runs, each actor records, in its own [Runs], the sample steps at which it is updated
//! and at which it sends its outputs, and hands them over with [record] when its loop ends.
//! Once the outermost model has completed ([model_completed]), the flowchart page is rewritten with the recorded run
//! and the run can be played back step by step in the browser.
//!
//! The steps of an actor follow from its rate, so they are stored as arithmetic progressions ([Run]s),
//! a few per actor whatever the length of the run.
//! The whole run is recorded unless `FLOWCHART_STEPS` limits the number of recorded steps.

use std::{
    collections::HashMap,
    env,
    fs::File,
    io::Write,
    path::PathBuf,
    sync::{
        Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
};

use serde_json::{Map, Value, json};

use crate::render::RenderError;

static ACTIVE: AtomicBool = AtomicBool::new(false);
// number of running models, a system runs its actors as a model nested in the model it belongs to
static RUNNING: AtomicUsize = AtomicUsize::new(0);
static MAX_STEPS: AtomicUsize = AtomicUsize::new(usize::MAX);
static SESSION: Mutex<Option<Session>> = Mutex::new(None);

/// The sample steps `start`, `start + period`, ..., `start + (count - 1) * period`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Run {
    start: usize,
    period: usize,
    count: usize,
}
impl Run {
    fn last(&self) -> usize {
        self.start + self.period * (self.count - 1)
    }
}

/// Increasing sample steps stored as [Run]s
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Runs(Vec<Run>);
impl Runs {
    /// Adds a step, a step that is not after the last one is ignored
    #[inline]
    pub fn push(&mut self, step: usize) {
        match self.0.last_mut() {
            Some(run) if step <= run.last() => (),
            Some(run) if run.count == 1 => {
                run.period = step - run.start;
                run.count = 2;
            }
            Some(run) if step == run.last() + run.period => run.count += 1,
            _ => self.0.push(Run {
                start: step,
                period: 0,
                count: 1,
            }),
        }
    }
    /// Returns the step following the last one
    fn end(&self) -> usize {
        self.0.last().map_or(0, |run| run.last() + 1)
    }
    fn to_json(&self) -> Value {
        self.0
            .iter()
            .map(|run| json!([run.start, run.period, run.count]))
            .collect()
    }
}

/// Recording of a model run
#[derive(Debug, Default)]
struct Session {
    path: PathBuf,
    title: String,
    model: String,
    // actor uid -> update steps
    updates: HashMap<u64, Runs>,
    // output hash -> send steps
    sends: HashMap<u64, Runs>,
    // bootstrapped output hashes
    bootstrap: Vec<u64>,
}

/// Opens a replay session for the flowchart page written at `path`
pub(crate) fn open(path: PathBuf, title: String, model: String) {
    let steps = env::var("FLOWCHART_STEPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(usize::MAX);
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

/// Returns the number of steps to record (`FLOWCHART_STEPS`, unlimited by default)
pub fn max_steps() -> usize {
    MAX_STEPS.load(Ordering::Relaxed)
}

/// Records the bootstrapped outputs `hashes`
pub fn record_bootstrap(hashes: impl IntoIterator<Item = u64>) {
    if !is_recording() {
        return;
    }
    if let Some(session) = SESSION.lock().unwrap().as_mut() {
        session.bootstrap.extend(hashes);
    }
}

/// Records the steps at which the actor `uid` has been updated and has sent the outputs `hashes`
pub fn record(uid: u64, updates: Runs, hashes: &[u64], sends: Runs) {
    if !is_recording() {
        return;
    }
    if let Some(session) = SESSION.lock().unwrap().as_mut() {
        session.updates.entry(uid).or_default().0.extend(updates.0);
        for &hash in hashes {
            session.sends.entry(hash).or_default().0.extend(sends.0.iter().copied());
        }
    }
}

/// Returns the recorded run as JSON
///
/// The run is written as
/// `{"steps": n, "bootstrap": [hash, ...], "u": {uid: runs, ...}, "s": {hash: runs, ...}}`,
/// with the actors uids and the output hashes as strings and the runs as `[[start, period, count], ...]`
fn to_json(session: &Session) -> Value {
    let steps = session
        .updates
        .values()
        .chain(session.sends.values())
        .map(Runs::end)
        .max()
        .unwrap_or(0);
    let runs = |map: &HashMap<u64, Runs>| -> Map<String, Value> {
        map.iter()
            .filter(|(_, runs)| !runs.0.is_empty())
            .map(|(key, runs)| (key.to_string(), runs.to_json()))
            .collect()
    };
    let mut bootstrap: Vec<_> = session.bootstrap.iter().map(u64::to_string).collect();
    bootstrap.sort();
    bootstrap.dedup();
    json!({
        "steps": steps,
        "bootstrap": bootstrap,
        "u": runs(&session.updates),
        "s": runs(&session.sends),
    })
}

/// Signals that a model is running
pub fn model_running() {
    RUNNING.fetch_add(1, Ordering::AcqRel);
}

/// Signals that a model has completed
///
/// Once the outermost model has completed, the flowchart page is rewritten with the recorded run
/// and the path to the page is returned; `None` is returned for a nested model or if there is no replay session
pub fn model_completed() -> Result<Option<PathBuf>, RenderError> {
    // decrements the number of running models without going below 0
    let mut running = RUNNING.load(Ordering::Acquire);
    while running > 0 {
        match RUNNING.compare_exchange_weak(
            running,
            running - 1,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => break,
            Err(n) => running = n,
        }
    }
    if running > 1 {
        return Ok(None);
    }
    write()
}

/// Rewrites the flowchart page with the recorded run and closes the replay session
///
/// Returns the path to the page or `None` if there is no replay session
fn write() -> Result<Option<PathBuf>, RenderError> {
    ACTIVE.store(false, Ordering::Release);
    let Some(session) = SESSION.lock().unwrap().take() else {
        return Ok(None);
    };
    let events = serde_json::to_string(&to_json(&session))?;
    let page = crate::elk::fill(&session.title, &session.model, &events);
    let mut file = File::create(&session.path)?;
    file.write_all(page.as_bytes())?;
    Ok(Some(session.path))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn runs(steps: impl IntoIterator<Item = usize>) -> Runs {
        let mut runs = Runs::default();
        steps.into_iter().for_each(|step| runs.push(step));
        runs
    }

    #[test]
    fn progressions() {
        assert_eq!(runs(0..5001).to_json(), json!([[0, 1, 5001]]));
        assert_eq!(runs((4..5000).step_by(5)).to_json(), json!([[4, 5, 1000]]));
        assert_eq!(runs([7]).to_json(), json!([[7, 0, 1]]));
        // a duplicated or out of order step is ignored
        assert_eq!(runs([0, 0, 10, 5, 20]).to_json(), json!([[0, 10, 3]]));
        // a change of period starts a new run
        assert_eq!(
            runs([0, 1, 2, 10, 20, 30, 31]).to_json(),
            json!([[0, 1, 3], [10, 10, 3], [31, 0, 1]])
        );
        assert_eq!(runs([0, 1, 2, 10, 20, 30, 31]).end(), 32);
    }

    #[test]
    fn session() {
        let mut session = Session::default();
        session.updates.insert(7, runs(0..10));
        session.updates.insert(8, runs((0..10).step_by(5)));
        session.sends.insert(1, runs(0..10));
        session.sends.insert(2, Runs::default());
        session.bootstrap = vec![5, 5];
        let events = to_json(&session);
        assert_eq!(events["steps"], 10);
        assert_eq!(events["bootstrap"], json!(["5"]));
        assert_eq!(events["u"]["7"], json!([[0, 1, 10]]));
        assert_eq!(events["u"]["8"], json!([[0, 5, 2]]));
        assert_eq!(events["s"]["1"], json!([[0, 1, 10]]));
        assert!(events["s"].get("2").is_none());
    }
}
