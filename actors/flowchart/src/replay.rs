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
//!
//! The runs of models executed one after the other can be chained into a single replay with [chain]:
//! the page of the last model is written with the merged graph of the models and the run of all of them,
//! the steps of a model following the steps of the previous one.
//! The actors that are in several models are identified by their client, so a client must be shared
//! by the models (e.g. by passing the same `Client` to the models), and an output by its actor and type.
//! A bootstrapped output that resends at the start of a model the data it has already sent at the end of
//! the previous model is not counted twice, so the k-th data received on a channel over the chained models
//! is still the k-th data sent on that channel.

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
// the models runs are chained into a single replay session
static CHAIN: AtomicBool = AtomicBool::new(false);
// the 1st step of the model being recorded within the chained runs
static OFFSET: AtomicUsize = AtomicUsize::new(0);
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
    /// Appends the steps of `runs`, that follow the last step, joining the runs that continue each other
    fn extend(&mut self, runs: Runs) {
        for mut run in runs.0 {
            // the steps of `run` up to the last step are ignored
            if let Some(last) = self.0.last().map(Run::last) {
                if run.start <= last {
                    let skip = if run.period == 0 {
                        run.count
                    } else {
                        (last - run.start) / run.period + 1
                    };
                    if skip >= run.count {
                        continue;
                    }
                    run.start += skip * run.period;
                    run.count -= skip;
                }
            }
            match self.0.last_mut() {
                // a single step continuing a run, or the start of a run continuing a single step
                _ if run.count == 1 => self.push(run.start),
                Some(last) if last.count == 1 && run.start - last.start == run.period => {
                    last.period = run.period;
                    last.count += run.count;
                }
                Some(last)
                    if last.period == run.period && run.start == last.last() + run.period =>
                {
                    last.count += run.count
                }
                _ => self.0.push(run),
            }
        }
    }
    /// Returns the steps shifted by `offset`
    fn shifted(self, offset: usize) -> Self {
        Self(
            self.0
                .into_iter()
                .map(|run| Run {
                    start: run.start + offset,
                    ..run
                })
                .collect(),
        )
    }
    pub(crate) fn to_json(&self) -> Value {
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
    // 1st step of the model being recorded within chained runs
    offset: usize,
}
impl Session {
    /// Records the bootstrapped outputs `hashes`
    ///
    /// In a model that follows another one, the bootstrapped data is sent at the end of the previous model run:
    /// it is recorded as a send at the last step of the previous run, unless it has been sent at that step already
    fn record_bootstrap(&mut self, hashes: impl IntoIterator<Item = u64>) {
        if self.offset == 0 {
            self.bootstrap.extend(hashes);
        } else {
            for hash in hashes {
                self.sends.entry(hash).or_default().push(self.offset - 1);
            }
        }
    }
    /// Records the steps at which the actor `uid` has been updated and has sent the outputs `hashes`
    fn record(&mut self, uid: u64, updates: Runs, hashes: &[u64], sends: Runs) {
        let (updates, sends) = (updates.shifted(self.offset), sends.shifted(self.offset));
        self.updates.entry(uid).or_default().extend(updates);
        for &hash in hashes {
            self.sends.entry(hash).or_default().extend(sends.clone());
        }
    }
    /// Returns the step following the last recorded step
    fn end(&self) -> usize {
        self.updates
            .values()
            .chain(self.sends.values())
            .map(Runs::end)
            .max()
            .unwrap_or(0)
    }
    /// Returns the 1st step of the run of the next chained model
    ///
    /// It is the step following the last data sent: the initiator of a model is updated one more time
    /// to find out that the model has completed but its last update has no data to send
    fn next_offset(&self) -> usize {
        self.sends
            .values()
            .map(Runs::end)
            .max()
            .unwrap_or_else(|| self.end())
    }
    /// Chains the run of the model of the page written at `path`
    fn chain(&mut self, path: PathBuf, title: String, model: String) -> Result<(), RenderError> {
        self.model = merge(&self.model, &model)?;
        self.path = path;
        self.title = title;
        Ok(())
    }
}

/// Merges the model `model` into the model `previous`
///
/// The actors of `previous` that are not in `model` are added to `model`:
/// an actor is identified by its client (`uid`) and a system by its label;
/// the actors of a system of `previous` that is in `model` are moved into the system of `model`.
fn merge(previous: &str, model: &str) -> Result<String, RenderError> {
    let previous: Value = serde_json::from_str(&previous.replace(r"<\/", "</"))?;
    let mut merged: Value = serde_json::from_str(&model.replace(r"<\/", "</"))?;
    let new_actors = merged["actors"].as_array().cloned().unwrap_or_default();
    let has_uid = |uid: &Value| !uid.is_null() && new_actors.iter().any(|a| &a["uid"] == uid);
    // ids of the systems of `previous` that are in `model`
    let systems: HashMap<String, Value> = previous["actors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|a| a["system"] == json!(true))
        .filter_map(|a| {
            new_actors
                .iter()
                .find(|b| b["system"] == json!(true) && b["label"] == a["label"])
                .map(|b| {
                    (
                        a["id"].as_str().unwrap_or_default().to_string(),
                        b["id"].clone(),
                    )
                })
        })
        .collect();
    let mut added = vec![];
    for actor in previous["actors"].as_array().into_iter().flatten() {
        let id = actor["id"].as_str().unwrap_or_default();
        if has_uid(&actor["uid"]) || systems.contains_key(id) {
            continue;
        }
        let mut actor = actor.clone();
        if let Some(parent) = actor["parent"].as_str().and_then(|p| systems.get(p)) {
            actor["parent"] = parent.clone();
        }
        added.push(actor);
    }
    if let Some(actors) = merged["actors"].as_array_mut() {
        actors.extend(added);
    }
    if let (Some(channels), Some(previous)) = (
        merged["channels"].as_object_mut(),
        previous["channels"].as_object(),
    ) {
        for (hash, channel) in previous {
            channels
                .entry(hash.clone())
                .or_insert_with(|| channel.clone());
        }
    }
    // escapes `</` so that the JSON can never close the script element
    Ok(serde_json::to_string(&merged)?.replace("</", r"<\/"))
}

/// Chains the runs of the next models into a single replay
///
/// The page of each model is written at the completion of its run with the merged graph of the chained models
/// and the run of all of them, so the page of the last model is the replay of the whole chain
pub fn chain() {
    CHAIN.store(true, Ordering::Release);
}

/// Opens a replay session for the flowchart page written at `path`
pub(crate) fn open(path: PathBuf, title: String, model: String) {
    let steps = env::var("FLOWCHART_STEPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(usize::MAX);
    MAX_STEPS.store(steps, Ordering::Relaxed);
    let mut session = SESSION.lock().unwrap();
    match session.as_mut() {
        Some(chained) if CHAIN.load(Ordering::Acquire) => {
            if let Err(e) = chained.chain(path.clone(), title.clone(), model.clone()) {
                log::warn!("failed to chain the replay of {path:?}, a new replay is started: {e}");
                *session = Some(Session {
                    path,
                    title,
                    model,
                    ..Default::default()
                });
            }
        }
        _ => {
            *session = Some(Session {
                path,
                title,
                model,
                ..Default::default()
            });
        }
    }
    OFFSET.store(session.as_ref().map_or(0, |s| s.offset), Ordering::Release);
    ACTIVE.store(true, Ordering::Release);
}

/// Returns true if a model run is being recorded
#[inline]
pub fn is_recording() -> bool {
    ACTIVE.load(Ordering::Acquire)
}

/// Returns the number of steps to record (`FLOWCHART_STEPS`, unlimited by default) in the model being recorded
///
/// In chained runs, `FLOWCHART_STEPS` counts the steps from the start of the 1st model
pub fn max_steps() -> usize {
    MAX_STEPS
        .load(Ordering::Relaxed)
        .saturating_sub(OFFSET.load(Ordering::Acquire))
}

/// Records the bootstrapped outputs `hashes`
pub fn record_bootstrap(hashes: impl IntoIterator<Item = u64>) {
    if !is_recording() {
        return;
    }
    if let Some(session) = SESSION.lock().unwrap().as_mut() {
        session.record_bootstrap(hashes);
    }
}

/// Records the steps at which the actor `uid` has been updated and has sent the outputs `hashes`
pub fn record(uid: u64, updates: Runs, hashes: &[u64], sends: Runs) {
    if !is_recording() {
        return;
    }
    if let Some(session) = SESSION.lock().unwrap().as_mut() {
        session.record(uid, updates, hashes, sends);
    }
}

/// Returns the recorded run as JSON
///
/// The run is written as
/// `{"steps": n, "bootstrap": [hash, ...], "u": {uid: runs, ...}, "s": {hash: runs, ...}}`,
/// with the actors uids and the output hashes as strings and the runs as `[[start, period, count], ...]`
fn to_json(session: &Session) -> Value {
    let steps = session.end();
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
/// With chained runs, the session is kept for the run of the next model, starting after the last recorded step.
/// Returns the path to the page or `None` if there is no replay session
fn write() -> Result<Option<PathBuf>, RenderError> {
    ACTIVE.store(false, Ordering::Release);
    let mut guard = SESSION.lock().unwrap();
    let Some(session) = guard.as_mut() else {
        return Ok(None);
    };
    let events = serde_json::to_string(&to_json(session))?;
    let page = crate::elk::fill(&session.title, &session.model, &events);
    let mut file = File::create(&session.path)?;
    file.write_all(page.as_bytes())?;
    let path = session.path.clone();
    if CHAIN.load(Ordering::Acquire) {
        session.offset = session.next_offset();
    } else {
        guard.take();
    }
    Ok(Some(path))
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
    fn extended_runs() {
        let mut r = runs(0..5);
        r.extend(runs(5..8));
        assert_eq!(r.to_json(), json!([[0, 1, 8]]));
        r.extend(runs((10..20).step_by(2)));
        assert_eq!(r.to_json(), json!([[0, 1, 8], [10, 2, 5]]));
        // a step before the last one is ignored
        r.extend(runs([3]));
        assert_eq!(r.to_json(), json!([[0, 1, 8], [10, 2, 5]]));
        // the steps of a run up to the last step are ignored
        r.extend(runs((16..30).step_by(2)));
        assert_eq!(r.to_json(), json!([[0, 1, 8], [10, 2, 10]]));
        let mut r = runs([4]);
        r.extend(runs(5..8));
        assert_eq!(r.to_json(), json!([[4, 1, 4]]));
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

    #[test]
    fn chained_session() {
        // 1st model: 5 steps, the output 1 is bootstrapped and sent at every step
        let mut session = Session::default();
        session.record_bootstrap([1]);
        session.record(7, runs(0..5), &[1], runs(0..5));
        // the output 2 is only in the 1st model, the output 3 only in the 2nd one
        session.record(8, runs(0..5), &[2], runs(0..5));
        // the initiator is updated once more at the end of the model, without sending anything
        session.record(10, runs(0..6), &[], Runs::default());
        session.offset = session.next_offset();
        assert_eq!(session.offset, 5);
        // 2nd model: 3 steps, the outputs 1 and 3 are bootstrapped
        session.record_bootstrap([1, 3]);
        session.record(7, runs(0..3), &[1], runs(0..3));
        session.record(9, runs(0..3), &[3], runs(0..3));
        session.record(10, runs(0..4), &[], Runs::default());
        let events = to_json(&session);
        assert_eq!(events["steps"], 9);
        assert_eq!(events["u"]["10"], json!([[0, 1, 9]]));
        // only the bootstrap of the 1st model
        assert_eq!(events["bootstrap"], json!(["1"]));
        // the resent bootstrap of output 1 is the send at step 4 of the 1st model
        assert_eq!(events["u"]["7"], json!([[0, 1, 8]]));
        assert_eq!(events["s"]["1"], json!([[0, 1, 8]]));
        assert_eq!(events["s"]["2"], json!([[0, 1, 5]]));
        // the bootstrap of output 3 is sent at the last step of the 1st model
        assert_eq!(events["s"]["3"], json!([[4, 1, 4]]));
        assert_eq!(events["u"]["9"], json!([[5, 1, 3]]));
    }

    #[test]
    fn merged_models() {
        let actor =
            |id: &str, label: &str, parent: Option<&str>, system: bool, uid: Option<&str>| {
                json!({"id": id, "label": label, "parent": parent, "system": system, "uid": uid,
                   "inputs": [], "outputs": [], "unbounded": [], "image": null})
            };
        let previous = json!({
            "name": "warmup",
            "actors": [
                actor("warmup/1", "Trajectory", None, false, Some("10")),
                actor("warmup/2", "Logger</", None, false, Some("11")),
                actor("warmup/3", "Servos", None, true, None),
                actor("warmup/3/4", "Fem", Some("warmup/3"), false, Some("12")),
                actor("warmup/3/5", "Only1", Some("warmup/3"), false, Some("13")),
            ],
            "channels": {"100": {"name": "A"}, "101": {"name": "B"}},
        })
        .to_string()
        .replace("</", r"<\/");
        let model = json!({
            "name": "model",
            "actors": [
                actor("model/1", "Trajectory", None, false, Some("10")),
                actor("model/3", "Servos", None, true, None),
                actor("model/3/4", "Fem", Some("model/3"), false, Some("12")),
                actor("model/6", "Optics", None, false, Some("14")),
            ],
            "channels": {"100": {"name": "A"}, "102": {"name": "C"}},
        })
        .to_string();
        let merged = merge(&previous, &model).unwrap();
        assert!(!merged.contains("</"));
        let merged: Value = serde_json::from_str(&merged.replace(r"<\/", "</")).unwrap();
        assert_eq!(merged["name"], "model");
        let ids: Vec<_> = merged["actors"]
            .as_array()
            .unwrap()
            .iter()
            .map(|a| (a["id"].as_str().unwrap(), a["parent"].as_str()))
            .collect();
        assert_eq!(
            ids,
            vec![
                ("model/1", None),
                ("model/3", None),
                ("model/3/4", Some("model/3")),
                ("model/6", None),
                ("warmup/2", None),
                ("warmup/3/5", Some("model/3")),
            ]
        );
        assert_eq!(
            merged["channels"]
                .as_object()
                .unwrap()
                .keys()
                .collect::<Vec<_>>(),
            vec!["100", "101", "102"]
        );
    }
}
