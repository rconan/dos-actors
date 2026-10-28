//! Live streaming of the signals of a running model into the flowchart page (feature `live`)
//!
//! With the environment variable `FLOWCHART_LIVE` set to a port (or to `address:port`, the address is
//! `127.0.0.1` otherwise), the ELK flowchart page of the running model is served at `http://localhost:<port>/`
//! and the data received by the Arrow loggers (`gmt_dos-clients_arrow` with the feature `flowchart-live`)
//! is streamed into the scopes of the page while the model runs.
//!
//! A logger hands over each row it logs with [push], within the [Context] of the actor that reads the data
//! (the actor, its sample step and the channel the data comes from, see [with]).
//! The rows are queued without ever waiting: if the streaming falls behind, the rows are dropped from the
//! stream, never from the logs, so the model is never slowed down by the streaming.
//!
//! A thread keeps a thinned history of each signal (one row every `k` rows, `k` doubling as the run goes),
//! so a scope opened late shows the run so far, and sends to each page, as server-sent events,
//! the new rows of the scopes it shows, at most 20 times per second.
//! The page is still written at the end of the run for the offline replay.
//!
//! The sampling frequency of the steps, to show the time in the page, is given with `FLOWCHART_LIVE_FS` [Hz]
//! and the pixel sizes of the images with `FLOWCHART_LIVE_PIXEL`, e.g. `Frame<Host>=3.93mas;Wavefront=1.6cm`.

use std::{
    cell::Cell,
    collections::{BTreeMap, HashMap},
    env,
    io::{BufRead, BufReader, Write},
    net::{TcpListener, TcpStream},
    sync::{
        Mutex, OnceLock,
        atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
        mpsc::{Receiver, RecvTimeoutError, SyncSender, TrySendError, sync_channel},
    },
    thread,
    time::{Duration, Instant},
};

use base64::{Engine, engine::general_purpose::STANDARD};
use serde_json::{Value, json};

// streaming on
static ACTIVE: AtomicBool = AtomicBool::new(false);
// queue of the hub thread
static QUEUE: OnceLock<SyncSender<Msg>> = OnceLock::new();
// server address, once started
static ADDRESS: Mutex<Option<String>> = Mutex::new(None);
static CLIENT_ID: AtomicU64 = AtomicU64::new(0);
// number of pages the recorded run has been queued to and not yet written to
static DELIVERING: AtomicUsize = AtomicUsize::new(0);

/// Capacity of the queue of the rows logged by the model
const QUEUE_CAPACITY: usize = 8192;
/// Interval between the updates sent to the pages
const FLUSH: Duration = Duration::from_millis(50);
/// Maximum number of rows of the history of a signal
const HISTORY_ROWS: usize = 10_000;
/// Maximum number of values of the history of a signal
const HISTORY_VALUES: usize = 4_000_000;
/// Maximum number of rows waiting to be sent to a page per signal, above which they are thinned
const PENDING_ROWS: usize = 2_000;
/// Number of updates waiting to be sent to a page, above which the updates are dropped
const CLIENT_CAPACITY: usize = 256;

/// The actor reading the data handed over with [push]
#[derive(Debug, Clone, Copy)]
pub struct Context {
    /// Identifier of the actor
    pub uid: u64,
    /// Sample step of the actor within the model run
    pub step: usize,
    /// Hash of the channel the data comes from
    pub hash: u64,
}

thread_local! {
    static CONTEXT: Cell<Option<Context>> = const { Cell::new(None) };
}

/// Resets the context, even if the client panics
struct Reset;
impl Drop for Reset {
    fn drop(&mut self) {
        CONTEXT.set(None);
    }
}

/// Returns true if the signals are streamed
#[inline]
pub fn is_active() -> bool {
    ACTIVE.load(Ordering::Relaxed)
}

/// Runs `f`, an actor client reading data, within the actor [Context]
#[inline]
pub fn with<R>(context: Context, f: impl FnOnce() -> R) -> R {
    if !is_active() {
        return f();
    }
    CONTEXT.set(Some(context));
    let _reset = Reset;
    f()
}

/// Hands over a row of data logged within an actor [Context] to the streaming
///
/// The row is dropped if the streaming is off, if there is no context or if the queue is full
#[inline]
pub fn push<I: IntoIterator<Item = f32>>(values: I) {
    if !is_active() {
        return;
    }
    let (Some(context), Some(queue)) = (CONTEXT.get(), QUEUE.get()) else {
        return;
    };
    let row = Row {
        uid: context.uid,
        hash: context.hash,
        step: (context.step + crate::replay::offset()) as i64,
        values: values.into_iter().collect(),
    };
    let _ = queue.try_send(Msg::Row(row));
}

/// Serves the flowchart page of a model with the title `title` and the model `model` (JSON)
///
/// The server is started the first time, if `FLOWCHART_LIVE` is set; the pages already opened are reloaded
pub(crate) fn open(title: &str, model: &str) {
    if QUEUE.get().is_none() && !start() {
        return;
    }
    send(Msg::Model {
        title: title.to_string(),
        model: model.to_string(),
    });
}

/// Sends the recorded run `events` (JSON) of the model to the pages, with the history of all the signals
///
/// Waits until the pages have received them, at most `FLOWCHART_LIVE_LINGER` seconds (60 by default),
/// so the pages can show any signal of the run once the server is gone
pub(crate) fn recorded(events: &str) {
    if QUEUE.get().is_none() {
        return;
    }
    let (tx, rx) = sync_channel(1);
    send(Msg::Events(events.to_string(), tx));
    let _ = rx.recv();
    let linger = env::var("FLOWCHART_LIVE_LINGER")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(60.);
    let deadline = Instant::now() + Duration::from_secs_f64(linger);
    let start = Instant::now();
    let mut told = false;
    while DELIVERING.load(Ordering::Acquire) > 0 && Instant::now() < deadline {
        if !told && start.elapsed() > Duration::from_secs(2) {
            println!("flowchart live: sending the run to the pages (at most {linger}s, FLOWCHART_LIVE_LINGER)");
            told = true;
        }
        thread::sleep(Duration::from_millis(50));
    }
}

/// Returns the address of the server, if started
pub fn address() -> Option<String> {
    ADDRESS.lock().unwrap().clone()
}

fn send(msg: Msg) {
    if let Some(queue) = QUEUE.get() {
        let _ = queue.send(msg);
    }
}

/// Starts the hub and the HTTP server, returns false if `FLOWCHART_LIVE` is not set or on failure
fn start() -> bool {
    let Ok(var) = env::var("FLOWCHART_LIVE") else {
        return false;
    };
    let address = if var.contains(':') {
        var
    } else {
        format!("127.0.0.1:{var}")
    };
    let listener = match TcpListener::bind(&address) {
        Ok(listener) => listener,
        Err(e) => {
            log::warn!("flowchart live streaming disabled, failed to listen on {address}: {e}");
            return false;
        }
    };
    let (tx, rx) = sync_channel(QUEUE_CAPACITY);
    if QUEUE.set(tx).is_err() {
        return true;
    }
    let fs = env::var("FLOWCHART_LIVE_FS")
        .ok()
        .and_then(|fs| fs.parse::<f64>().ok())
        .filter(|fs| *fs > 0.);
    let pixels = env::var("FLOWCHART_LIVE_PIXEL")
        .map(|v| parse_pixels(&v))
        .unwrap_or_default();
    thread::Builder::new()
        .name("flowchart-live-hub".into())
        .spawn(move || Hub::new(fs, pixels).run(rx))
        .expect("failed to spawn the flowchart live hub");
    thread::Builder::new()
        .name("flowchart-live-server".into())
        .spawn(move || serve(listener))
        .expect("failed to spawn the flowchart live server");
    let port = address.rsplit_once(':').map_or("", |(_, p)| p);
    println!("flowchart live at http://localhost:{port}/");
    *ADDRESS.lock().unwrap() = Some(address);
    ACTIVE.store(true, Ordering::Release);
    true
}

/// Parses `NAME=SIZE[UNIT];...`, e.g. `Frame<Host>=3.93mas`
fn parse_pixels(value: &str) -> HashMap<String, (f64, String)> {
    value
        .split(';')
        .filter_map(|item| {
            let (name, size) = item.trim().rsplit_once('=')?;
            let i = size
                .find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | 'e' | 'E' | '-' | '+')))
                .unwrap_or(size.len());
            Some((
                name.to_string(),
                (size[..i].parse().ok()?, size[i..].trim().to_string()),
            ))
        })
        .collect()
}

#[derive(Debug)]
struct Row {
    uid: u64,
    hash: u64,
    step: i64,
    values: Vec<f32>,
}

enum Msg {
    Row(Row),
    Model { title: String, model: String },
    // the recorded run, acknowledged once queued to the pages
    Events(String, SyncSender<()>),
    Connect(u64, SyncSender<String>),
    Disconnect(u64),
    Subscribe(u64, u64),
    Unsubscribe(u64, u64),
    Page(SyncSender<String>),
}

/// Rows of a signal
#[derive(Debug, Default)]
struct Rows {
    steps: Vec<i64>,
    data: Vec<f32>,
}
impl Rows {
    fn push(&mut self, step: i64, values: &[f32]) {
        self.steps.push(step);
        self.data.extend_from_slice(values);
    }
    /// Keeps every other row
    fn halve(&mut self, n: usize) {
        let rows: Vec<usize> = (0..self.steps.len()).step_by(2).collect();
        self.data = rows
            .iter()
            .flat_map(|&i| self.data[i * n..(i + 1) * n].iter().copied())
            .collect();
        self.steps = rows.iter().map(|&i| self.steps[i]).collect();
    }
    fn to_json(&self, hash: u64) -> Value {
        let bytes: Vec<u8> = self.data.iter().flat_map(|v| v.to_le_bytes()).collect();
        json!({"hash": hash.to_string(), "steps": self.steps, "data": STANDARD.encode(bytes)})
    }
}

/// A signal: the data received by a logger on a channel
#[derive(Debug)]
struct Signal {
    file: String,
    name: String,
    n: usize,
    shape: Option<(usize, usize)>,
    pixel: Option<(f64, String)>,
    // thinned history: one row every `stride` rows
    history: Rows,
    stride: usize,
    count: usize,
}
impl Signal {
    fn push(&mut self, step: i64, values: &[f32]) {
        if self.count % self.stride == 0 {
            self.history.push(step, values);
            let max_rows = HISTORY_ROWS.min((HISTORY_VALUES / self.n).max(2));
            if self.history.steps.len() > max_rows {
                self.history.halve(self.n);
                self.stride *= 2;
            }
        }
        self.count += 1;
    }
    fn notes(&self) -> Vec<String> {
        if self.stride > 1 {
            vec![format!("live: 1 row out of {} in the history", self.stride)]
        } else {
            vec![]
        }
    }
    fn meta(&self, hash: u64) -> Value {
        json!({
            "hash": hash.to_string(),
            "name": self.name,
            "file": self.file,
            "n": self.n,
            "shape": self.shape.map(|(w, h)| [w, h]),
            "pixel": self.pixel.as_ref().map(|(size, unit)| json!({"size": size, "unit": unit})),
        })
    }
}

/// A page
struct Client {
    tx: SyncSender<String>,
    // rows of the subscribed signals not sent yet
    pending: HashMap<u64, Rows>,
    step: Option<i64>,
}
impl Client {
    /// Sends an event, returns false if the page is gone
    fn send(&self, event: &str, data: &Value) -> bool {
        match self
            .tx
            .try_send(format!("event: {event}\ndata: {data}\n\n"))
        {
            Ok(()) | Err(TrySendError::Full(_)) => true,
            Err(TrySendError::Disconnected(_)) => false,
        }
    }
}

/// Signals history and pages
struct Hub {
    fs: Option<f64>,
    pixels: HashMap<String, (f64, String)>,
    title: String,
    // model JSON, `</` escaped, and parsed
    model: String,
    actors: HashMap<String, String>,
    channels: HashMap<String, String>,
    events: Option<String>,
    signals: BTreeMap<u64, Signal>,
    clients: HashMap<u64, Client>,
    step: Option<i64>,
    catalog: bool,
}
impl Hub {
    fn new(fs: Option<f64>, pixels: HashMap<String, (f64, String)>) -> Self {
        Self {
            fs,
            pixels,
            title: String::new(),
            model: "null".into(),
            actors: HashMap::new(),
            channels: HashMap::new(),
            events: None,
            signals: BTreeMap::new(),
            clients: HashMap::new(),
            step: None,
            catalog: false,
        }
    }
    fn run(mut self, rx: Receiver<Msg>) {
        let mut next = Instant::now() + FLUSH;
        loop {
            match rx.recv_timeout(next.saturating_duration_since(Instant::now())) {
                Ok(msg) => self.handle(msg),
                Err(RecvTimeoutError::Timeout) => (),
                Err(RecvTimeoutError::Disconnected) => break,
            }
            if Instant::now() >= next {
                self.flush();
                next = Instant::now() + FLUSH;
            }
        }
    }
    fn handle(&mut self, msg: Msg) {
        match msg {
            Msg::Row(row) => self.row(row),
            Msg::Model { title, model } => {
                let parsed: Value =
                    serde_json::from_str(&model.replace(r"<\/", "</")).unwrap_or(Value::Null);
                self.actors = parsed["actors"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(|a| {
                        Some((
                            a["uid"].as_str()?.to_string(),
                            a["label"].as_str()?.to_string(),
                        ))
                    })
                    .collect();
                self.channels = parsed["channels"]
                    .as_object()
                    .into_iter()
                    .flatten()
                    .filter_map(|(h, c)| Some((h.clone(), c["name"].as_str()?.to_string())))
                    .collect();
                self.title = title;
                self.model = model;
                self.events = None;
                self.broadcast("reload", &Value::Null);
            }
            Msg::Events(events, ack) => {
                let value: Value = serde_json::from_str(&events).unwrap_or(Value::Null);
                self.flush();
                self.histories();
                // the pages the recorded run is queued to, the page thread counts it down once written
                let event = format!("event: events\ndata: {value}\n\n");
                self.clients
                    .retain(|_, client| match client.tx.try_send(event.clone()) {
                        Ok(()) => {
                            DELIVERING.fetch_add(1, Ordering::AcqRel);
                            true
                        }
                        Err(TrySendError::Full(_)) => true,
                        Err(TrySendError::Disconnected(_)) => false,
                    });
                self.events = Some(events);
                let _ = ack.send(());
            }
            Msg::Connect(id, tx) => {
                let client = Client {
                    tx,
                    pending: HashMap::new(),
                    step: None,
                };
                client.send("hello", &json!({"id": id.to_string()}));
                client.send("signals", &self.catalog());
                self.clients.insert(id, client);
            }
            Msg::Disconnect(id) => {
                self.clients.remove(&id);
            }
            Msg::Subscribe(id, hash) => {
                let Some(client) = self.clients.get_mut(&id) else {
                    return;
                };
                if let Some(signal) = self.signals.get(&hash) {
                    let mut history = signal.history.to_json(hash);
                    history["notes"] = json!(signal.notes());
                    client.send("history", &history);
                    client.pending.insert(hash, Rows::default());
                }
            }
            Msg::Unsubscribe(id, hash) => {
                if let Some(client) = self.clients.get_mut(&id) {
                    client.pending.remove(&hash);
                }
            }
            Msg::Page(tx) => {
                let _ = tx.send(self.page());
            }
        }
    }
    fn row(&mut self, row: Row) {
        let n = row.values.len();
        if n == 0 {
            return;
        }
        let signal = self.signals.entry(row.hash).or_insert_with(|| {
            self.catalog = true;
            let name = self
                .channels
                .get(&row.hash.to_string())
                .cloned()
                .unwrap_or_else(|| row.hash.to_string());
            let file = self
                .actors
                .get(&row.uid.to_string())
                .cloned()
                .unwrap_or_default();
            let shape = {
                let w = (n as f64).sqrt().round() as usize;
                (w >= 32 && w * w == n).then_some((w, w))
            };
            let pixel = shape.and(
                self.pixels
                    .get(&name)
                    .or_else(|| self.pixels.get(&format!("{file}:{name}")))
                    .cloned(),
            );
            Signal {
                file,
                name,
                n,
                shape,
                pixel,
                history: Rows::default(),
                stride: 1,
                count: 0,
            }
        });
        if signal.n != n {
            return;
        }
        signal.push(row.step, &row.values);
        let image = signal.shape.is_some();
        for client in self.clients.values_mut() {
            if let Some(pending) = client.pending.get_mut(&row.hash) {
                pending.push(row.step, &row.values);
                // the images are sent once per update, the time series thinned if the page falls behind
                if image && pending.steps.len() > 1 {
                    pending.steps.remove(0);
                    pending.data.drain(..n);
                } else if pending.steps.len() > PENDING_ROWS {
                    pending.halve(n);
                }
            }
        }
        self.step = self.step.max(Some(row.step));
    }
    /// Sends to the pages the history of the signals they are not subscribed to,
    /// so they can be shown once the run is recorded and the server is gone
    fn histories(&mut self) {
        let signals = &self.signals;
        self.clients.retain(|_, client| {
            signals
                .iter()
                .filter(|(hash, _)| !client.pending.contains_key(hash))
                .all(|(&hash, signal)| {
                    let mut history = signal.history.to_json(hash);
                    history["notes"] = json!(signal.notes());
                    client.send("history", &history)
                })
        });
    }
    fn catalog(&self) -> Value {
        json!(
            self.signals
                .iter()
                .map(|(&hash, s)| s.meta(hash))
                .collect::<Vec<_>>()
        )
    }
    fn broadcast(&mut self, event: &str, data: &Value) {
        self.clients.retain(|_, client| client.send(event, data));
    }
    /// Sends the new signals, the new rows of the subscribed signals and the last step to the pages
    fn flush(&mut self) {
        if self.catalog {
            self.catalog = false;
            let catalog = self.catalog();
            self.broadcast("signals", &catalog);
        }
        let step = self.step;
        self.clients.retain(|_, client| {
            let updates: Vec<Value> = client
                .pending
                .iter_mut()
                .filter(|(_, pending)| !pending.steps.is_empty())
                .map(|(&hash, pending)| std::mem::take(pending).to_json(hash))
                .collect();
            if !updates.iter().all(|rows| client.send("rows", rows)) {
                return false;
            }
            if client.step != step {
                client.step = step;
                return client.send("step", &json!({ "step": step }));
            }
            true
        });
    }
    /// The live page: the flowchart without the signals, received from the server
    fn page(&self) -> String {
        let scope = json!({"live": true, "fs": self.fs, "signals": []}).to_string();
        let page = crate::elk::fill(
            &self.title,
            &self.model,
            self.events.as_deref().unwrap_or("null"),
            &scope,
        );
        crate::elk::with_echarts(page)
    }
}

/// Accepts the connections of the pages
fn serve(listener: TcpListener) {
    for stream in listener.incoming().flatten() {
        let _ = thread::Builder::new()
            .name("flowchart-live-client".into())
            .spawn(move || {
                if let Err(e) = request(stream) {
                    log::debug!("flowchart live request failed: {e}");
                }
            });
    }
}

/// Answers a request: the page, the events stream (`/events`) or a subscription (`/subscribe?id=&add=|remove=`)
fn request(stream: TcpStream) -> std::io::Result<()> {
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut line = String::new();
    reader.read_line(&mut line)?;
    // the headers are ignored
    let mut header = String::new();
    while reader.read_line(&mut header)? > 2 {
        header.clear();
    }
    let target = line.split_whitespace().nth(1).unwrap_or("/");
    let (path, query) = target.split_once('?').unwrap_or((target, ""));
    let params: HashMap<&str, &str> = query
        .split('&')
        .filter_map(|kv| kv.split_once('='))
        .collect();
    let mut stream = stream;
    let respond = |stream: &mut TcpStream, status: &str, kind: &str, body: &[u8]| {
        write!(
            stream,
            "HTTP/1.1 {status}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
            body.len()
        )?;
        stream.write_all(body)
    };
    match path {
        "/events" => events(stream),
        "/subscribe" => {
            let id = params.get("id").and_then(|id| id.parse().ok());
            let msg = match (id, params.get("add"), params.get("remove")) {
                (Some(id), Some(h), _) => h.parse().ok().map(|h| Msg::Subscribe(id, h)),
                (Some(id), _, Some(h)) => h.parse().ok().map(|h| Msg::Unsubscribe(id, h)),
                _ => None,
            };
            match msg {
                Some(msg) => {
                    send(msg);
                    respond(&mut stream, "204 No Content", "text/plain", b"")
                }
                None => respond(
                    &mut stream,
                    "400 Bad Request",
                    "text/plain",
                    b"bad subscription",
                ),
            }
        }
        "/" | "/index.html" => {
            let (tx, rx) = sync_channel(1);
            send(Msg::Page(tx));
            let page = rx.recv().unwrap_or_default();
            respond(
                &mut stream,
                "200 OK",
                "text/html; charset=utf-8",
                page.as_bytes(),
            )
        }
        _ => respond(&mut stream, "404 Not Found", "text/plain", b"not found"),
    }
}

/// Streams the events of the hub to a page
fn events(mut stream: TcpStream) -> std::io::Result<()> {
    write!(
        stream,
        "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\nConnection: keep-alive\r\n\r\nretry: 2000\n\n"
    )?;
    let id = CLIENT_ID.fetch_add(1, Ordering::Relaxed);
    let (tx, rx) = sync_channel(CLIENT_CAPACITY);
    send(Msg::Connect(id, tx));
    let result = (|| loop {
        match rx.recv_timeout(Duration::from_secs(10)) {
            Ok(event) => {
                stream.write_all(event.as_bytes())?;
                if event.starts_with(RECORDED) {
                    stream.flush()?;
                    DELIVERING.fetch_sub(1, Ordering::AcqRel);
                }
            }
            // a comment checks that the page is still there
            Err(RecvTimeoutError::Timeout) => stream.write_all(b": ping\n\n")?,
            Err(RecvTimeoutError::Disconnected) => return Ok(()),
        }
    })();
    send(Msg::Disconnect(id));
    // the recorded run is not delivered to a page that is gone
    rx.try_iter()
        .filter(|event| event.starts_with(RECORDED))
        .for_each(|_| {
            DELIVERING.fetch_sub(1, Ordering::AcqRel);
        });
    result
}

const RECORDED: &str = "event: events\n";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pixels() {
        let p = parse_pixels("Frame<Host>=3.93mas; Wavefront=1.6 cm");
        assert_eq!(p["Frame<Host>"], (3.93, "mas".to_string()));
        assert_eq!(p["Wavefront"], (1.6, "cm".to_string()));
    }

    #[test]
    fn history() {
        let mut signal = Signal {
            file: "f".into(),
            name: "x".into(),
            n: 1,
            shape: None,
            pixel: None,
            history: Rows::default(),
            stride: 1,
            count: 0,
        };
        for k in 0..(4 * HISTORY_ROWS) {
            signal.push(k as i64, &[k as f32]);
        }
        assert!(signal.history.steps.len() <= HISTORY_ROWS);
        assert_eq!(signal.stride, 4);
        let steps = &signal.history.steps;
        assert!(steps.windows(2).all(|w| w[1] - w[0] == 4));
        assert_eq!(signal.history.data[1], 4.);
    }

    #[test]
    fn hub() {
        let mut hub = Hub::new(Some(1000.), HashMap::new());
        hub.handle(Msg::Model {
            title: "MODEL".into(),
            model: json!({"name": "model", "actors": [{"uid": "7", "label": "logs"}], "channels": {"3": {"name": "X"}}})
                .to_string(),
        });
        let (tx, rx) = sync_channel(16);
        hub.handle(Msg::Connect(1, tx));
        assert!(rx.recv().unwrap().starts_with("event: hello"));
        assert_eq!(rx.recv().unwrap(), "event: signals\ndata: []\n\n");
        for step in 0..3 {
            hub.handle(Msg::Row(Row {
                uid: 7,
                hash: 3,
                step,
                values: vec![step as f32, 1.],
            }));
        }
        hub.handle(Msg::Subscribe(1, 3));
        hub.handle(Msg::Row(Row {
            uid: 7,
            hash: 3,
            step: 3,
            values: vec![3., 1.],
        }));
        hub.flush();
        let events: Vec<String> = rx.try_iter().collect();
        let event = |name: &str| -> Value {
            let e = events
                .iter()
                .find(|e| e.starts_with(&format!("event: {name}\n")))
                .unwrap();
            serde_json::from_str(e.lines().nth(1).unwrap().strip_prefix("data: ").unwrap()).unwrap()
        };
        assert_eq!(event("history")["steps"], json!([0, 1, 2]));
        assert_eq!(event("rows")["steps"], json!([3]));
        assert_eq!(event("step")["step"], 3);
        assert_eq!(event("signals")[0]["name"], "X");
        assert_eq!(event("signals")[0]["file"], "logs");
        assert!(hub.page().contains(r#""live":true"#));
        // the histories of the signals not subscribed to are sent when the run is recorded
        hub.handle(Msg::Row(Row { uid: 7, hash: 4, step: 4, values: vec![1.] }));
        let (ack, done) = sync_channel(1);
        hub.handle(Msg::Events(json!({"steps": 5}).to_string(), ack));
        assert!(done.try_recv().is_ok());
        DELIVERING.store(0, Ordering::Release);
        let events: Vec<String> = rx.try_iter().collect();
        let histories: Vec<&String> = events.iter().filter(|e| e.starts_with("event: history\n")).collect();
        assert_eq!(histories.len(), 1);
        assert!(histories[0].contains(r#""hash":"4""#));
        assert!(events.last().unwrap().starts_with("event: events\n"));
    }
}
