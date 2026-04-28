# Async Code Audit — `gmt_dos-actors`

Audit date: 2026-04-28

---

## Critical

### 1. `unwrap()` on task builder spawn — `model/ready.rs:36`

```rust
tokio::task::Builder::new()
    .name(actor.name())
    .spawn(async move { actor.task().await })
    .unwrap()  // panics if spawn fails
```

Active only under `#[cfg(tokio_unstable)]`, but a spawn failure during model startup
will panic the entire runtime instead of propagating a recoverable error.

**Recommendation:** Return the `Result` from `spawn()` and propagate it through
`Model::run()` rather than unwrapping.

---

## High

### 2. `unwrap()` inside `Clone` impl — `system.rs:80`

```rust
fn clone(&self) -> Self {
    let mut sys = self.sys.clone();
    sys.build().unwrap();  // panics if build fails
    ...
}
```

`Clone` must not panic on a recoverable failure. If `build()` can fail, the impl
should either be replaced by a fallible `try_clone() -> Result<Self>` method or
`build()` should be made infallible.

### 3. `panic!()` used as error handling — `framework/network/logs.rs:41,49,75,83` and `framework/network/inputs.rs:63,66`

Configuration errors such as exhausted receivers or empty input lists are expressed
as `panic!()` instead of being returned as `Err`. These fire during model construction
before any async task is running, but they are unrecoverable and produce unhelpful
backtraces.

**Recommendation:** Convert the affected functions to return `Result` and propagate
errors up to `Model::check()` / `Model::run()`.

### 4. `blocking_lock()` in async context — `framework/network/logs.rs:52,86–90`

```rust
task::block_in_place(|| actor.client().blocking_lock().entry(size));
```

The call is wrapped in `block_in_place`, so it will not deadlock on a multi-threaded
runtime. However, it silently requires `tokio::runtime::Builder::new_multi_thread()`
and will panic on the current-thread runtime.

**Recommendation:** Either convert to `lock().await` (requires making the call site
async) or restructure the logging setup to occur outside any async context so that
`block_in_place` is not needed.

---

## Medium

### 5. Sequential task awaiting — `model/running.rs:17`

```rust
for task_handle in task_handles.into_iter() {
    match task_handle.await? { ... }
}
```

Tasks are awaited one at a time in iteration order. If task *N* is slow, tasks
*N+1…M* are not polled at all until it completes, and the first error from any
later task is invisible until all earlier tasks finish.

**Recommendation:** Replace the `for` loop with `futures::future::try_join_all()` or
`tokio::task::JoinSet` so all handles are driven concurrently and the first error
surfaces immediately.

```rust
// example
use futures::future::try_join_all;

try_join_all(task_handles).await?;
```

### 6. No cancellation or shutdown signal

There is no `CancellationToken` or `tokio::sync::watch` shutdown channel. The only
way a task stops is by receiving a `DropRecv` / `DropSend` / `Disconnected` error
from the channel layer. A task that hangs — e.g., blocked on a `recv_async().await`
with no sender — will stall `Model::wait()` indefinitely with no way to interrupt it.

**Recommendation:** Thread a `tokio_util::sync::CancellationToken` into each actor
task and use `tokio::select!` to race channel operations against the cancellation
signal. Expose a `Model::abort()` method that triggers the token.

```rust
tokio::select! {
    result = self.collect() => result?,
    _ = token.cancelled() => return Err(ActorError::Cancelled),
}
```

### 7. No timeout support in `Model::wait()`

There is no way to impose a deadline on model execution. A slow or stuck model
blocks the caller forever.

**Recommendation:** Add an optional `timeout` parameter or a companion
`Model::wait_timeout(duration)` method wrapping the await in
`tokio::time::timeout()`.

### 8. Default channel buffer size of 1 — `framework/network/outputs/builder.rs:12`

```rust
capacity: vec![1; n],
```

A buffer of 1 causes every send to block until the downstream actor consumes the
value. This is a valid backpressure design, but it tightly serialises actors that
could otherwise run ahead by a few steps, and it cannot be tuned without touching
source code.

**Recommendation:** Keep the current default but document the design intent and
expose a per-output capacity builder method (or an environment variable) for users
who need higher throughput at the cost of memory.

---

## Low

### 9. Inconsistent `Arc::clone` vs `.clone()` — `framework/network/outputs.rs`

Some call sites use `std::sync::Arc::clone(&x)`, others use `x.clone()`.  Both
compile to the same code, but `Arc::clone` makes the intent explicit and is the
style preferred by Clippy's `clippy::clone_on_ref_ptr` lint.

**Recommendation:** Standardise on `Arc::clone(&x)` throughout the crate.

### 10. Full data clone per fanout sender — `actor/io/output.rs:152`

```rust
try_join_all(self.tx.iter().map(|tx| tx.send_async(data.clone())))
```

Each subscriber in a multiplexed output receives a full clone of the payload.
For large data vectors with many subscribers this multiplies allocation linearly.

**Recommendation:** Wrap the outgoing value in `Arc<_>` before the fanout so that
all senders share a single allocation. This requires adjusting the channel type from
`S<U>` to `Arc<S<U>>` on the fanout path.

---

## What is working well

- `tokio::sync::Mutex` is used correctly throughout — no `std::sync::Mutex` is held
  across `.await` points.
- The client lock is always released before channel sends (`actor/io/output.rs:143–159`).
- All `JoinHandle`s are stored in `Model.task_handles` and awaited — no orphaned tasks.
- Initiator tasks call `tokio::task::yield_now().await` before entering their loop
  (`actor/task.rs:93`), cooperating with the scheduler.
- `futures::future::try_join_all()` is used for concurrent multi-sender dispatch in
  the output layer.

---

## Priority order

| # | Location | Severity | Effort |
|---|----------|----------|--------|
| 1 | `model/ready.rs:36` | Critical | Low |
| 3 | `framework/network/logs.rs`, `inputs.rs` | High | Medium |
| 2 | `system.rs:80` | High | Medium |
| 5 | `model/running.rs:17` | Medium | Low |
| 4 | `framework/network/logs.rs:52,86` | High | Medium |
| 6 | All actor tasks | Medium | High |
| 7 | `model/running.rs` | Medium | Low |
| 8 | `framework/network/outputs/builder.rs:12` | Medium | Low |
| 10 | `actor/io/output.rs:152` | Low | Medium |
| 9 | `framework/network/outputs.rs` | Low | Low |
