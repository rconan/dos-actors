//! Recording of the actor loop for the flowchart replay (feature `flowchart-replay`)
//!
//! Each actor keeps its own sample step: an actor with inputs is updated every `NI` steps
//! and an actor without inputs every `NO` steps.
//! Without the feature, [Replay] does nothing.
//!
//! With the feature `flowchart-live`, the actor loop runs within a task-local [live] context
//! (the actor and its sample step), so the data read by a logger can be streamed into the flowchart page.

use interface::TryUpdate;

use super::Actor;

/// Identifier of a running actor: the address of its client
pub(crate) fn uid<C, const NI: usize, const NO: usize>(actor: &Actor<C, NI, NO>) -> u64
where
    C: TryUpdate,
{
    std::sync::Arc::as_ptr(&actor.client) as *const () as usize as u64
}

#[cfg(feature = "flowchart-replay")]
pub(crate) use recording::Replay;
#[cfg(not(feature = "flowchart-replay"))]
pub(crate) use noop::Replay;

#[cfg(feature = "flowchart-replay")]
mod recording {
    use gmt_dos_actors_flowchart::replay::{self, Runs};
    use interface::TryUpdate;

    use super::{super::Actor, uid};

    /// Actor loop recorder
    ///
    /// The steps are recorded locally and handed over to the replay session
    /// when the recorder is dropped at the end of the actor loop
    pub(crate) struct Replay {
        active: bool,
        max_steps: usize,
        uid: u64,
        // step of the next update
        step: usize,
        // step of the last update
        last: usize,
        period: usize,
        outputs: Vec<u64>,
        updates: Runs,
        sends: Runs,
    }
    impl Replay {
        pub fn new<C, const NI: usize, const NO: usize>(actor: &Actor<C, NI, NO>) -> Self
        where
            C: TryUpdate,
        {
            Self {
                active: replay::is_recording(),
                max_steps: replay::max_steps(),
                uid: uid(actor),
                step: 0,
                last: 0,
                period: if actor.inputs.is_some() { NI } else { NO },
                outputs: actor
                    .outputs
                    .iter()
                    .flatten()
                    .map(|output| output.get_hash())
                    .collect(),
                updates: Runs::default(),
                sends: Runs::default(),
            }
        }
        /// Records the bootstrapped outputs
        pub fn bootstrap<C, const NI: usize, const NO: usize>(&self, actor: &Actor<C, NI, NO>)
        where
            C: TryUpdate,
        {
            if self.active {
                replay::record_bootstrap(
                    actor
                        .outputs
                        .iter()
                        .flatten()
                        .filter(|output| output.bootstrap())
                        .map(|output| output.get_hash()),
                );
            }
        }
        /// Records an update of the client
        #[inline]
        pub fn update(&mut self) {
            if self.active && self.step < self.max_steps {
                self.updates.push(self.step);
            }
            self.last = self.step;
            self.step += self.period;
            #[cfg(feature = "flowchart-live")]
            super::live::set_step(self.step);
        }
        /// Records the outputs sent `offset` steps after the last update
        #[inline]
        pub fn send(&mut self, offset: usize) {
            let step = self.last + offset;
            if self.active && step < self.max_steps {
                self.sends.push(step);
            }
        }
    }
    impl Drop for Replay {
        fn drop(&mut self) {
            if self.active {
                replay::record(
                    self.uid,
                    std::mem::take(&mut self.updates),
                    &self.outputs,
                    std::mem::take(&mut self.sends),
                );
            }
        }
    }
}

/// Context of the actors for the live streaming of the flowchart (feature `flowchart-live`)
#[cfg(feature = "flowchart-live")]
pub(crate) mod live {
    use std::{cell::Cell, future::Future};

    use gmt_dos_actors_flowchart::live::{self, Context};

    tokio::task_local! {
        // the actor uid and its sample step
        static ACTOR: Cell<(u64, usize)>;
    }

    /// Runs the actor loop `f` of the actor `uid` within the actor context
    pub(crate) async fn scope<F: Future>(uid: u64, f: F) -> F::Output {
        ACTOR.scope(Cell::new((uid, 0)), f).await
    }

    /// Sets the sample step of the actor
    #[inline]
    pub(crate) fn set_step(step: usize) {
        let _ = ACTOR.try_with(|actor| actor.set((actor.get().0, step)));
    }

    /// Runs `f`, the client reading the data of the channel `hash`, within the actor context
    #[inline]
    pub(crate) fn read<R>(hash: u64, f: impl FnOnce() -> R) -> R {
        if !live::is_active() {
            return f();
        }
        match ACTOR.try_with(Cell::get) {
            Ok((uid, step)) => live::with(Context { uid, step, hash }, f),
            Err(_) => f(),
        }
    }
}

#[cfg(not(feature = "flowchart-replay"))]
mod noop {
    use interface::TryUpdate;

    use super::super::Actor;

    /// Actor loop recorder, it does nothing without the feature `flowchart-replay`
    pub(crate) struct Replay;
    impl Replay {
        #[inline(always)]
        pub fn new<C, const NI: usize, const NO: usize>(_: &Actor<C, NI, NO>) -> Self
        where
            C: TryUpdate,
        {
            Self
        }
        #[inline(always)]
        pub fn bootstrap<C, const NI: usize, const NO: usize>(&self, _: &Actor<C, NI, NO>)
        where
            C: TryUpdate,
        {
        }
        #[inline(always)]
        pub fn update(&mut self) {}
        #[inline(always)]
        pub fn send(&mut self, _: usize) {}
    }
}
