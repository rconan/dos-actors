//! Recording of the actor loop for the flowchart replay (feature `flowchart-replay`)
//!
//! Each actor keeps its own sample step: an actor with inputs is updated every `NI` steps
//! and an actor without inputs every `NO` steps.
//! Without the feature, [Replay] does nothing.

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
    use gmt_dos_actors_flowchart::replay;
    use interface::TryUpdate;

    use super::{super::Actor, uid};

    /// Actor loop recorder
    pub(crate) struct Replay {
        uid: u64,
        // step of the next update
        step: usize,
        // step of the last update
        last: usize,
        period: usize,
        outputs: Vec<u64>,
    }
    impl Replay {
        pub fn new<C, const NI: usize, const NO: usize>(actor: &Actor<C, NI, NO>) -> Self
        where
            C: TryUpdate,
        {
            Self {
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
            }
        }
        /// Records the bootstrapped outputs
        pub fn bootstrap<C, const NI: usize, const NO: usize>(&self, actor: &Actor<C, NI, NO>)
        where
            C: TryUpdate,
        {
            replay::record_send(
                None,
                actor
                    .outputs
                    .iter()
                    .flatten()
                    .filter(|output| output.bootstrap())
                    .map(|output| output.get_hash()),
            );
        }
        /// Records an update of the client
        pub fn update(&mut self) {
            replay::record_update(self.uid, self.step);
            self.last = self.step;
            self.step += self.period;
        }
        /// Records the outputs sent `offset` steps after the last update
        pub fn send(&self, offset: usize) {
            replay::record_send(Some(self.last + offset), self.outputs.iter().copied());
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
        pub fn send(&self, _: usize) {}
    }
}
