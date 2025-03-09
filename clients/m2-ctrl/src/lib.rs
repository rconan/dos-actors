#[cfg(topend = "ASM")]
mod asm;
#[cfg(topend = "ASM")]
pub use asm::*;
#[cfg(topend = "FSM")]
mod fsm;
#[cfg(topend = "FSM")]
pub use fsm::*;

#[cfg(topend = "ASM")]
pub type M2SegmentInnerController<const SID: u8> = AsmSegmentInnerController<SID>;
#[cfg(topend = "FSM")]
pub type M2SegmentInnerController<const SID: u8> = FsmSegmentInnerController<SID>;

mod positioner;
pub use positioner::{Positioners, PositionersError};
