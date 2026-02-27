#[cfg(feature = "gmt_dos-clients_arrow")]
pub mod arrow;
mod state;
pub use state::{MirrorState, OpticalState, SegmentState};

use interface::UniqueIdentifier;
///
/// M1 optics state
pub enum M1State {}
impl UniqueIdentifier for M1State {
    type DataType = MirrorState;
    const PORT: u16 = 50_012;
}
/// M2 optics state
pub enum M2State {}
impl UniqueIdentifier for M2State {
    type DataType = MirrorState;
    const PORT: u16 = 50_013;
}

pub enum OpticsState {}
impl UniqueIdentifier for OpticsState {
    type DataType = OpticalState;
}
