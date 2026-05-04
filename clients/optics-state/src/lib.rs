#[cfg(feature = "gmt_dos-clients_arrow")]
pub mod arrow;
mod state;
pub use state::{MirrorState, OpticalState, SegmentState, units};

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

#[cfg(test)]
mod tests {
    use interface::{Data, Read, Update};

    use super::*;

    #[test]
    fn reset_state() {
        let zp = OpticalState::m2(MirrorState::default().set_segment_state(
            1,
            SegmentState::rbms({
                let mut rbms = vec![0f64; 7];
                rbms[0] = 1e-6;
                rbms
            }),
        ));
        let mut state = OpticalState::default().set_zero_point(zp);
        dbg!(&state);

        let m2_state = MirrorState::default().set_segment_state(
            1,
            SegmentState::rbms({
                let mut rbms = vec![0f64; 7];
                rbms[0] = -0.25e-6;
                rbms[1] = -1e-6;
                rbms
            }),
        ).set_segment_state(
            2,
            SegmentState::rbms({
                let mut rbms = vec![0f64; 7];
                rbms[1] = -1e-6;
                rbms
            }),
        );
        dbg!(&m2_state);
        <_ as Read<M2State>>::read(&mut state, Data::new(m2_state.clone()));
        state.update();
        dbg!(&state);
        <_ as Read<M2State>>::read(&mut state, Data::new(m2_state));
        state.update();
        dbg!(&state);
    }

    // #[test]
    // fn interface() {
    //     let m1 = MirrorState::default();
    //     let m2 = MirrorState::default()
    //         .set_segment_state(1, SegmentState::rbms([1e-6, 0., 0., 0., 0., 0.]));
    //     let mut optical_state = OpticalState::m1(MirrorState::default().zeros_modes(3))
    //         .set_zero_point(OpticalState::new(&m1, &m2));

    //     <_ as Read<OpticsState>>::read(&mut optical_state, Data::new(Default::default()));
    //     let reference = OpticalState::m2(&m2).set_zero_point(OpticalState::m2(m2));
    //     assert_eq!(reference, optical_state);
    // }
}
