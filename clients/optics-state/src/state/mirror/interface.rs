use std::mem;

use gmt_dos_clients_io::{
    gmt_m1::{self, M1ModeShapes, M1RigidBodyMotions},
    gmt_m2::M2RigidBodyMotions,
};
use interface::{Data, Left, Read, Right, Update, Write};

use crate::{M1State, M2State, MirrorState, SegmentState};

impl Update for MirrorState {}

impl Read<M1State> for MirrorState {
    fn read(&mut self, data: Data<M1State>) {
        let state = &*data;
        let _ = mem::replace(self, state.clone());
    }
}

impl Read<M2State> for MirrorState {
    fn read(&mut self, data: Data<M2State>) {
        let state = &*data;
        let _ = mem::replace(self, state.clone());
    }
}

impl Read<M2RigidBodyMotions> for MirrorState {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.segment = data
            .chunks(6)
            .map(|data| SegmentState::rbms(data))
            .map(|segment| Some(segment))
            .collect();
    }
}

impl Read<M1RigidBodyMotions> for MirrorState {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        self.segment = data
            .chunks(6)
            .map(|data| SegmentState::rbms(data))
            .map(|segment| Some(segment))
            .collect();
    }
}

impl Read<M1ModeShapes> for MirrorState {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        self.segment = data
            .chunks(data.len() / 7)
            .map(|data| SegmentState::modes(data))
            .map(|segment| Some(segment))
            .collect();
    }
}
impl Read<Right<M1ModeShapes>> for MirrorState {
    fn read(&mut self, data: Data<Right<M1ModeShapes>>) {
        self.segment = data
            .chunks(data.len() / 7)
            .map(|data| SegmentState::modes(data))
            .map(|segment| Some(segment))
            .collect();
    }
}
impl Read<Left<M1ModeShapes>> for MirrorState {
    fn read(&mut self, data: Data<Left<M1ModeShapes>>) {
        self.segment = data
            .chunks(data.len() / 7)
            .map(|data| SegmentState::modes(data))
            .map(|segment| Some(segment))
            .collect();
    }
}

impl<const ID: u8> Write<gmt_m1::segment::RBM<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<gmt_m1::segment::RBM<ID>>> {
        Some(
            self.iter()
                .nth(ID as usize - 1)
                .flatten()
                .map_or_else(
                    || vec![0f64; 6],
                    |SegmentState { rbms, .. }| {
                        rbms.as_ref()
                            .map_or_else(|| vec![0f64; 6], |rbms| rbms.as_ref().to_vec())
                    },
                )
                .into(),
        )
    }
}

impl<const ID: u8> Write<gmt_m1::segment::ModeShapes<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<gmt_m1::segment::ModeShapes<ID>>> {
        Some(
            self.iter()
                .nth(ID as usize - 1)
                .flatten()
                .map_or_else(
                    || vec![],
                    |SegmentState { modes, .. }| {
                        modes
                            .as_ref()
                            .map_or_else(|| vec![], |modes| modes.as_ref().to_vec())
                    },
                )
                .into(),
        )
    }
}

impl Write<M1State> for MirrorState {
    fn write(&mut self) -> Option<Data<M1State>> {
        Some(Data::new(self.clone()))
    }
}

impl Write<M2State> for MirrorState {
    fn write(&mut self) -> Option<Data<M2State>> {
        Some(Data::new(self.clone()))
    }
}
