use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions};
use interface::{Data, Read, Update, Write};

use crate::{M1State, M2State, MirrorState, SegmentState};

impl Update for MirrorState {}

impl Read<M2RigidBodyMotions> for MirrorState {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.segment = data
            .chunks(6)
            .map(|data| SegmentState::rbms(data))
            .map(|segment| Some(segment))
            .collect();
    }
}

impl Write<M2State> for MirrorState {
    fn write(&mut self) -> Option<Data<M2State>> {
        Some(Data::new(self.clone()))
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

impl Write<M1State> for MirrorState {
    fn write(&mut self) -> Option<Data<M1State>> {
        Some(Data::new(self.clone()))
    }
}
