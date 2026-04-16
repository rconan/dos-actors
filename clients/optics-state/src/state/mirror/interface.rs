pub mod units;

use std::sync::Arc;

use gmt_dos_clients_io::{
    gmt_m1::{self, M1ModeShapes, M1RigidBodyMotions},
    gmt_m2::M2RigidBodyMotions,
};
use interface::{Data, Left, Read, Right, Update, Write};

use crate::{M1State, M2State, MirrorState, SegmentState};

impl Update for MirrorState {}

impl Read<M1State> for MirrorState {
    fn read(&mut self, data: Data<M1State>) {
        *self = if let Some(zero_point) = self.get_zero_point().take() {
            (&*data + &zero_point).set_zero_point(zero_point)
        } else {
            (&*data).clone()
        };
    }
}

impl Read<M2State> for MirrorState {
    fn read(&mut self, data: Data<M2State>) {
        *self = if let Some(zero_point) = self.get_zero_point().take() {
            (&*data + &zero_point).set_zero_point(zero_point)
        } else {
            (&*data).clone()
        };
    }
}

impl Read<M2RigidBodyMotions> for MirrorState {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        data.chunks(6)
            .zip(&mut self.segment)
            .for_each(|(data, segment)| {
                segment.get_or_insert_default().rbms = Some(Arc::new(data.to_vec()));
            });
        if let Some(zero_point) = self.get_zero_point().take() {
            *self = (&self.clone() + &zero_point).set_zero_point(zero_point);
        }
    }
}

impl Read<M1RigidBodyMotions> for MirrorState {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        data.chunks(6)
            .zip(&mut self.segment)
            .for_each(|(data, segment)| {
                segment.get_or_insert_default().rbms = Some(Arc::new(data.to_vec()));
            });
        if let Some(zero_point) = self.get_zero_point().take() {
            *self = (&self.clone() + &zero_point).set_zero_point(zero_point);
        }
    }
}

impl Read<M1ModeShapes> for MirrorState {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        data.chunks(data.len() / 7)
            .zip(&mut self.segment)
            .for_each(|(data, segment)| {
                segment.get_or_insert_default().modes = Some(Arc::new(data.to_vec()));
            });
        if let Some(zero_point) = self.get_zero_point().take() {
            *self = (&self.clone() + &zero_point).set_zero_point(zero_point);
        }
    }
}
impl Read<Right<M1ModeShapes>> for MirrorState {
    fn read(&mut self, data: Data<Right<M1ModeShapes>>) {
        <_ as Read<M1ModeShapes>>::read(self, data.transmute())
    }
}
impl Read<Left<M1ModeShapes>> for MirrorState {
    fn read(&mut self, data: Data<Left<M1ModeShapes>>) {
        <_ as Read<M1ModeShapes>>::read(self, data.transmute())
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
impl Write<M2RigidBodyMotions> for MirrorState {
    fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
        Some(
            self.iter()
                .flat_map(|segment| {
                    segment.map_or_else(
                        || vec![0f64; 6],
                        |SegmentState { rbms, .. }| {
                            rbms.as_ref()
                                .map_or_else(|| vec![0f64; 6], |rbms| rbms.as_ref().to_vec())
                        },
                    )
                })
                .collect::<Vec<_>>()
                .into(),
        )
    }
}
impl Write<Left<M2RigidBodyMotions>> for MirrorState {
    fn write(&mut self) -> Option<Data<Left<M2RigidBodyMotions>>> {
        <_ as Write<M2RigidBodyMotions>>::write(self).map(|data| data.transmute())
    }
}
impl Write<Left<M1RigidBodyMotions>> for MirrorState {
    fn write(&mut self) -> Option<Data<Left<M1RigidBodyMotions>>> {
        <_ as Write<M1RigidBodyMotions>>::write(self).map(|data| data.transmute())
    }
}
impl Write<M1RigidBodyMotions> for MirrorState {
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        Some(
            self.iter()
                .flat_map(|segment| {
                    segment.map_or_else(
                        || vec![0f64; 6],
                        |SegmentState { rbms, .. }| {
                            rbms.as_ref()
                                .map_or_else(|| vec![0f64; 6], |rbms| rbms.as_ref().to_vec())
                        },
                    )
                })
                .collect::<Vec<_>>()
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
        Some(Data::new(self.clone().no_zero_point()))
    }
}

impl Write<M2State> for MirrorState {
    fn write(&mut self) -> Option<Data<M2State>> {
        Some(Data::new(self.clone().no_zero_point()))
    }
}
