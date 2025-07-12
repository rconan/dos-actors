pub mod state;
use state::{MirrorState, OpticalState};

use crate::{Data, Read, UniqueIdentifier, Write};
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
pub trait Optics {}
impl<C: Write<M1State> + Write<M2State> + Optics> Write<OpticsState> for C {
    fn write(&mut self) -> Option<Data<OpticsState>> {
        let s1 = <_ as Write<M1State>>::write(self);
        let s2 = <_ as Write<M2State>>::write(self);
        match (&s1, &s2) {
            (None, None) => None,
            _ => Some(Data::new(OpticalState {
                m1: s1.map(|data| (*data).clone()),
                m2: s2.map(|data| (*data).clone()),
            })),
        }
    }
}
impl<C: Read<M1State> + Read<M2State> + Optics> Read<OpticsState> for C {
    fn read(&mut self, data: Data<OpticsState>) {
        let OpticalState { m1, m2 } = &*data;
        if let Some(s1) = m1 {
            <_ as Read<M1State>>::read(self, Data::new(s1.clone()));
        }
        if let Some(s2) = m2 {
            <_ as Read<M2State>>::read(self, Data::new(s2.clone()));
        }
    }
}
