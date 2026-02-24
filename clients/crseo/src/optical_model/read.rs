use gmt_dos_clients_optics_state::{M1State, M2State, OpticalState, OpticsState};
use interface::{Data, Read};

use crate::{OpticalModel, sensors::SensorPropagation};

mod m1;
mod m2;

impl<T: SensorPropagation> Read<OpticsState> for OpticalModel<T>
where
    OpticalModel: Read<M1State> + Read<M2State>,
{
    fn read(&mut self, data: Data<OpticsState>) {
        let OpticalState { m1, m2, .. } = &*data;
        if let Some(s1) = m1 {
            <_ as Read<M1State>>::read(self, Data::new(s1.clone()));
        }
        if let Some(s2) = m2 {
            <_ as Read<M2State>>::read(self, Data::new(s2.clone()));
        }
    }
}
