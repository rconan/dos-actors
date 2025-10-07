use gmt_dos_clients_io::{
    gmt_m1::{
        M1ModeShapes, M1RigidBodyMotions,
        assembly::M1ModeCoefficients,
        segment::{ModeShapes, RBM},
    },
    optics::{M1GlobalTipTilt, M1GlobalTxyz, M1Modes},
};
use interface::{
    Data, Read,
    optics::{M1State, state::SegmentState},
};

use crate::{OpticalModel, sensors::SensorPropagation};

impl<T: SensorPropagation, const SID: u8> Read<RBM<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<RBM<SID>>) {
        self.gmt
            .m1_segment_state(SID as i32, &data[..3], &data[3..]);
    }
}

impl<T: SensorPropagation> Read<M1RigidBodyMotions> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        data.chunks(6).enumerate().for_each(|(sid, data)| {
            self.gmt
                .m1_segment_state(1 + sid as i32, &data[..3], &data[3..]);
        });
    }
}

impl<T: SensorPropagation, const SID: u8> Read<ModeShapes<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<ModeShapes<SID>>) {
        self.gmt.m1_segment_modes(SID, &data);
    }
}

impl<T: SensorPropagation> Read<M1Modes> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1Modes>) {
        assert_eq!(7 * self.gmt.m1.n_mode, data.len());
        self.gmt.m1_modes(&data);
    }
}
impl<T: SensorPropagation> Read<M1State> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1State>) {
        let state = data.into_arc();
        for (sid, SegmentState { rbms, modes }) in state
            .iter()
            .enumerate()
            .filter_map(|(i, state)| state.map(|state| (i + 1, state)))
        {
            if let Some(data) = rbms {
                self.gmt
                    .m1_segment_state(sid as i32, &data[..3], &data[3..]);
            }
            if let Some(data) = modes {
                self.gmt.m1_segment_modes(sid as u8, &data);
            }
        }
        // if let Some(rbms) = state.into_rbms() {
        //     <Self as Read<M1RigidBodyMotions>>::read(self, rbms.into());
        // }
        // if let Some(modes) = state.into_modes() {
        //     <Self as Read<M1ModeShapes>>::read(self, modes.into());
        // }
    }
}
impl<T: SensorPropagation> Read<M1ModeShapes> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        self.gmt.m1_modes(&data);
    }
}
impl<T: SensorPropagation> Read<M1ModeCoefficients> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1ModeCoefficients>) {
        self.gmt.m1_modes(&data);
    }
}
impl<T: SensorPropagation> Read<M1GlobalTipTilt> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1GlobalTipTilt>) {
        let rbms = geotrans::Mirror::<geotrans::M1>::tiptilt_2_rigidbodymotions((data[0], data[1]));
        <OpticalModel<T> as Read<M1RigidBodyMotions>>::read(self, rbms.into())
    }
}
impl<T: SensorPropagation> Read<M1GlobalTxyz> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1GlobalTxyz>) {
        let rbms =
            geotrans::Mirror::<geotrans::M1>::translations_2_rigidbodymotions((data[0], data[1], data[2]));
        <OpticalModel<T> as Read<M1RigidBodyMotions>>::read(self, rbms.into())
    }
}
