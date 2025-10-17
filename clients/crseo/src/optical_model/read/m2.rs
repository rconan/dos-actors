use gmt_dos_clients_io::{
    gmt_m2::{
        M2RigidBodyMotions,
        asm::{
            M2ASMAsmCommand, M2ASMFaceSheetFigure,
            segment::{AsmCommand, FaceSheetFigure},
        },
    },
    optics::{M2GlobalTipTilt, M2GlobalTxyz, M2Modes},
};
use interface::{
    Data, Read,
    optics::{M2State, state::SegmentState},
};

use crate::{OpticalModel, sensors::SensorPropagation};

impl<T: SensorPropagation> Read<M2RigidBodyMotions> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        data.chunks(6).enumerate().for_each(|(sid, data)| {
            self.gmt
                .m2_segment_state(1 + sid as i32, &data[..3], &data[3..]);
        });
    }
}

impl<T: SensorPropagation, const SID: u8> Read<AsmCommand<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<AsmCommand<SID>>) {
        self.gmt.m2_segment_modes(SID, &data);
    }
}

impl<T: SensorPropagation> Read<M2ASMAsmCommand> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2ASMAsmCommand>) {
        self.gmt.m2_modes(&data);
    }
}

impl<T: SensorPropagation> Read<M2GlobalTipTilt> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2GlobalTipTilt>) {
        let rbms = geotrans::Mirror::<geotrans::M2>::tiptilt_2_rigidbodymotions((data[0], data[1]));
        /* rbms.chunks(6).enumerate().for_each(|(i, c)| {
            println!(
                "S{}, {:+7.0?} {:+7.0?}",
                i + 1,
                c[..3].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
                c[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
            )
        }); */
        <OpticalModel<T> as Read<M2RigidBodyMotions>>::read(self, rbms.into())
    }
}
impl<T: SensorPropagation> Read<M2GlobalTxyz> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2GlobalTxyz>) {
        let rbms = geotrans::Mirror::<geotrans::M2>::translations_2_rigidbodymotions((
            data[0], data[1], data[2],
        ));
        <OpticalModel<T> as Read<M2RigidBodyMotions>>::read(self, rbms.into())
    }
}

// impl<T: SensorPropagation> Read<M2Modes> for OpticalModel<T> {
//     fn read(&mut self, data: Data<M2Modes>) {
//         assert_eq!(7 * self.gmt.m2.n_mode, data.len());
//         // if 7 * self.gmt.m2.n_mode > data.len() {
//         //     let augmented_data: Vec<_> = data
//         //         .chunks(data.len() / 7)
//         //         .flat_map(|data| {
//         //             let mut v = vec![0f64];
//         //             v.extend_from_slice(data);
//         //             v
//         //         })
//         //         .collect();
//         //     assert_eq!(augmented_data.len(), self.gmt.m2.n_mode * 7);
//         //     self.gmt.m2_modes(&augmented_data);
//         // } else {
//         self.gmt.m2_modes(&data);
//         // }
//     }
// }
impl<T: SensorPropagation> Read<M2Modes> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2Modes>) {
        // assert_eq!(7 * self.gmt.m2.n_mode, data.len());
        if 7 * self.gmt.m2.n_mode > data.len() {
            let augmented_data: Vec<_> = data
                .chunks(data.len() / 7)
                .flat_map(|data| {
                    let mut v = vec![0f64];
                    v.extend_from_slice(data);
                    v
                })
                .collect();
            assert_eq!(augmented_data.len(), self.gmt.m2.n_mode * 7);
            self.gmt.m2_modes(&augmented_data);
        } else {
            self.gmt.m2_modes(&data);
        }
    }
}

impl<T: SensorPropagation> Read<M2State> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2State>) {
        let state = data.into_arc();
        for (sid, SegmentState { rbms, modes }) in state
            .iter()
            .enumerate()
            .filter_map(|(i, state)| state.map(|state| (i + 1, state)))
        {
            if let Some(data) = rbms {
                self.gmt
                    .m2_segment_state(sid as i32, &data[..3], &data[3..]);
            }
            if let Some(data) = modes {
                self.gmt.m2_segment_modes(sid as u8, &data);
            }
        }
    }
}

impl<T: SensorPropagation, const ID: u8> Read<FaceSheetFigure<ID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<FaceSheetFigure<ID>>) {
        self.gmt.m2_segment_modes(ID, &data);
    }
}
impl<T: SensorPropagation> Read<M2ASMFaceSheetFigure> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2ASMFaceSheetFigure>) {
        let q: Vec<_> = data.iter().flatten().cloned().collect();
        self.gmt.m2_modes(q.as_slice());
    }
}
