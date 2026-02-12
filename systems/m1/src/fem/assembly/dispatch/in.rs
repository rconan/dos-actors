use std::sync::Arc;

use serde::{Deserialize, Serialize};

#[cfg(m1_hp_force_extension)]
use gmt_dos_clients_io::gmt_m1::{assembly::M1HardpointsForces, segment::HardpointsForces};
#[cfg(not(m1_hp_force_extension))]
use gmt_dos_clients_io::gmt_m1::{assembly::M1HardpointsMotion, segment::HardpointsMotion};
use gmt_dos_clients_io::{
    Assembly,
    gmt_m1::{
        M1ModeShapes,
        assembly::{M1ActuatorCommandForces, M1RigidBodyMotions},
        segment::{ActuatorCommandForces, ModeShapes, RBM},
    },
};
use interface::{
    Data, Left, Read, Right, UniqueIdentifier, Update, Write,
    optics::{
        M1State,
        state::{MirrorState, SegmentState},
    },
};

use super::NA;

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchIn
where
    Self: Assembly,
{
    m1_rigid_body_motions: Vec<Arc<Vec<f64>>>,
    m1_actuator_command_forces: Vec<Arc<Vec<f64>>>,
    #[cfg(not(m1_hp_force_extension))]
    m1_hardpoints_motion: Arc<Vec<Arc<Vec<f64>>>>,
    #[cfg(m1_hp_force_extension)]
    m1_hardpoints_forces: Arc<Vec<Arc<Vec<f64>>>>,
    idx: Vec<usize>,
    mode_2_force_transforms: Option<Vec<nalgebra::DMatrix<f64>>>,
}
impl DispatchIn {
    pub fn new() -> Self {
        let m1_actuator_command_forces: Vec<_> = <Self as Assembly>::SIDS
            .into_iter()
            .map(|i| Arc::new(vec![0f64; NA[i as usize - 1]]))
            .collect();
        let m1_rigid_body_motions: Vec<_> = <Self as Assembly>::SIDS
            .into_iter()
            .map(|_| Arc::new(vec![0f64; 6]))
            .collect();
        let mut idx = vec![0; 7];
        <Self as Assembly>::SIDS
            .iter()
            .enumerate()
            .for_each(|(i, &id)| {
                idx[id as usize - 1] = i;
            });
        Self {
            m1_rigid_body_motions,
            m1_actuator_command_forces,
            #[cfg(not(m1_hp_force_extension))]
            m1_hardpoints_motion: Default::default(),
            #[cfg(m1_hp_force_extension)]
            m1_hardpoints_forces: Default::default(),
            idx,
            mode_2_force_transforms: None,
        }
    }
    // pub fn add_ouput(
    //     &mut this: Actor<Self>,
    //     actor: &mut Sys<SegmentControl<1, R>>,
    // ) -> Result<(), ActorOutputsError> {
    //     this.add_output()
    //         .build::<RBM<1>>()
    //         .into_input::<Hardpoints<1>>(actor)
    // }
    pub fn modes_to_forces(mut self, transforms: Vec<nalgebra::DMatrix<f64>>) -> Self {
        self.mode_2_force_transforms = Some(transforms);
        self
    }
}

impl Assembly for DispatchIn {}

impl Update for DispatchIn {}

impl Read<M1State> for DispatchIn {
    fn read(&mut self, data: Data<M1State>) {
        let m2fts = self
            .mode_2_force_transforms
            .as_ref()
            .expect("missing modal to zonal forces matrices in systems::m1::DispatchIn");
        for (segment_state, (segment_rbms, (segment_m2ft, segment_forces))) in data.iter().zip(
            self.m1_rigid_body_motions.iter_mut().zip(
                m2fts
                    .into_iter()
                    .zip(self.m1_actuator_command_forces.iter_mut()),
            ),
        ) {
            if let Some(SegmentState { rbms, modes }) = segment_state {
                if let Some(rbms) = rbms {
                    *segment_rbms = rbms.clone();
                }
                if let Some(modes) = modes {
                    let b = nalgebra::DVector::from_column_slice(modes);
                    let y = segment_m2ft * b;
                    *segment_forces = y.as_slice().to_vec().into();
                }
            }
        }
    }
}

impl Read<M1RigidBodyMotions> for DispatchIn {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        let data = data.into_arc();
        self.m1_rigid_body_motions.iter_mut().fold(0, |i, out| {
            let (slice, _) = data[i..].split_at(6);
            *out = Arc::new(slice.to_vec());
            i + 6
        });
    }
}

impl<U> Read<Left<U>> for DispatchIn
where
    U: UniqueIdentifier,
    DispatchIn: Read<U>,
{
    fn read(&mut self, data: Data<Left<U>>) {
        <Self as Read<U>>::read(self, data.transmute());
    }
}
impl<U> Read<Right<U>> for DispatchIn
where
    U: UniqueIdentifier,
    DispatchIn: Read<U>,
{
    fn read(&mut self, data: Data<Right<U>>) {
        <Self as Read<U>>::read(self, data.transmute());
    }
}

impl<const ID: u8> Write<RBM<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<RBM<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.m1_rigid_body_motions
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}

impl Read<M1ActuatorCommandForces> for DispatchIn {
    fn read(&mut self, data: Data<M1ActuatorCommandForces>) {
        let data = data.into_arc();
        NA.iter()
            .zip(self.m1_actuator_command_forces.iter_mut())
            .fold(0, |i, (&s, out)| {
                let (slice, _) = data[i..].split_at(s);
                *out = Arc::new(slice.to_vec());
                i + s
            });
    }
}
impl Read<M1ModeShapes> for DispatchIn {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        let data = data.into_arc();
        let mut data_iter = data.iter().cloned();
        self.mode_2_force_transforms
            .as_ref()
            .expect("missing modal to zonal forces matrices in systems::m1::DispatchIn")
            .into_iter()
            .zip(&NA)
            .map(|(mat, na)| {
                let (nrows, ncols) = mat.shape();
                assert_eq!(
                    nrows, *na,
                    "modal to zonal forces matrices shape mismatch in systems::m1::DispatchIn"
                );
                let b = nalgebra::DVector::from_iterator(ncols, data_iter.by_ref().take(ncols));
                let y = mat * b;
                y.as_slice().to_vec()
            })
            .zip(self.m1_actuator_command_forces.iter_mut())
            .for_each(|(f, a)| *a = f.into());
    }
}
impl<const ID: u8> Read<ActuatorCommandForces<ID>> for DispatchIn {
    fn read(&mut self, data: Data<ActuatorCommandForces<ID>>) {
        self.m1_actuator_command_forces[ID as usize - 1] = data.into_arc();
    }
}
impl<const ID: u8> Write<ActuatorCommandForces<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<ActuatorCommandForces<ID>>> {
        Some(
            self.m1_actuator_command_forces[self.idx[ID as usize - 1]]
                .clone()
                .into(),
        )
    }
}
impl<const ID: u8> Read<ModeShapes<ID>> for DispatchIn {
    fn read(&mut self, data: Data<ModeShapes<ID>>) {
        let data = data.into_arc();
        let i = ID as usize - 1;
        let mat = &self
            .mode_2_force_transforms
            .as_ref()
            .expect("missing modal to zonal forces matrices in systems::m1::DispatchIn")[i];
        let b = nalgebra::DVector::from_column_slice(&data);
        let y = mat * b;
        self.m1_actuator_command_forces[i] = y.as_slice().to_vec().into();
    }
}

#[cfg(not(m1_hp_force_extension))]
impl Read<M1HardpointsMotion> for DispatchIn {
    fn read(&mut self, data: Data<M1HardpointsMotion>) {
        self.m1_hardpoints_motion = data.into_arc();
    }
}
#[cfg(not(m1_hp_force_extension))]
impl<const ID: u8> Write<HardpointsMotion<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<HardpointsMotion<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.m1_hardpoints_motion
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}
#[cfg(m1_hp_force_extension)]
impl Read<M1HardpointsForces> for DispatchIn {
    fn read(&mut self, data: Data<M1HardpointsForces>) {
        self.m1_hardpoints_forces = data.into_arc();
    }
}
#[cfg(m1_hp_force_extension)]
impl<const ID: u8> Write<HardpointsForces<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<HardpointsForces<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.m1_hardpoints_forces
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}
