use std::sync::Arc;

use gmt_dos_clients::operator;
use gmt_dos_clients_io::{
    Assembly,
    gmt_m1::{
        M1ModeShapes,
        assembly::{M1ActuatorCommandForces, M1HardpointsMotion, M1RigidBodyMotions},
        segment::{ActuatorCommandForces, HardpointsMotion, RBM},
    },
};
use interface::{Data, Read, UniqueIdentifier, Update, Write};
use serde::{Deserialize, Serialize};

use super::NA;

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchIn
where
    Self: Assembly,
{
    m1_rigid_body_motions: Vec<Arc<Vec<f64>>>,
    m1_actuator_command_forces: Vec<Arc<Vec<f64>>>,
    m1_hardpoints_motion: Arc<Vec<Arc<Vec<f64>>>>,
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
            m1_hardpoints_motion: Default::default(),
            idx,
            mode_2_force_transforms: None,
        }
    }
    pub fn modes_to_forces(mut self, transforms: Vec<nalgebra::DMatrix<f64>>) -> Self {
        self.mode_2_force_transforms = Some(transforms);
        self
    }
}
impl Assembly for DispatchIn {}
impl Update for DispatchIn {}

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

impl<U> Read<operator::Left<U>> for DispatchIn
where
    U: UniqueIdentifier,
    DispatchIn: Read<U>,
{
    fn read(&mut self, data: Data<operator::Left<U>>) {
        <Self as Read<U>>::read(self, data.transmute());
    }
}
impl<U> Read<operator::Right<U>> for DispatchIn
where
    U: UniqueIdentifier,
    DispatchIn: Read<U>,
{
    fn read(&mut self, data: Data<operator::Right<U>>) {
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

impl Read<M1HardpointsMotion> for DispatchIn {
    fn read(&mut self, data: Data<M1HardpointsMotion>) {
        self.m1_hardpoints_motion = data.into_arc();
    }
}
impl<const ID: u8> Write<HardpointsMotion<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<HardpointsMotion<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.m1_hardpoints_motion
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}
