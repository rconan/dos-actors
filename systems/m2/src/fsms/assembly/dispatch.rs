use std::sync::Arc;

use gmt_dos_clients_io::{
    Assembly,
    gmt_m2::fsm::{
        M2FSMFsmCommand, M2FSMPiezoForces, M2FSMPiezoNodes,
        segment::{FsmCommand, PiezoForces, PiezoNodes},
    },
};
use interface::{Data, Read, Update, Write};
use serde::{Deserialize, Serialize};

impl Assembly for DispatchIn {}
impl Assembly for DispatchOut {}

/// Inputs dispatch
///
/// Distributes the FSMS command and piezostack actuator motions to the segments
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchIn
where
    Self: Assembly,
{
    fsms_command: Arc<Vec<Arc<Vec<f64>>>>,
    fsms_pzt_motion: Arc<Vec<Arc<Vec<f64>>>>,
}
impl DispatchIn
where
    Self: Assembly,
{
    pub fn new() -> Self {
        Self {
            fsms_command: Arc::new(vec![Arc::new(vec![0f64; 3]); <Self as Assembly>::N]),
            fsms_pzt_motion: Arc::new(vec![Arc::new(vec![0f64; 6]); <Self as Assembly>::N]),
        }
    }
}

/// Outputs dispatch
///
/// Collects the FSMS piezostack actuator forces from the segments
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchOut
where
    Self: Assembly,
{
    fsms_pzt_forces: Vec<Arc<Vec<f64>>>,
}
impl DispatchOut
where
    Self: Assembly,
{
    pub fn new() -> Self {
        Self {
            fsms_pzt_forces: vec![Arc::new(vec![0f64; 6]); <Self as Assembly>::N],
        }
    }
}

impl Update for DispatchIn {}
impl Update for DispatchOut {}

impl Read<M2FSMPiezoNodes> for DispatchIn {
    fn read(&mut self, data: Data<M2FSMPiezoNodes>) {
        self.fsms_pzt_motion =
            Arc::new(data.chunks(6).map(|data| Arc::new(data.to_vec())).collect());
    }
}
impl<const ID: u8> Write<PiezoNodes<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<PiezoNodes<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.fsms_pzt_motion
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}
impl Read<M2FSMFsmCommand> for DispatchIn {
    fn read(&mut self, data: Data<M2FSMFsmCommand>) {
        self.fsms_command = Arc::new(data.chunks(3).map(|data| Arc::new(data.to_vec())).collect());
    }
}
impl<const ID: u8> Write<FsmCommand<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<FsmCommand<ID>>> {
        <Self as Assembly>::position::<ID>()
            .and_then(|idx| self.fsms_command.get(idx).map(|data| data.clone().into()))
    }
}
impl<const ID: u8> Read<PiezoForces<ID>> for DispatchOut {
    fn read(&mut self, data: Data<PiezoForces<ID>>) {
        if let Some(idx) = <Self as Assembly>::position::<ID>() {
            let forces = data.into_arc();
            self.fsms_pzt_forces[idx] = forces;
        }
    }
}
impl Write<M2FSMPiezoForces> for DispatchOut {
    fn write(&mut self) -> Option<Data<M2FSMPiezoForces>> {
        Some(Data::new(
            self.fsms_pzt_forces
                .iter()
                .flat_map(|data| data.as_slice().to_vec())
                .collect(),
        ))
    }
}
