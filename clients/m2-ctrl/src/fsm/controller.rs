use std::{ptr, sync::Arc};

use gmt_dos_clients_io::gmt_m2::fsm::segment::{FsmCommand, PiezoForces, PiezoNodes};
use gmt_m2_ctrl_fsm_piezo_7::FsmPiezo7;
use gmt_m2_ctrl_fsm_piezo_135::FsmPiezo135;
use gmt_m2_ctrl_fsm_piezo_246::FsmPiezo246;
use interface::{Data, Read, Size, Update, Write};
use serde::{Deserialize, Serialize};

/// Piezostack actuator controller
///
/// The controller tuning depends on the segments:
///  * segment #1,3,5: [FsmPiezo135](https://docs.rs/gmt_m2-ctrl_fsm-piezo-135/latest/gmt_m2_ctrl_fsm_piezo_135/type.FsmPiezo135.html)
///  * segment #2,4,6: [FsmPiezo246](https://docs.rs/gmt_m2-ctrl_fsm-piezo-246/latest/gmt_m2_ctrl_fsm_piezo_246/type.FsmPiezo246.html)
///  * segment #7    : [FsmPiezo7](https://docs.rs/gmt_m2-ctrl_fsm-piezo-7/latest/gmt_m2_ctrl_fsm_piezo_7/type.FsmPiezo7.html)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PiezoStackController {
    OuterOdd(FsmPiezo135),
    OuterEven(FsmPiezo246),
    Center(FsmPiezo7),
}
impl PiezoStackController {
    /// Creates a new piezostack actuator controller
    pub fn new(sid: u8) -> Self {
        assert!(0 < sid && sid < 8, "sid must in the range [1,7]");
        match sid {
            7 => PiezoStackController::Center(FsmPiezo7::new()),
            i if i % 2 == 0 => PiezoStackController::OuterEven(FsmPiezo246::new()),
            _ => PiezoStackController::OuterOdd(FsmPiezo135::new()),
        }
    }
    /// Sets the controller inputs
    pub fn u(&mut self, value: &[f64]) {
        unsafe {
            match self {
                PiezoStackController::OuterOdd(pzt_cfb) => ptr::copy_nonoverlapping(
                    value.as_ptr(),
                    pzt_cfb.inputs.pzt_error.as_mut_ptr(),
                    3,
                ),
                PiezoStackController::OuterEven(pzt_cfb) => ptr::copy_nonoverlapping(
                    value.as_ptr(),
                    pzt_cfb.inputs.pzt_error.as_mut_ptr(),
                    3,
                ),
                PiezoStackController::Center(pzt_cfb) => ptr::copy_nonoverlapping(
                    value.as_ptr(),
                    pzt_cfb.inputs.pzt_error.as_mut_ptr(),
                    3,
                ),
            };
        }
    }
    /// Steps the controller
    pub fn step(&mut self) {
        match self {
            PiezoStackController::OuterOdd(pzt_cfb) => pzt_cfb.step(),
            PiezoStackController::OuterEven(pzt_cfb) => pzt_cfb.step(),
            PiezoStackController::Center(pzt_cfb) => pzt_cfb.step(),
        };
    }
    /// Gets the controller outputs
    pub fn y(&mut self, value: &mut [f64]) {
        unsafe {
            match self {
                PiezoStackController::OuterOdd(pzt_cfb) => ptr::copy_nonoverlapping(
                    pzt_cfb.outputs.pzt_control.as_ptr(),
                    value.as_mut_ptr(),
                    3,
                ),
                PiezoStackController::OuterEven(pzt_cfb) => ptr::copy_nonoverlapping(
                    pzt_cfb.outputs.pzt_control.as_ptr(),
                    value.as_mut_ptr(),
                    3,
                ),
                PiezoStackController::Center(pzt_cfb) => ptr::copy_nonoverlapping(
                    pzt_cfb.outputs.pzt_control.as_ptr(),
                    value.as_mut_ptr(),
                    3,
                ),
            };
        }
    }
}

/// FSM segment [piezostack ](PiezoStackController) actuators controller
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FsmSegmentInnerController<const ID: u8> {
    piezo: PiezoStackController,
    forces: Vec<f64>,
    nodes: Vec<f64>,
    command: Arc<Vec<f64>>,
}

impl<const ID: u8> FsmSegmentInnerController<ID> {
    /// Creates a new FSM segment controller
    pub fn new() -> Self {
        Self {
            piezo: PiezoStackController::new(ID),
            forces: vec![0f64; 3],
            nodes: vec![0f64; 3],
            command: Arc::new(vec![0f64; 3]),
        }
    }
}

impl<const ID: u8> Update for FsmSegmentInnerController<ID> {
    fn update(&mut self) {
        let deltas: Vec<_> = self
            .command
            .iter()
            .zip(&self.nodes)
            .map(|(c, n)| c - n)
            .collect();
        self.piezo.u(&deltas);
        self.piezo.step();
        self.piezo.y(&mut self.forces);
    }
}

impl<const ID: u8> Size<PiezoNodes<ID>> for FsmSegmentInnerController<ID> {
    fn len(&self) -> usize {
        6
    }
}
impl<const ID: u8> Read<PiezoNodes<ID>> for FsmSegmentInnerController<ID> {
    fn read(&mut self, data: Data<PiezoNodes<ID>>) {
        self.nodes = data.chunks(2).map(|x| x[1] - x[0]).collect();
    }
}
impl<const ID: u8> Size<FsmCommand<ID>> for FsmSegmentInnerController<ID> {
    fn len(&self) -> usize {
        3
    }
}
impl<const ID: u8> Read<FsmCommand<ID>> for FsmSegmentInnerController<ID> {
    fn read(&mut self, data: Data<FsmCommand<ID>>) {
        self.command = data.into_arc();
    }
}

impl<const ID: u8> Size<PiezoForces<ID>> for FsmSegmentInnerController<ID> {
    fn len(&self) -> usize {
        6
    }
}
impl<const ID: u8> Write<PiezoForces<ID>> for FsmSegmentInnerController<ID> {
    fn write(&mut self) -> Option<Data<PiezoForces<ID>>> {
        Some(
            self.forces
                .iter()
                .flat_map(|&x| vec![-x, x])
                .collect::<Vec<_>>()
                .into(),
        )
    }
}
