use gmt_dos_actors::prelude::Actor;
use gmt_dos_clients::sampler::Sampler;
use gmt_dos_clients_io::gmt_m1::segment::ActuatorCommandForces;
use serde::{Deserialize, Serialize};

use crate::Calibration;
use gmt_dos_clients_m1_ctrl::{Actuators, Hardpoints, LoadCells};

#[derive(Clone, Serialize, Deserialize)]
pub struct SegmentControl<const S: u8, const R: usize> {
    pub hardpoints: Actor<Hardpoints>,
    pub loadcells: Actor<LoadCells, 1, R>,
    pub actuators: Actor<Actuators<S>, R, 1>,
    pub sampler: Actor<Sampler<Vec<f64>, ActuatorCommandForces<S>>, 1, R>,
}

impl<const S: u8, const R: usize> SegmentControl<S, R> {
    pub fn new(calibration: &Calibration) -> Self {
        assert!(
            S > 0 && S < 8,
            "expect segment # in the range [1,7], found {S}"
        );
        let idx = (S - 1) as usize;
        let Calibration {
            stiffness,
            rbm_2_hp,
            lc_2_cg,
        } = calibration;

        let hardpoints: Actor<_> = (
            Hardpoints::new(*stiffness, rbm_2_hp[idx]),
            format!("M1S{S}\nHardpoints"),
        )
            .into();

        let loadcells: Actor<_, 1, R> = (
            LoadCells::new(*stiffness, lc_2_cg[idx]),
            format!("M1S{S}\nLoadcells"),
        )
            .into();

        let actuators: Actor<_, R, 1> =
            (Actuators::<S>::new(), format!("M1S{S}\nActuators")).into();

        Self {
            hardpoints,
            loadcells,
            actuators,
            sampler: Sampler::default().into(),
        }
    }
}
