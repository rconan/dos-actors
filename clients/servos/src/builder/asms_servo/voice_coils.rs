use gmt_dos_clients_fem::{DiscreteStateSpace, StateSpaceError, solvers::ExponentialMatrix};
use nalgebra::{DMatrix, DMatrixView};

use crate::builder::Include;

/**
ASMS voice coils builder

The builder is used to set the modes to voice coil displacements transformation matrices

```no_run
use gmt_dos_clients_servos::{asms_servo, AsmsServo, GmtServoMechanisms};
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80; // 100Hz

let frequency = 8000_f64; // Hz
let fem = FEM::from_env()?;

let gmt_servos =
    GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(frequency, fem)
        .asms_servo(
            AsmsServo::new().voice_coils(
                asms_servo::VoiceCoils::new(vec![])
            ),
        )
        .build()?;
# Ok::<(), Box<dyn std::error::Error>>(())
```
*/
#[derive(Debug, Default, Clone)]
pub struct VoiceCoils {
    ins_transforms: Vec<DMatrix<f64>>,
    outs_transforms: Vec<DMatrix<f64>>,
}

impl VoiceCoils {
    /// Creates a new `VoiceCoils` object from the modes to voice coil displacements transformation matrices
    pub fn new(ins_transforms: Vec<DMatrix<f64>>) -> Self {
        let outs_transforms: Vec<_> = ins_transforms.iter().map(|x| x.transpose()).collect();
        Self {
            ins_transforms,
            outs_transforms,
        }
    }
    /// Returns the number of modes of each segment
    pub fn n_modes(&self) -> Option<Vec<usize>> {
        if self.ins_transforms.is_empty() {
            None
        } else {
            Some(self.ins_transforms.iter().map(|x| x.ncols()).collect())
        }
    }
    pub(crate) fn ins_transforms_view<'a>(&'a self) -> Vec<DMatrixView<'a, f64>> {
        self.ins_transforms.iter().map(|x| x.as_view()).collect()
    }
    pub(crate) fn outs_transforms_view<'a>(&'a self) -> Vec<DMatrixView<'a, f64>> {
        self.outs_transforms.iter().map(|x| x.as_view()).collect()
    }
}

impl<'a> Include<'a, VoiceCoils> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(self, voice_coils: Option<&'a mut VoiceCoils>) -> Result<Self, StateSpaceError> {
        if let Some(voice_coils) = voice_coils {
            self.including_asms(
                Some(vec![1, 2, 3, 4, 5, 6, 7]),
                Some(voice_coils.ins_transforms_view()),
                Some(voice_coils.outs_transforms_view()),
            )
        } else {
            self.including_asms(Some(vec![1, 2, 3, 4, 5, 6, 7]), None, None)
        }
    }
}
