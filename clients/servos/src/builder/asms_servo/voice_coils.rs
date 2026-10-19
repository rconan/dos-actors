use gmt_dos_clients_fem::{DiscreteStateSpace, StateSpaceError, solvers::ExponentialMatrix};
use gmt_dos_clients_io::{Assembly, gmt_m2::asm::M2ASMVoiceCoilsForces};
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

/// Returns the matrices of the `ASSEMBLY` segments
///
/// `matrices` holds either one matrix per segment of the `ASSEMBLY`, in the same order,
/// or one matrix for each of the 7 segments
fn assembly<T>(matrices: &[T]) -> Vec<&T> {
    if matrices.len() == 7 {
        <M2ASMVoiceCoilsForces as Assembly>::SIDS
            .into_iter()
            .map(|sid| &matrices[sid as usize - 1])
            .collect()
    } else {
        assert_eq!(
            matrices.len(),
            <M2ASMVoiceCoilsForces as Assembly>::N,
            "expected 7 ASMS voice coils transforms or one per ASSEMBLY segment"
        );
        matrices.iter().collect()
    }
}

/// Adds the ASMS voice coils of the `ASSEMBLY` segments to the state space model
pub(crate) fn including_asms<'a>(
    state_space: DiscreteStateSpace<'a, ExponentialMatrix>,
    voice_coils: Option<&'a VoiceCoils>,
) -> Result<DiscreteStateSpace<'a, ExponentialMatrix>, StateSpaceError> {
    let sids = <M2ASMVoiceCoilsForces as Assembly>::SIDS.to_vec();
    if let Some(voice_coils) = voice_coils {
        state_space.including_asms(
            Some(sids),
            Some(voice_coils.ins_transforms_view()),
            Some(voice_coils.outs_transforms_view()),
        )
    } else {
        state_space.including_asms(Some(sids), None, None)
    }
}

impl VoiceCoils {
    /// Creates a new `VoiceCoils` object from the modes to voice coil displacements transformation matrices
    ///
    /// Either one matrix per segment of the `ASSEMBLY`, in the same order,
    /// or one matrix for each of the 7 segments
    /// (only the matrices of the `ASSEMBLY` segments are used).
    pub fn new(ins_transforms: Vec<DMatrix<f64>>) -> Self {
        let outs_transforms: Vec<_> = ins_transforms.iter().map(|x| x.transpose()).collect();
        Self {
            ins_transforms,
            outs_transforms,
        }
    }
    /// Returns the number of modes of each segment of the `ASSEMBLY`
    pub fn n_modes(&self) -> Option<Vec<usize>> {
        if self.ins_transforms.is_empty() {
            None
        } else {
            Some(
                assembly(&self.ins_transforms)
                    .into_iter()
                    .map(|x| x.ncols())
                    .collect(),
            )
        }
    }
    pub(crate) fn ins_transforms_view<'a>(&'a self) -> Vec<DMatrixView<'a, f64>> {
        assembly(&self.ins_transforms)
            .into_iter()
            .map(|x| x.as_view())
            .collect()
    }
    pub(crate) fn outs_transforms_view<'a>(&'a self) -> Vec<DMatrixView<'a, f64>> {
        assembly(&self.outs_transforms)
            .into_iter()
            .map(|x| x.as_view())
            .collect()
    }
}

impl<'a> Include<'a, VoiceCoils> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(self, voice_coils: Option<&'a mut VoiceCoils>) -> Result<Self, StateSpaceError> {
        including_asms(self, voice_coils.map(|voice_coils| &*voice_coils))
    }
}
