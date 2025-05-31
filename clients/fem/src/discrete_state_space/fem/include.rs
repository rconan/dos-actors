use nalgebra::DMatrixView;

use crate::{DiscreteStateSpace, StateSpaceError, solvers::Solver};

type Result<T> = std::result::Result<T, StateSpaceError>;

impl<'a, T: Solver + Default> DiscreteStateSpace<'a, T> {
    #[cfg(mount)]
    pub fn including_mount(self) -> Self {
        use crate::fem_io;

        self.ins::<fem_io::actors_inputs::OSSElDriveTorque>()
            .ins::<fem_io::actors_inputs::OSSAzDriveTorque>()
            .ins::<fem_io::actors_inputs::OSSRotDriveTorque>()
            .outs::<fem_io::actors_outputs::OSSElEncoderAngle>()
            .outs::<fem_io::actors_outputs::OSSAzEncoderAngle>()
            .outs::<fem_io::actors_outputs::OSSRotEncoderAngle>()
    }
    pub fn including_m1(self, sids: Option<Vec<u8>>) -> Result<Self> {
        let mut names: Vec<_> = if let Some(sids) = sids {
            sids.into_iter()
                .map(|i| {
                    assert!(i > 0 && i < 8, "expected 1≤sid≤7,found sid={}", i);
                    format!("M1_actuators_segment_{i}")
                })
                .collect()
        } else {
            (1..=7)
                .map(|i| format!("M1_actuators_segment_{i}"))
                .collect()
        };
        names.push("OSS_Harpoint_delta_F".to_string());
        self.ins_by_name(names)
            .and_then(|this| this.outs_by_name(vec!["OSS_Hardpoint_D"]))
    }
    pub fn including_asms(
        self,
        sids: Option<Vec<u8>>,
        ins_transforms: Option<Vec<DMatrixView<'a, f64>>>,
        outs_transforms: Option<Vec<DMatrixView<'a, f64>>>,
    ) -> Result<Self> {
        let mut ins1_names = vec![];
        let mut ins2_names = vec![];
        let mut outs_names = vec![];
        for i in sids.unwrap_or_else(|| (1..=7).collect()) {
            assert!(i > 0 && i < 8, "expected 1≤sid≤7,found sid={}", i);
            ins1_names.push(format!("MC_M2_S{i}_VC_delta_F"));
            ins2_names.push(format!("MC_M2_S{i}_fluid_damping_F"));
            outs_names.push(format!("MC_M2_S{i}_VC_delta_D"))
        }
        match (ins_transforms, outs_transforms) {
            (None, None) => self
                .ins_by_name(ins1_names)
                .and_then(|this| this.ins_by_name(ins2_names))
                .and_then(|this| this.outs_by_name(outs_names)),
            (None, Some(outs_transforms)) => self
                .ins_by_name(ins1_names)
                .and_then(|this| this.ins_by_name(ins2_names))
                .and_then(|this| this.outs_with_by_name(outs_names, outs_transforms)),
            (Some(ins_transforms), None) => self
                .ins_with_by_name(ins1_names, ins_transforms.clone())
                .and_then(|this| this.ins_with_by_name(ins2_names, ins_transforms))
                .and_then(|this| this.outs_by_name(outs_names)),
            (Some(ins_transforms), Some(outs_transforms)) => self
                .ins_with_by_name(ins1_names, ins_transforms.clone())
                .and_then(|this| this.ins_with_by_name(ins2_names, ins_transforms))
                .and_then(|this| this.outs_with_by_name(outs_names, outs_transforms)),
        }
    }
}
