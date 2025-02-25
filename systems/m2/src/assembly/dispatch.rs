use std::sync::Arc;

use gmt_dos_clients_io::{
    gmt_m2::asm::{
        segment::{AsmCommand, FluidDampingForces, VoiceCoilsForces, VoiceCoilsMotion},
        M2ASMAsmCommand, M2ASMFluidDampingForces, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
    },
    Assembly,
};
use interface::{Data, Read, Update, Write};
use serde::{Deserialize, Serialize};

impl Assembly for DispatchIn {}
impl Assembly for DispatchOut {}

/// Inputs dispatch
///
/// Distributes the ASMS command and voice coils motions
/// to the segments
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchIn
where
    Self: Assembly,
{
    asms_command: Vec<Arc<Vec<f64>>>,
    asms_voice_coil_motion: Arc<Vec<Arc<Vec<f64>>>>,
    n: Vec<usize>,
    idx: Vec<usize>,
}

/// Outputs dispatch
///
/// Collects the ASMS voice coils forces and fluid damping forces
/// from the segments
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DispatchOut
where
    Self: Assembly,
{
    asms_voice_coil_forces: Vec<Arc<Vec<f64>>>,
    asms_fluid_damping_forces: Vec<Arc<Vec<f64>>>,
    #[allow(dead_code)]
    n: Vec<usize>,
}

impl DispatchIn {
    /// Creates a new instance of `DispatchIn` with the number of degrees of freedom for each segment
    pub fn new(n: Vec<usize>) -> Self {
        let (asms_command, asms_voice_coil_motion): (Vec<_>, Vec<_>) = n
            .clone()
            .into_iter()
            .map(|n| (Arc::new(vec![0f64; n]), Arc::new(vec![0f64; n])))
            .unzip();
        let mut idx = vec![0; 7];
        <Self as Assembly>::SIDS
            .iter()
            .enumerate()
            .for_each(|(i, &id)| {
                idx[id as usize - 1] = i;
            });
        Self {
            asms_command,
            asms_voice_coil_motion: Arc::new(asms_voice_coil_motion),
            n,
            idx,
        }
    }
}

impl DispatchOut {
    const NA: usize = 675;

    /// Creates a new instance of `DispatchOut` with the number of degrees of freedom for each segment
    pub fn new(n: Vec<usize>) -> Self {
        Self {
            n,
            asms_voice_coil_forces: vec![
                Arc::new(Vec::with_capacity(Self::NA));
                <Self as Assembly>::N
            ],
            asms_fluid_damping_forces: vec![
                Arc::new(Vec::with_capacity(Self::NA));
                <Self as Assembly>::N
            ],
        }
    }
}

impl Update for DispatchIn {}
impl Update for DispatchOut {}

impl Read<M2ASMVoiceCoilsMotion> for DispatchIn {
    fn read(&mut self, data: Data<M2ASMVoiceCoilsMotion>) {
        self.asms_voice_coil_motion = data.into_arc();
    }
}
impl<const ID: u8> Write<VoiceCoilsMotion<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<VoiceCoilsMotion<ID>>> {
        <Self as Assembly>::position::<ID>().and_then(|idx| {
            self.asms_voice_coil_motion
                .get(idx)
                .map(|data| data.clone().into())
        })
    }
}

impl Read<M2ASMAsmCommand> for DispatchIn {
    fn read(&mut self, data: Data<M2ASMAsmCommand>) {
        let data = data.into_arc();
        self.n
            .iter()
            .zip(self.asms_command.iter_mut())
            .fold(0, |i, (&s, out)| {
                let (slice, _) = data[i..].split_at(s);
                *out = Arc::new(slice.to_vec());
                i + s
            });
    }
}
impl<const ID: u8> Write<AsmCommand<ID>> for DispatchIn {
    fn write(&mut self) -> Option<Data<AsmCommand<ID>>> {
        // <Self as Assembly>::position::<ID>()
        //     .and_then(|idx| self.asms_command.get(idx).map(|data| data.clone().into()))
        Some(self.asms_command[self.idx[ID as usize - 1]].clone().into())
    }
}

impl<const ID: u8> Read<VoiceCoilsForces<ID>> for DispatchOut {
    fn read(&mut self, data: Data<VoiceCoilsForces<ID>>) {
        if let Some(idx) = <Self as Assembly>::position::<ID>() {
            let forces = data.into_arc();
            self.asms_voice_coil_forces[idx] = forces;
        }
    }
}
impl Write<M2ASMVoiceCoilsForces> for DispatchOut {
    fn write(&mut self) -> Option<Data<M2ASMVoiceCoilsForces>> {
        Some(Data::new(self.asms_voice_coil_forces.clone()))
    }
}

impl<const ID: u8> Read<FluidDampingForces<ID>> for DispatchOut {
    fn read(&mut self, data: Data<FluidDampingForces<ID>>) {
        if let Some(idx) = <Self as Assembly>::position::<ID>() {
            let forces = data.into_arc();
            self.asms_fluid_damping_forces[idx] = forces;
        }
    }
}
impl Write<M2ASMFluidDampingForces> for DispatchOut {
    fn write(&mut self) -> Option<Data<M2ASMFluidDampingForces>> {
        Some(Data::new(self.asms_fluid_damping_forces.clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dispatch_in() {
        let mut din = DispatchIn::new(vec![2, 3, 1]);
        let data = Data::<M2ASMAsmCommand>::new(vec![1., 2., 3., 4., 5., 6.]);
        <DispatchIn as Read<M2ASMAsmCommand>>::read(&mut din, data);
        dbg!(&din);
        dbg!(<DispatchIn as Write<AsmCommand<1>>>::write(&mut din));
        dbg!(<DispatchIn as Write<AsmCommand<2>>>::write(&mut din));
        dbg!(<DispatchIn as Write<AsmCommand<3>>>::write(&mut din));
    }
}
