use interface::{TryUpdate, Who};

use crate::{
    framework::model::{Check, CheckError},
    ActorError,
};

use super::{Actor, PlainActor};

type Result<T> = std::result::Result<T, CheckError>;

impl<C, const NI: usize, const NO: usize> Check for Actor<C, NI, NO>
where
    C: 'static + TryUpdate,
{
    fn check_inputs(&self) -> Result<()> {
        match self.inputs {
            Some(_) if NI == 0 => Err(ActorError::SomeInputsZeroRate(Who::who(self))),
            None if NI > 0 => Err(ActorError::NoInputsPositiveRate(Who::who(self))),
            _ => Ok(()),
        }?;
        Ok(())
    }
    fn check_outputs(&self) -> Result<()> {
        match self.outputs {
            Some(_) if NO == 0 => Err(ActorError::SomeOutputsZeroRate(Who::who(self))),
            None if NO > 0 => Err(ActorError::NoOutputsPositiveRate(Who::who(self))),
            _ => Ok(()),
        }?;
        Ok(())
    }
    fn n_inputs(&self) -> usize {
        self.inputs.as_ref().map_or(0, |i| i.len())
    }
    fn n_outputs(&self) -> usize {
        self.outputs
            .as_ref()
            .map_or(0, |o| o.iter().map(|o| o.len()).sum())
    }
    fn inputs_hashes(&self) -> Vec<u64> {
        self.inputs.as_ref().map_or(Vec::new(), |inputs| {
            inputs.iter().map(|input| input.get_hash()).collect()
        })
    }
    fn outputs_hashes(&self) -> Vec<u64> {
        self.outputs.as_ref().map_or(Vec::new(), |outputs| {
            outputs
                .iter()
                .flat_map(|output| vec![output.get_hash(); output.len()])
                .collect()
        })
    }
    fn _as_plain(&self) -> PlainActor {
        self.into()
    }
}
