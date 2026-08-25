use gmt_dos_clients_fem::{DiscreteStateSpace, solvers::ExponentialMatrix};

use super::Include;

/**
FEM Inputs/Outputs builder

Enable some FEM inputs and/or outputs
 **/
#[derive(Debug, Default, Clone)]
pub struct FemIO {
    pub(crate) inputs: Option<Vec<String>>,
    pub(crate) outputs: Option<Vec<String>>,
}

impl FemIO {
    /// Creates a new [M1SegmentFigure] builder instance
    pub fn new() -> Self {
        Default::default()
    }
    pub fn inputs(mut self, values: &[&str]) -> Self {
        self.inputs = Some(values.into_iter().map(|&x| x.into()).collect());
        self
    }
    pub fn outputs(mut self, values: &[&str]) -> Self {
        self.outputs = Some(values.into_iter().map(|&x| x.into()).collect());
        self
    }
}

impl<'a> Include<'a, FemIO> for DiscreteStateSpace<'a, ExponentialMatrix> {
    fn including(
        self,
        fem_io: Option<&'a mut FemIO>,
    ) -> Result<Self, gmt_dos_clients_fem::StateSpaceError>
    where
        Self: 'a + Sized,
    {
        let Some(fem_io) = fem_io else {
            return Ok(self);
        };
        match (&fem_io.inputs,&fem_io.outputs){
            (None, None) => Ok(self),
            (None, Some(outputs)) => self.outs_by_name(outputs.to_vec()),
            (Some(inputs), None) => self.ins_by_name(inputs.to_vec()),
            (Some(inputs), Some(outputs)) => self.ins_by_name(inputs.to_vec())?.outs_by_name(outputs.to_vec()),
        }
    }
}
