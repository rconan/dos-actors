use gmt_fem::{FEM, IOData};
use interface::UniqueIdentifier;
use nalgebra as na;

use crate::fem_io;

/// Select/deselect FEM inputs/outputs
#[derive(Debug, Clone, Copy)]
pub enum Switch {
    On,
    Off,
}
pub trait Model {
    fn in_position<U>(&self) -> Option<usize>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>;
    fn keep_input<U>(&mut self) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>;
    fn keep_input_by<U, F>(&mut self, pred: F) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
        F: Fn(&IOData) -> bool + Copy;
    fn out_position<U>(&self) -> Option<usize>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>;
    fn keep_output<U>(&mut self) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>;
    fn keep_output_by<U, F>(&mut self, pred: F) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
        F: Fn(&IOData) -> bool + Copy;
    fn in2modes<U>(&self) -> Option<Vec<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>;
    fn trim2in<U>(&self, matrix: &na::DMatrix<f64>) -> Option<na::DMatrix<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>;
    fn modes2out<U>(&self) -> Option<Vec<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>;
    fn trim2out<U>(&self, matrix: &na::DMatrix<f64>) -> Option<na::DMatrix<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>;
    fn switch_inputs(&mut self, switch: Switch, id: Option<&[usize]>) -> &mut Self;
    fn switch_outputs(&mut self, switch: Switch, id: Option<&[usize]>) -> &mut Self;
    fn switch_input<U>(&mut self, switch: Switch) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>;
    fn switch_output<U>(&mut self, switch: Switch) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>;
    fn switch_inputs_by_name<S: Into<String>>(
        &mut self,
        names: Vec<S>,
        switch: Switch,
    ) -> gmt_fem::Result<&mut Self>;
    fn switch_outputs_by_name<S: Into<String>>(
        &mut self,
        names: Vec<S>,
        switch: Switch,
    ) -> gmt_fem::Result<&mut Self>;
}

impl Model for FEM {
    fn in_position<U>(&self) -> Option<usize>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
    {
        <Vec<Option<gmt_fem::fem_io::Inputs>> as fem_io::FemIo<U>>::position(&self.inputs)
    }
    fn keep_input<U>(&mut self) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
    {
        self.in_position::<U>().map(|i| self.keep_inputs(&[i]))
    }
    fn keep_input_by<U, F>(&mut self, pred: F) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
        F: Fn(&IOData) -> bool + Copy,
    {
        self.in_position::<U>()
            .map(|i| self.keep_inputs_by(&[i], pred))
    }
    fn out_position<U>(&self) -> Option<usize>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
    {
        <Vec<Option<gmt_fem::fem_io::Outputs>> as fem_io::FemIo<U>>::position(&self.outputs)
    }
    fn keep_output<U>(&mut self) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
    {
        self.out_position::<U>().map(|i| self.keep_outputs(&[i]))
    }
    fn keep_output_by<U, F>(&mut self, pred: F) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
        F: Fn(&IOData) -> bool + Copy,
    {
        self.out_position::<U>()
            .map(|i| self.keep_outputs_by(&[i], pred))
    }
    /// Returns the inputs 2 modes transformation matrix for an input type
    fn in2modes<U>(&self) -> Option<Vec<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
    {
        <Vec<Option<gmt_fem::fem_io::Inputs>> as fem_io::FemIo<U>>::position(&self.inputs)
            .and_then(|id| self.input2modes(id))
    }
    fn trim2in<U>(&self, matrix: &na::DMatrix<f64>) -> Option<na::DMatrix<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
    {
        <Vec<Option<gmt_fem::fem_io::Inputs>> as fem_io::FemIo<U>>::position(&self.inputs)
            .and_then(|id| self.trim2input(id, matrix))
    }
    /// Returns the modes 2 outputs transformation matrix for an output type
    fn modes2out<U>(&self) -> Option<Vec<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
    {
        <Vec<Option<gmt_fem::fem_io::Outputs>> as fem_io::FemIo<U>>::position(&self.outputs)
            .and_then(|id| self.modes2output(id))
    }
    fn trim2out<U>(&self, matrix: &na::DMatrix<f64>) -> Option<na::DMatrix<f64>>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
    {
        <Vec<Option<gmt_fem::fem_io::Outputs>> as fem_io::FemIo<U>>::position(&self.outputs)
            .and_then(|id| self.trim2output(id, matrix))
    }
    /// Inputs on/off switch
    ///
    /// Either flips all inputs if id is [None] or only the inputs specified with `id`
    fn switch_inputs(&mut self, switch: Switch, id: Option<&[usize]>) -> &mut Self {
        for i in id
            .map(|i| i.to_vec())
            .unwrap_or_else(|| (0..self.inputs.len()).collect::<Vec<usize>>())
        {
            self.inputs.get_mut(i).and_then(|input| {
                input.as_mut().map(|input| {
                    input.iter_mut().for_each(|io| {
                        *io = match switch {
                            Switch::On => io.clone().switch_on(),
                            Switch::Off => io.clone().switch_off(),
                        };
                    })
                })
            });
        }
        self
    }
    /// Outputs on/off switch
    ///
    /// Either flips all outputs if id is [None] or only the outputs specified with `id`
    fn switch_outputs(&mut self, switch: Switch, id: Option<&[usize]>) -> &mut Self {
        for i in id
            .map(|i| i.to_vec())
            .unwrap_or_else(|| (0..self.outputs.len()).collect::<Vec<usize>>())
        {
            self.outputs.get_mut(i).and_then(|output| {
                output.as_mut().map(|output| {
                    output.iter_mut().for_each(|io| {
                        *io = match switch {
                            Switch::On => io.clone().switch_on(),
                            Switch::Off => io.clone().switch_off(),
                        };
                    })
                })
            });
        }
        self
    }
    /// Input on/off switch
    ///
    /// Flips input of type `U`
    fn switch_input<U>(&mut self, switch: Switch) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Inputs>>: fem_io::FemIo<U>,
    {
        self.in_position::<U>()
            .map(|i| self.switch_inputs(switch, Some(&[i])))
    }
    /// Output on/off switch
    ///
    /// Flips output of type `U`
    fn switch_output<U>(&mut self, switch: Switch) -> Option<&mut Self>
    where
        U: UniqueIdentifier,
        Vec<Option<gmt_fem::fem_io::Outputs>>: fem_io::FemIo<U>,
    {
        self.out_position::<U>()
            .map(|i| self.switch_outputs(switch, Some(&[i])))
    }
    /// Inputs on/off switch
    ///
    /// Flips  inputs with the given names
    fn switch_inputs_by_name<S: Into<String>>(
        &mut self,
        names: Vec<S>,
        switch: Switch,
    ) -> gmt_fem::Result<&mut Self> {
        for name in names {
            Box::<dyn fem_io::GetIn>::try_from(name.into())
                .map(|x| x.position(&self.inputs))
                .map(|i| i.map(|i| self.switch_inputs(switch, Some(&[i]))))?;
        }
        Ok(self)
    }
    /// Outputs on/off switch
    ///
    /// Flips outputs with the given names
    fn switch_outputs_by_name<S: Into<String>>(
        &mut self,
        names: Vec<S>,
        switch: Switch,
    ) -> gmt_fem::Result<&mut Self> {
        for name in names {
            Box::<dyn fem_io::GetOut>::try_from(name.into())
                .map(|x| x.position(&self.outputs))
                .map(|i| i.map(|i| self.switch_outputs(switch, Some(&[i]))))?;
        }
        Ok(self)
    }
}
