//! # FEM inputs/outputs definitions

use std::{
    any::{type_name, Any},
    fmt,
    fmt::Debug,
    marker::PhantomData,
    ops::Range,
};

use gmt_fem::fem_io::{Inputs, Outputs};
use gmt_fem::FEM;
use interface::UniqueIdentifier;
use nalgebra::DMatrix;

/// Find the index corresponding to `U` in the [FEM] [Inputs] and [Outputs] vectors
///
/// `U` is either an [actors_inputs] or [actors_outputs].
pub trait FemIo<U: UniqueIdentifier> {
    /// Returns the index position
    fn position(&self) -> Option<usize>;
}

//fem_macros::ad_hoc! {}
mod inputs {
    use super::{FemIo, GetIn, SplitFem};
    use gmt_fem::{fem_io::Inputs, FemError};
    // pub mod actors_inputs {
    //     include!(concat!(env!("OUT_DIR"), "/fem_actors_inputs.rs"));
    // }
    use gmt_dos_clients_io::gmt_fem::inputs::*;
    include!(concat!(env!("OUT_DIR"), "/fem_get_in.rs"));
    include!(concat!(env!("OUT_DIR"), "/fem_inputs.rs"));
}
mod outputs {
    use super::{FemIo, GetOut, SplitFem};
    use gmt_fem::{fem_io::Outputs, FemError};
    // pub mod actors_outputs {
    //     include!(concat!(env!("OUT_DIR"), "/fem_actors_outputs.rs"));
    // }
    use gmt_dos_clients_io::gmt_fem::outputs::*;
    include!(concat!(env!("OUT_DIR"), "/fem_get_out.rs"));
    include!(concat!(env!("OUT_DIR"), "/fem_outputs.rs"));
}
pub use gmt_dos_clients_io::gmt_fem::inputs as actors_inputs;
pub use gmt_dos_clients_io::gmt_fem::outputs as actors_outputs;
// pub use outputs::actors_outputs;

use crate::model::Model;

/// Hold the range of indices corresponding to `U` in the [FEM] [Inputs] and [Outputs] vectors
///
/// `U` is either an [actors_inputs] or [actors_outputs].
//#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone)]
pub struct SplitFem<U: UniqueIdentifier> {
    range: Range<usize>,
    io: PhantomData<U>,
}
#[cfg(feature = "serde")]
impl<U: UniqueIdentifier> serde::Serialize for SplitFem<U> {
    fn serialize<S>(&self, s: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        SplitFemErased::from(self).serialize(s)
    }
}
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct SplitFemErased {
    kind: String,
    range: Range<usize>,
}
#[cfg(feature = "serde")]
impl<U: UniqueIdentifier> From<&SplitFem<U>> for SplitFemErased {
    fn from(value: &SplitFem<U>) -> Self {
        Self {
            kind: type_name::<U>().split("::").last().unwrap().into(),
            range: value.range.clone(),
        }
    }
}
impl<U: UniqueIdentifier> From<&SplitFem<U>> for SplitFem<U> {
    fn from(value: &SplitFem<U>) -> Self {
        SplitFem {
            range: value.range.clone(),
            io: PhantomData,
        }
    }
}

impl<U: UniqueIdentifier> SplitFem<U> {
    /// Creates a new [SplitFem] object
    pub fn new() -> Self {
        Self {
            range: Range::default(),
            io: PhantomData,
        }
    }
    pub fn ranged(range: Range<usize>) -> Self {
        Self {
            range,
            io: PhantomData,
        }
    }
    /// Returns the actors type
    pub fn fem_type(&self) -> String {
        type_name::<U>().to_string()
    }
    /// Returns the range
    pub fn range(&self) -> &Range<usize> {
        &self.range
    }
}
impl<U: UniqueIdentifier> Debug for SplitFem<U> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct(&format!("SplitFem<{}>", self.fem_type()))
            .field("range", &self.range)
            .finish()
    }
}
impl<U: UniqueIdentifier> Default for SplitFem<U> {
    fn default() -> Self {
        Self::new()
    }
}

/// Range setting for [SplitFem]
pub trait SetRange {
    /// Sets the range
    fn set_range(&mut self, start: usize, end: usize);
}
impl<U: UniqueIdentifier> SetRange for SplitFem<U> {
    fn set_range(&mut self, start: usize, end: usize) {
        self.range = Range { start, end };
    }
}
/// Interface between the FEM [Inputs] and the [DOS actors inputs](actors_inputs)
pub trait GetIn: SetRange + Debug + Send + Sync {
    fn as_any(&self) -> &dyn Any;
    /// Returns the inputs to modes matrix for a given input
    fn get_in(&self, fem: &FEM) -> Option<DMatrix<f64>>;
    /// Trims the inputs to modes matrix to the given input
    fn trim_in(&self, fem: &FEM, matrix: &DMatrix<f64>) -> Option<DMatrix<f64>>;
    /// Returns the actors type
    fn fem_type(&self) -> String;
    /// Gets the input range of indices
    fn range(&self) -> Range<usize>;
    /// Gets the input length
    fn len(&self) -> usize;
    /// Returns the input position in the FEM [Inputs] vector
    fn position(&self, fem: &Vec<Option<Inputs>>) -> Option<usize>;
}
impl<U: 'static + UniqueIdentifier + Send + Sync> GetIn for SplitFem<U>
where
    Vec<Option<Inputs>>: FemIo<U>,
{
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn get_in(&self, fem: &FEM) -> Option<DMatrix<f64>> {
        fem.in2modes::<U>()
            .as_ref()
            .map(|x| DMatrix::from_row_slice(fem.n_modes(), x.len() / fem.n_modes(), x))
    }
    fn trim_in(&self, fem: &FEM, matrix: &DMatrix<f64>) -> Option<DMatrix<f64>> {
        fem.trim2in::<U>(matrix)
    }
    fn fem_type(&self) -> String {
        self.fem_type()
    }
    fn range(&self) -> Range<usize> {
        self.range.clone()
    }
    fn len(&self) -> usize {
        self.range.end - self.range.start
    }
    fn position(&self, inputs: &Vec<Option<Inputs>>) -> Option<usize> {
        <Vec<Option<Inputs>> as FemIo<U>>::position(inputs)
    }
}
impl<U: 'static + UniqueIdentifier + Send + Sync> SplitFem<U> {
    pub fn get_in(object: &Box<dyn GetIn>) -> Option<Self> {
        object.as_any().downcast_ref::<Self>().map(|x| x.into())
    }
    pub fn get_out(object: &Box<dyn GetOut>) -> Option<Self> {
        object.as_any().downcast_ref::<Self>().map(|x| x.into())
    }
}

/// Interface between the FEM [Outputs] and the [DOS actors outputs](actors_outputs)
pub trait GetOut: SetRange + Debug + Send + Sync {
    fn as_any(&self) -> &dyn Any;
    /// Returns the outputs to modes matrix for a given output
    fn get_out(&self, fem: &FEM) -> Option<DMatrix<f64>>;
    /// Trims the outputs to modes matrix to the given output
    fn trim_out(&self, fem: &FEM, matrix: &DMatrix<f64>) -> Option<DMatrix<f64>>;
    /// Returns the actors type
    fn fem_type(&self) -> String;
    /// Sets the output range of indices
    fn range(&self) -> Range<usize>;
    /// Returns the output position in the FEM [Outputs] vector
    fn position(&self, outputs: &Vec<Option<Outputs>>) -> Option<usize>;
}
impl<U: 'static + UniqueIdentifier + Send + Sync> GetOut for SplitFem<U>
where
    Vec<Option<Outputs>>: FemIo<U>,
{
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn get_out(&self, fem: &FEM) -> Option<DMatrix<f64>> {
        fem.modes2out::<U>()
            .as_ref()
            .map(|x| DMatrix::from_row_slice(x.len() / fem.n_modes(), fem.n_modes(), x))
    }
    fn trim_out(&self, fem: &FEM, matrix: &DMatrix<f64>) -> Option<DMatrix<f64>> {
        fem.trim2out::<U>(matrix)
    }
    fn fem_type(&self) -> String {
        self.fem_type()
    }

    fn range(&self) -> Range<usize> {
        self.range.clone()
    }

    fn position(&self, outputs: &Vec<Option<Outputs>>) -> Option<usize> {
        <Vec<Option<Outputs>> as FemIo<U>>::position(outputs)
    }
}
