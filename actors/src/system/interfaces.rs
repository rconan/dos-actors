use std::fmt::Display;

use crate::{
    actor::{Actor, PlainActor},
    prelude::GetName,
};

use super::SystemError;

/// System interface
pub trait System: Sized + Clone + Display + Send + Sync + GetName {
    fn name(&self) -> String {
        String::from("SYSTEM")
    }
    fn build(&mut self) -> Result<&mut Self, SystemError> {
        Ok(self)
    }
    fn plain(&self) -> PlainActor;
}

/// System inputs interface
pub trait SystemInput<C, const NI: usize, const NO: usize>
where
    C: interface::Update,
{
    fn input(&mut self) -> &mut Actor<C, NI, NO>;
}

/// System outputs interface
pub trait SystemOutput<C, const NI: usize, const NO: usize>
where
    C: interface::Update,
{
    fn output(&mut self) -> &mut Actor<C, NI, NO>;
}
