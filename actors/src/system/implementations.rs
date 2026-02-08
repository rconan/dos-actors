use crate::{
    actor::{Actor, PlainActor},
    framework::{
        model::{Check, CheckError, Task, TaskError},
        network::{ActorOutput, AddActorInput},
    },
    prelude::{AddActorOutput, GetName, Model, Unknown},
    ActorError,
};

use super::{
    interfaces::{System, SystemInput, SystemOutput},
    Sys,
};

#[async_trait::async_trait]
impl<T> Task for Sys<T>
where
    T: System,
    for<'a> &'a T: IntoIterator<Item = Box<&'a dyn Check>>,
    Box<T>: IntoIterator<Item = Box<dyn Task>>,
{
    async fn async_run(&mut self) -> std::result::Result<(), TaskError> {
        todo!()
    }

    async fn task(mut self: Box<Self>) -> std::result::Result<(), TaskError> {
        let name = self.name();
        let q = *self;
        let w = q.sys;
        let b = Box::new(w);
        Model::<Unknown>::from_iter(b)
            .name(name)
            .skip_check()
            .run()
            .await?;
        Ok(())
    }

    fn as_plain(&self) -> PlainActor {
        self.plain()
    }
}

impl<T> Check for Sys<T>
where
    T: System,
    for<'a> &'a T: IntoIterator<Item = Box<&'a dyn Check>>,
{
    fn check_inputs(&self) -> std::result::Result<(), CheckError> {
        self.into_iter()
            .map(|a| a.check_inputs())
            .collect::<Result<Vec<_>, _>>()
            .map(|_| ())
    }

    fn check_outputs(&self) -> std::result::Result<(), CheckError> {
        self.into_iter()
            .map(|a| a.check_outputs())
            .collect::<Result<Vec<_>, _>>()
            .map(|_| ())
    }

    fn n_inputs(&self) -> usize {
        self.into_iter()
            .map(|a: Box<&dyn Check>| a.n_inputs())
            .sum()
    }
    fn n_outputs(&self) -> usize {
        self.into_iter()
            .map(|a: Box<&dyn Check>| a.n_outputs())
            .sum()
    }

    fn inputs_hashes(&self) -> Vec<u64> {
        self.into_iter()
            .flat_map(|a: Box<&dyn Check>| a.inputs_hashes())
            .collect()
    }

    fn outputs_hashes(&self) -> Vec<u64> {
        self.into_iter()
            .flat_map(|a: Box<&dyn Check>| a.outputs_hashes())
            .collect()
    }

    fn _as_plain(&self) -> PlainActor {
        self.plain()
    }

    fn is_system(&self) -> bool {
        true
    }
}

impl<T: System> GetName for T {
    fn get_name(&self) -> String {
        self.name()
    }
}

impl<'a, T, CO, const NI: usize, const NO: usize> AddActorOutput<'a, CO, NI, NO> for Sys<T>
where
    T: System,
    Sys<T>: SystemOutput<CO, NI, NO>,
    CO: interface::TryUpdate + 'static,
{
    fn add_output(&'a mut self) -> ActorOutput<'a, Actor<CO, NI, NO>> {
        AddActorOutput::add_output(self.output())
    }
}

impl<U, T, CI, const NI: usize, const NO: usize> AddActorInput<U, CI, NI, NO> for Sys<T>
where
    T: System,
    Sys<T>: SystemInput<CI, NI, NO>,
    U: 'static + interface::UniqueIdentifier,
    CI: interface::TryRead<U> + 'static,
    ActorError: From<<CI as interface::TryRead<U>>::Error>,
{
    fn add_input(&mut self, rx: flume::Receiver<interface::Data<U>>, hash: u64) {
        AddActorInput::add_input(self.input(), rx, hash)
    }
}
