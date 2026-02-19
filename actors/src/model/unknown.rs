use crate::framework::model::Task;

use super::{Actors, Model, ModelError, Ready, Result, Unknown};
use std::{marker::PhantomData, time::Instant};

impl Default for Model<Unknown> {
    fn default() -> Self {
        Self {
            name: Default::default(),
            actors: Default::default(),
            task_handles: Default::default(),
            state: Default::default(),
            start: Instant::now(),
            verbose: true,
            elapsed_time: Default::default(),
        }
    }
}

impl FromIterator<Box<dyn Task>> for Model<Unknown> {
    fn from_iter<T: IntoIterator<Item = Box<dyn Task>>>(iter: T) -> Self {
        Self {
            actors: Some(iter.into_iter().collect()),
            ..Default::default()
        }
    }
}

impl Model<Unknown> {
    /// Returns a new model
    pub fn new(actors: Actors) -> Self {
        Self {
            name: None,
            actors: Some(actors),
            task_handles: None,
            state: PhantomData,
            start: Instant::now(),
            verbose: true,
            elapsed_time: Default::default(),
        }
    }
    /// Sets the model name
    pub fn name<S: Into<String>>(self, name: S) -> Self {
        Self {
            name: Some(name.into()),
            ..self
        }
    }
    /// Quiet mode
    pub fn quiet(mut self) -> Self {
        self.verbose = false;
        self
    }
    pub fn verbose(mut self,verbose: bool) -> Self {
        self.verbose = verbose;
        self
    }
    /// Validates actors inputs and outputs
    pub fn check(self) -> Result<Model<Ready>> {
        let (n_inputs, n_outputs) = self.n_io();
        let name = self.name.clone().unwrap_or_default();
        assert_eq!(
            n_inputs, n_outputs,
            "{} I/O #({},{}) don't match, did you forget to add some actors to the model:\n{}",
            name, n_inputs, n_outputs, self
        );
        match self.actors {
            Some(ref actors) => {
                let mut inputs_hashes = vec![];
                let mut outputs_hashes = vec![];
                for actor in actors {
                    actor.check_inputs().map_err(Box::new)?;
                    actor.check_outputs().map_err(Box::new)?;
                    inputs_hashes.append(&mut actor.inputs_hashes());
                    outputs_hashes.append(&mut actor.outputs_hashes());
                }
                let hashes_diff = outputs_hashes
                    .into_iter()
                    .zip(inputs_hashes)
                    .map(|(o, i)| o as i128 - i as i128)
                    .sum::<i128>();
                assert_eq!(hashes_diff,0i128,
                "{} I/O hashes difference: expected 0, found {}, did you forget to add some actors to the model?",
                self.name.unwrap_or_default(),
                hashes_diff);
                Ok(Model::<Ready> {
                    name: self.name,
                    actors: self.actors,
                    task_handles: None,
                    state: PhantomData,
                    start: Instant::now(),
                    verbose: self.verbose,
                    elapsed_time: Default::default(),
                })
            }
            None => Err(ModelError::NoActors),
        }
    }
    pub fn skip_check(self) -> Model<Ready> {
        Model::<Ready> {
            name: self.name,
            actors: self.actors,
            task_handles: None,
            state: PhantomData,
            start: Instant::now(),
            verbose: self.verbose,
            elapsed_time: Default::default(),
        }
    }
}
