//! # Actors aggregations
//!
//! Algebraic rules to add [Model] and [Actor] to create a new model

use std::ops::{Add, AddAssign};

use interface::TryUpdate;

use crate::{
    actor::Actor,
    framework::model::TaskError,
    model,
    model::{Model, Unknown},
};

/// Aggregation of models into a new model
impl Add for Model<Unknown> {
    type Output = Model<Unknown>;

    fn add(self, rhs: Self) -> Self::Output {
        match (self.actors, rhs.actors) {
            (None, None) => Model::new(vec![]),
            (None, Some(b)) => Model::new(b),
            (Some(a), None) => Model::new(a),
            (Some(mut a), Some(mut b)) => {
                a.append(&mut b);
                Model::new(a)
            }
        }
    }
}

/// Aggregation of a model and an actor into a new model
impl<C, const NI: usize, const NO: usize> Add<Actor<C, NI, NO>> for Model<Unknown>
where
    C: TryUpdate + 'static,
    TaskError: From<<C as TryUpdate>::Error>,
{
    type Output = Model<Unknown>;

    fn add(self, rhs: Actor<C, NI, NO>) -> Self::Output {
        self + model!(rhs)
    }
}

/// Aggregation of an actor and a model into a new model
impl<C, const NI: usize, const NO: usize> Add<Model<Unknown>> for Actor<C, NI, NO>
where
    C: TryUpdate + 'static,
    TaskError: From<<C as TryUpdate>::Error>,
{
    type Output = Model<Unknown>;

    fn add(self, rhs: Model<Unknown>) -> Self::Output {
        model!(self) + rhs
    }
}

/// Aggregation of actors into a model
impl<A, const A_NI: usize, const A_NO: usize, B, const B_NI: usize, const B_NO: usize>
    Add<Actor<B, B_NI, B_NO>> for Actor<A, A_NI, A_NO>
where
    A: TryUpdate + 'static,
    B: TryUpdate + 'static,
    TaskError: From<<A as TryUpdate>::Error>,
    TaskError: From<<B as TryUpdate>::Error>,
{
    type Output = Model<Unknown>;

    fn add(self, rhs: Actor<B, B_NI, B_NO>) -> Self::Output {
        model!(self) + model!(rhs)
    }
}
/* /// Aggregation of subsystems into a model
impl<Right, Left> Add<SubSystem<Right>> for SubSystem<Left>
where
    Right: Gateways + BuildSystem<Right> + GetField + 'static,
    Model<model::Unknown>: From<Right>,
    Left: Gateways + BuildSystem<Left> + GetField + 'static,
    Model<model::Unknown>: From<Left>,
{
    type Output = Model<Unknown>;

    fn add(self, rhs: SubSystem<Right>) -> Self::Output {
        model!(self, rhs)
    }
} */

impl<C, const NI: usize, const NO: usize> AddAssign<Actor<C, NI, NO>> for Model<Unknown>
where
    C: TryUpdate + 'static,
    TaskError: From<<C as TryUpdate>::Error>,
{
    fn add_assign(&mut self, rhs: Actor<C, NI, NO>) {
        self.actors.get_or_insert(vec![]).push(Box::new(rhs));
    }
}
