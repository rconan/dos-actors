use std::ops::{Deref, DerefMut};

use crate::{actor::PlainActor, framework::model::Check};

#[derive(Debug, Hash, Default, Clone)]
pub struct PlainModel(Vec<PlainActor>);
impl Deref for PlainModel {
    type Target = [PlainActor];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for PlainModel {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl From<Vec<PlainActor>> for PlainModel {
    fn from(value: Vec<PlainActor>) -> Self {
        Self(value)
    }
}

impl FromIterator<PlainActor> for PlainModel {
    fn from_iter<T: IntoIterator<Item = PlainActor>>(iter: T) -> Self {
        Self(iter.into_iter().collect())
    }
}

impl<'a> FromIterator<Box<&'a dyn Check>> for PlainModel {
    fn from_iter<T: IntoIterator<Item = Box<&'a dyn Check>>>(iter: T) -> Self {
        Self(iter.into_iter().map(|x| x._as_plain()).collect())
    }
}

impl IntoIterator for &PlainModel {
    type Item = PlainActor;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.to_vec().into_iter()
    }
}
