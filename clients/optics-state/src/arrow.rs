use std::marker::PhantomData;

use gmt_dos_clients_arrow::{Arrow, ArrowBuilder};
use gmt_dos_clients_io::{
    gmt_m1::{M1ModeShapes, M1RigidBodyMotions},
    gmt_m2::M2RigidBodyMotions,
};
use interface::{Data, Entry, Read, UniqueIdentifier, Update};

use crate::{M1State, M2State, OpticsState};

/// [OpticsStateArrow] builder
pub struct OpticalStateArrowBuilder<M1 = M1State, M2 = M2State>
where
    M1: UniqueIdentifier,
    M2: UniqueIdentifier,
{
    arrow: ArrowBuilder,
    m1: PhantomData<M1>,
    m2: PhantomData<M2>,
}

impl<M1: UniqueIdentifier, M2: UniqueIdentifier> OpticalStateArrowBuilder<M1, M2> {
    /// Sets the parquet file name
    pub fn file_name<S: Into<String>>(self, file_name: S) -> Self {
        Self {
            arrow: self.arrow.filename(file_name),
            ..self
        }
    }
}

pub struct OpticalStateArrow<M1 = M1State, M2 = M2State>
where
    M1: UniqueIdentifier,
    M2: UniqueIdentifier,
{
    arrow: Arrow,
    m1: PhantomData<M1>,
    m2: PhantomData<M2>,
}
impl<M1: UniqueIdentifier, M2: UniqueIdentifier> OpticalStateArrow<M1, M2> {
    /// Creates an instance of [OpticalStateArrowBuilder]
    pub fn builder() -> OpticalStateArrowBuilder<M1, M2> {
        OpticalStateArrowBuilder {
            arrow: Arrow::builder(10_000).filename("optical_state.parquet"),
            m1: PhantomData,
            m2: PhantomData,
        }
    }
}
impl OpticalStateArrowBuilder<M1State, ()> {
    /// Builds an instance of [OpticalStateArrow]
    pub fn build(self, n_bm: usize) -> OpticalStateArrow<M1State, ()> {
        let mut arrow = self.arrow.build();
        <_ as Entry<M1RigidBodyMotions>>::entry(&mut arrow, 42);
        <_ as Entry<M1ModeShapes>>::entry(&mut arrow, n_bm * 7);
        OpticalStateArrow {
            arrow,
            m1: PhantomData,
            m2: PhantomData,
        }
    }
}
impl OpticalStateArrowBuilder<M1State, M2RigidBodyMotions> {
    /// Builds an instance of [OpticalStateArrow]
    pub fn build(self, n_bm: usize) -> OpticalStateArrow<M1State, M2RigidBodyMotions> {
        let mut arrow = self.arrow.build();
        <_ as Entry<M1RigidBodyMotions>>::entry(&mut arrow, 42);
        <_ as Entry<M2RigidBodyMotions>>::entry(&mut arrow, 42);
        <_ as Entry<M1ModeShapes>>::entry(&mut arrow, n_bm * 7);
        OpticalStateArrow {
            arrow,
            m1: PhantomData,
            m2: PhantomData,
        }
    }
}

impl<M1: UniqueIdentifier, M2: UniqueIdentifier> Update for OpticalStateArrow<M1, M2> {}

impl Read<OpticsState> for OpticalStateArrow<M1State, M2RigidBodyMotions> {
    fn read(&mut self, data: Data<OpticsState>) {
        let m1_rbms: Vec<_> = data
            .m1
            .iter()
            .flat_map(|m1| m1.into_rbms())
            .flatten()
            .collect();
        let m2_rbms: Vec<_> = data
            .m2
            .iter()
            .flat_map(|m2| m2.into_rbms())
            .flatten()
            .collect();
        let modes: Vec<_> = data
            .m1
            .iter()
            .flat_map(|m1| {
                m1.modes_into_iter()
                    .filter_map(|modes| modes.as_ref().map(|modes| modes.as_ref().to_vec()))
            })
            .flatten()
            .collect();
        <_ as Read<M1RigidBodyMotions>>::read(&mut self.arrow, m1_rbms.into());
        <_ as Read<M2RigidBodyMotions>>::read(&mut self.arrow, m2_rbms.into());
        <_ as Read<M1ModeShapes>>::read(&mut self.arrow, modes.into());
    }
}

impl Read<OpticsState> for OpticalStateArrow<M1State, ()> {
    fn read(&mut self, data: Data<OpticsState>) {
        let m1_rbms: Vec<_> = data
            .m1
            .iter()
            .flat_map(|m1| m1.into_rbms())
            .flatten()
            .collect();
        let modes: Vec<_> = data
            .m1
            .iter()
            .flat_map(|m1| {
                m1.modes_into_iter()
                    .filter_map(|modes| modes.as_ref().map(|modes| modes.as_ref().to_vec()))
            })
            .flatten()
            .collect();
        <_ as Read<M1RigidBodyMotions>>::read(&mut self.arrow, m1_rbms.into());
        <_ as Read<M1ModeShapes>>::read(&mut self.arrow, modes.into());
    }
}
