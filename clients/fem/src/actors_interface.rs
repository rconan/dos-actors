/*!
# GMT Finite Element Model client

The module implements the client interface for the [GMT FEM Rust API](https://docs.rs/gmt-fem)

*/

#[doc(hidden)]
pub mod prelude {
    pub use crate::{solvers::Solver, DiscreteModalSolver, Get, Set};
    pub use interface::{Data, Read, Size, UniqueIdentifier, Update, Write};
    pub mod fem_io {
        pub use gmt_dos_clients_io::gmt_fem::inputs::*;
        pub use gmt_dos_clients_io::gmt_fem::outputs::*;
    }
    pub use std::sync::Arc;
}

use gmt_dos_clients::operator;
use interface::Units;
use prelude::*;

#[cfg(all(fem, any(cfd2021, cfd2025)))]
mod cfd;
#[cfg(all(fem, m1))]
mod m1;
#[cfg(all(fem, m2))]
mod m2;
#[cfg(all(fem, mount))]
mod mount;
mod rbm_removal;
pub use rbm_removal::RbmRemoval;

impl<S> Units for DiscreteModalSolver<S> where S: Solver + Default {}

impl<S> Update for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default + Send + Sync,
{
    fn update(&mut self) {
        log::debug!("update");
        self.next();
    }
}

#[cfg(all(fem, m1, m2))]
impl<S> Write<gmt_dos_clients_io::M12RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<gmt_dos_clients_io::M12RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Write<gmt_dos_clients_io::gmt_m1::M1RigidBodyMotions>>::write(
            self,
        )
        .zip(<DiscreteModalSolver<S> as Write<
            gmt_dos_clients_io::gmt_m2::M2RigidBodyMotions,
        >>::write(self))
        .map(|(m1, m2)| {
            m1.iter()
                .cloned()
                .chain(m2.iter().cloned())
                .collect::<Vec<_>>()
                .into()
        })
    }
}
impl<S, U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    Vec<Option<gmt_fem::fem_io::Inputs>>: crate::fem_io::FemIo<U>,
    S: Solver + Default + Send + Sync,
    U: 'static,
{
    fn read(&mut self, data: Data<U>) {
        <DiscreteModalSolver<S> as Set<U>>::set(self, &**data)
    }
}

impl<S, U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    Vec<Option<gmt_fem::fem_io::Outputs>>: crate::fem_io::FemIo<U>,
    S: Solver + Default + Send + Sync,
    U: 'static,
{
    fn write(&mut self) -> Option<Data<U>> {
        <DiscreteModalSolver<S> as Get<U>>::get(self).map(|data| Data::new(data))
    }
}

impl<S, U: UniqueIdentifier<DataType = Vec<f64>>> Write<operator::Left<U>>
    for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    Vec<Option<gmt_fem::fem_io::Outputs>>: crate::fem_io::FemIo<U>,
    S: Solver + Default + Send + Sync,
    U: 'static,
{
    fn write(&mut self) -> Option<Data<operator::Left<U>>> {
        <DiscreteModalSolver<S> as Get<U>>::get(self).map(|data| Data::new(data))
    }
}

impl<S, U: UniqueIdentifier<DataType = Vec<f64>>> Write<operator::Right<U>>
    for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    Vec<Option<gmt_fem::fem_io::Outputs>>: crate::fem_io::FemIo<U>,
    S: Solver + Default + Send + Sync,
    U: 'static,
{
    fn write(&mut self) -> Option<Data<operator::Right<U>>> {
        <DiscreteModalSolver<S> as Get<U>>::get(self).map(|data| Data::new(data))
    }
}
