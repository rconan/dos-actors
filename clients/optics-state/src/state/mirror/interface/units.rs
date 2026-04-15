use std::marker::PhantomData;

use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions};
use interface::{
    Data, UniqueIdentifier, Write,
    units::{Arcsec, Mas, MuM, NM, UnitsConversion},
};

use crate::MirrorState;

pub trait MirrorRbms: UniqueIdentifier<DataType = Vec<f64>> {}

impl MirrorRbms for M1RigidBodyMotions {}
impl MirrorRbms for M2RigidBodyMotions {}

pub type NmArcsec<T> = RbmsUnits<T, NM<T>, Arcsec<T>>;
pub type NmMas<T> = RbmsUnits<T, NM<T>, Mas<T>>;
pub type MuArcsec<T> = RbmsUnits<T, MuM<T>, Arcsec<T>>;
pub type MuMas<T> = RbmsUnits<T, MuM<T>, Mas<T>>;

pub struct RbmsUnits<T: MirrorRbms, TU: UnitsConversion, RU: UnitsConversion>(
    PhantomData<T>,
    PhantomData<TU>,
    PhantomData<RU>,
);
impl<T, TU, RU> UniqueIdentifier for RbmsUnits<T, TU, RU>
where
    T: MirrorRbms,
    TU: UnitsConversion + UniqueIdentifier,
    RU: UnitsConversion + UniqueIdentifier,
{
    const PORT: u16 = <T as UniqueIdentifier>::PORT;
    type DataType = <T as UniqueIdentifier>::DataType;
}

impl<T, TU, RU> Write<RbmsUnits<T, TU, RU>> for MirrorState
where
    T: MirrorRbms,
    TU: UnitsConversion<ID = T> + UniqueIdentifier<DataType = <T as UniqueIdentifier>::DataType>,
    RU: UnitsConversion<ID = T> + UniqueIdentifier<DataType = <T as UniqueIdentifier>::DataType>,
    Self: Write<T>,
{
    fn write(&mut self) -> Option<Data<RbmsUnits<T, TU, RU>>> {
        <Self as Write<T>>::write(self)
            .map(|data| {
                data.chunks(6)
                    .flat_map(|data| {
                        let t_xyz =
                            <TU as UnitsConversion>::conversion(&Data::new(data[..3].to_vec()))
                                .unwrap();
                        let r_xyz =
                            <RU as UnitsConversion>::conversion(&Data::new(data[3..].to_vec()))
                                .unwrap();
                        let mut rbms = Vec::<f64>::new();
                        rbms.extend(&*t_xyz);
                        rbms.extend(&*r_xyz);
                        rbms
                    })
                    .collect::<Vec<_>>()
            })
            .map(|rbms| rbms.into())
    }
}
