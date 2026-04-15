use std::{fmt::Display, marker::PhantomData};

use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions};
use interface::{
    Data, UniqueIdentifier, Write,
    units::{Arcsec, Mas, MuM, NM, UnitsConversion},
};

use crate::MirrorState;

/// GMT mirror rigid body motions marker trait
pub trait MirrorRbms: UniqueIdentifier<DataType = Vec<f64>> {}

impl MirrorRbms for M1RigidBodyMotions {}
impl MirrorRbms for M2RigidBodyMotions {}

/// Translations in `[nm]` and rotations in `[arcsec]`
pub type NmArcsec<T> = RbmsUnits<T, NM<T>, Arcsec<T>>;
/// Translations in `[nm]` and rotations in `[mas]`
pub type NmMas<T> = RbmsUnits<T, NM<T>, Mas<T>>;
/// Translations in `[micron]` and rotations in `[arcsec]`
pub type MuArcsec<T> = RbmsUnits<T, MuM<T>, Arcsec<T>>;
/// Translations in `[micron]` and rotations in `[mas]`
pub type MuMas<T> = RbmsUnits<T, MuM<T>, Mas<T>>;

/// Units conversion form GMT M1 & M2 rigid body motions
pub struct RbmsUnits<T, TU, RU>(PhantomData<T>, PhantomData<TU>, PhantomData<RU>)
where
    T: MirrorRbms,
    TU: UnitsConversion,
    RU: UnitsConversion;
impl<T, TU, RU> UniqueIdentifier for RbmsUnits<T, TU, RU>
where
    T: MirrorRbms,
    TU: UnitsConversion + UniqueIdentifier,
    RU: UnitsConversion + UniqueIdentifier,
{
    const PORT: u16 = <T as UniqueIdentifier>::PORT;
    type DataType = <T as UniqueIdentifier>::DataType;
}

impl<T, TU, RU> RbmsUnits<T, TU, RU>
where
    T: MirrorRbms,
    TU: UnitsConversion,
    RU: UnitsConversion,
{
    pub fn new() -> Self {
        Self(PhantomData, PhantomData, PhantomData)
    }
}

impl<T: MirrorRbms> Display for NmArcsec<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[nm,arcsec]")
    }
}
impl<T: MirrorRbms> Display for NmMas<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[nm,mas]")
    }
}
impl<T: MirrorRbms> Display for MuArcsec<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[micron,arcsec]")
    }
}
impl<T: MirrorRbms> Display for MuMas<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[micron,mas]")
    }
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

#[cfg(test)]
mod tests {
    use skyangle::Conversion;

    use crate::SegmentState;

    use super::*;

    #[test]
    fn convert() {
        let mut m1 = MirrorState::default().set_segment_state(
            1,
            SegmentState::rbms([1e-6, 0., 0., 150f64.from_mas(), 0., 0.]),
        );
        let data = <_ as Write<NmMas<M1RigidBodyMotions>>>::write(&mut m1).unwrap();
        println!("{}", NmMas::<M1RigidBodyMotions>::new());
        dbg!(&data);
        let data = <_ as Write<MuArcsec<M1RigidBodyMotions>>>::write(&mut m1).unwrap();
        println!("{}", MuArcsec::<M1RigidBodyMotions>::new());
        dbg!(&data);
    }
}
