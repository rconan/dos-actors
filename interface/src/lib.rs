/*!
# gmt_dos-actors-clients_interface

Interface definition betweeen an [actor] and an [actor]'s client.

Data is passed from the [actor] to the client by invoking [Read::read] from the client.

Data is passed from the client to the [actor] by invoking [Write::write] from the client.

The client state may be updated by invoking [Update::update] from the client

[actor]: https://docs.rs/gmt_dos-actors
*/

use std::any::type_name;

mod data;
pub use data::Data;
pub use dos_uid_derive::UID;
pub mod units;

pub mod select;

#[cfg(any(feature = "filing", feature = "pickling"))]
pub mod filing;

pub type Assoc<U> = <U as UniqueIdentifier>::DataType;

/// Marker to allow the UID data to be either left or right added or substracted with the [Operator](https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/operator/index.html) client
pub trait OperatorLeftRight {
    const LEFT: bool;
}
/// Units conversion marker trait for clients
pub trait Units {}

/// Timer heartbeat identifier
pub enum Tick {}
impl UniqueIdentifier for Tick {
    type DataType = ();
}
/// Timer marker trait
pub trait TimerMarker {}
impl<T> Read<Tick> for T
where
    T: TimerMarker + Update,
{
    fn read(&mut self, _: Data<Tick>) {}
}

/// Defines the data type associated with unique identifier data type
pub trait UniqueIdentifier: Send + Sync {
    const PORT: u16 = 50_000;
    type DataType: Send + Sync;
}
pub trait Quote {
    fn quote() -> String;
}
impl<U: UniqueIdentifier> Quote for U {
    fn quote() -> String {
        fn inner(name: &str) -> String {
            if let Some((prefix, suffix)) = name.split_once('<') {
                let generics: Vec<_> = suffix.split(',').map(|s| inner(s)).collect();
                format!("{}<{}", inner(prefix), generics.join(","))
            } else {
                if let Some((_, suffix)) = name.rsplit_once("::") {
                    suffix.into()
                } else {
                    name.into()
                }
            }
        }
        inner(type_name::<U>())
    }
}
impl UniqueIdentifier for () {
    type DataType = ();
}

/// Actor client state update interface
pub trait Update: Send + Sync {
    fn update(&mut self) {}
}
/// Client input data reader interface
pub trait Read<U: UniqueIdentifier>: Update {
    /// Read data from an input
    fn read(&mut self, data: Data<U>);
}
/// Client output data writer interface
pub trait Write<U: UniqueIdentifier>: Update {
    fn write(&mut self) -> Option<Data<U>>;
}
/// Interface for IO data sizes
pub trait Size<U: UniqueIdentifier>: Update {
    fn len(&self) -> usize;
}

pub trait Who<T> {
    /// Returns type name
    fn who(&self) -> String {
        type_name::<T>().to_string()
    }
    fn highlight(&self) -> String {
        let me = <Self as Who<T>>::who(&self);
        paris::formatter::colorize_string(format!("<italic><on-bright-cyan>{}</>", me))
    }
    fn lite(&self) -> String {
        let me = <Self as Who<T>>::who(&self);
        paris::formatter::colorize_string(format!("<italic><bright-cyan>{}</>", me))
    }
}

use log::{info, warn};

/// Pretty prints error message
pub fn print_info<S: Into<String>>(msg: S, e: Option<&dyn std::error::Error>) {
    if let Some(e) = e {
        let mut msg: Vec<String> = vec![msg.into()];
        msg.push(format!("{}", e));
        let mut current = e.source();
        while let Some(cause) = current {
            msg.push(format!("{}", cause));
            current = cause.source();
        }
        warn!("{}", msg.join("\n .due to: "))
    } else {
        info!("{}", msg.into())
    }
}

/// Interface for data logging types
pub trait Entry<U: UniqueIdentifier>: Update {
    /// Adds an entry to the logger
    fn entry(&mut self, size: usize);
}

pub fn trim_type_name<T>() -> String {
    fn trim(name: &str) -> String {
        if let Some((prefix, suffix)) = name.split_once('<') {
            let generics: Vec<_> = suffix.split(',').map(|s| trim(s)).collect();
            format!("{}<{}", trim(prefix), generics.join(","))
        } else {
            if let Some((_, suffix)) = name.rsplit_once("::") {
                suffix.into()
            } else {
                name.into()
            }
        }
    }
    trim(type_name::<T>())
}

/**
Clients chain

Apply the traits [Read], [Update] and [Write] (and in that order)
 to clients which inputs and outputs from an uninterrupted chain

A single client:
```ignore
chain!(
    input_uid: input_data;
    client;
    output_uid: output_data);
```

Two clients where the UID of the output of `client1` is
also the UID of the input of `client2`:
```ignore
chain!(
    client1_input_uid: client_1_input_data;
    client1;
    common_uid;
    client2:
    client2_output_uid: client2_output_data);
    );
```

*/
#[macro_export]
macro_rules! chain {
    ($r:ty:$vr:expr;$e:expr) => {
        <_ as ::interface::Read<$r>>::read($e, $vr);
        <_ as ::interface::Update>::update($e);
    };
    ($e:expr;$w:ty:$vw:ident) => {
        let $vw = {
            <_ as ::interface::Update>::update($e);
            <_ as ::interface::Write<$w>>::write($e).expect(&format!(
                "cannot write to {}",
                ::std::any::type_name::<$w>()
            ))
        };
    };
    ($r:ty:$vr:expr;$e:expr;$w:ty:$vw:ident) => {
        let $vw = {
            <_ as ::interface::Read<$r>>::read($e, $vr);
            <_ as ::interface::Update>::update($e);
            <_ as ::interface::Write<$w>>::write($e).expect(&format!(
                "cannot write to {}",
                ::std::any::type_name::<$w>()
            ))
        };
    };
    ($r1:ty:$vr1:expr;$e1:expr;$w1:ty;$e2:expr) => {
        ::interface::chain!($r1:$vr1;$e1;$w1:data);
        ::interface::chain!($w1:data;$e2);
    };
    ($e1:expr;$w1:ty;$e2:expr) => {
        ::interface::chain!($e1;$w1:data);
        ::interface::chain!($w1:data;$e2);
    };
    ($e1:expr;$w1:ty;$e2:expr;$w2:ty:$vw2:ident) => {
        ::interface::chain!($e1;$w1:data);
        ::interface::chain!($w1:data;$e2;$w2:$vw2);
    };
    ($r1:ty:$vr1:expr;$e1:expr;$w1:ty;$e2:expr;$w2:ty:$vw2:ident) => {
        ::interface::chain!($r1:$vr1;$e1;$w1:data);
        ::interface::chain!($w1:data;$e2;$w2:$vw2);
    };
    ($r1:ty:$vr1:expr;$e1:expr;$w1:ty;$e2:expr;$w2:ty;$e3:expr;$w3:ty:$vw3:ident) => {
        ::interface::chain!($r1:$vr1;$e1;$w1;$e2;$w2:data);
        ::interface::chain!($w2:data;$e3;$w3:$vw3);
    };
    ($r1:ty:$vr1:expr;$e1:expr;$w1:ty;$e2:expr;$w2:ty;$e3:expr;$w3:ty;$e4:expr;$w4:ty:$vw4:ident) => {
        ::interface::chain!($r1:$vr1;$e1;$w1;$e2;$w2;$e3;$w3:data);
        ::interface::chain!($w3:data;$e4;$w4:$vw4);
    };
}
