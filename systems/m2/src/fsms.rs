pub use assembly::{DispatchIn, DispatchOut, FsmsInnerControllers, FSMS};
use gmt_dos_actors::system::{Sys, SystemError};
use gmt_dos_clients_io::Assembly;

mod assembly;

impl<const R: usize> FSMS<R> {
    /// Creates a new [FSMS] [system ](gmt_dos_actors::system::Sys) instance
    pub fn new() -> Result<Sys<Self>, SystemError> {
        let segments: Vec<_> = <Self as Assembly>::SIDS
            .into_iter()
            .map(|id| FsmsInnerControllers::new(id))
            .collect();
        Sys::new(Self {
            segments,
            dispatch_in: DispatchIn::new().into(),
            dispatch_out: DispatchOut::new().into(),
        })
        .build()
    }
}
