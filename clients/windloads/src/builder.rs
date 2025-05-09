use crate::{CfdLoads, Result, WindLoadsError, FOH, ZOH};

mod builder;
pub use builder::Builder;

impl Builder<ZOH> {
    /// Returns a [CfdLoads] [Builder]
    pub fn zoh<C: Into<String>>(cfd_case: C) -> Self {
        Self {
            cfd_case: cfd_case.into(),
            upsampling: ZOH(20),
            ..Default::default()
        }
    }
}
impl Builder<FOH> {
    /// Returns a [CfdLoads] [Builder]
    pub fn foh<C: Into<String>>(cfd_case: C, upsampling: usize) -> Self {
        Self {
            cfd_case: cfd_case.into(),
            upsampling: FOH::new(upsampling / 20),
            ..Default::default()
        }
    }
}
impl CfdLoads<ZOH> {
    /// Creates a new [CfdLoads] object
    pub fn zoh<C: Into<String>>(cfd_case: C) -> Builder<ZOH> {
        Builder::zoh(cfd_case)
    }
}
impl CfdLoads<FOH> {
    /// Creates a new [CfdLoads] object
    pub fn foh<C: Into<String>>(cfd_case: C, upsampling: usize) -> Builder<FOH> {
        Builder::foh(cfd_case, upsampling)
    }
}

impl<S> TryFrom<Builder<S>> for CfdLoads<S> {
    type Error = WindLoadsError;

    fn try_from(builder: Builder<S>) -> Result<Self> {
        builder.build()
    }
}
