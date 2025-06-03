use gmt_dos_clients_io::cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads};
use interface::{Data, Size, UID, Update, Write};

use crate::{CfdLoads, FOH, ZOH};

impl Update for CfdLoads<ZOH> {}
impl Update for CfdLoads<FOH> {
    fn update(&mut self) {
        if self.step > self.max_step {
            self.step = usize::MAX;
        }
        self.upsampling.update(self.step);
        self.step += 1;
    }
}

#[derive(UID)]
pub enum MountLoads {}
impl Write<MountLoads> for CfdLoads<ZOH> {
    fn write(&mut self) -> Option<Data<MountLoads>> {
        self.oss.as_mut().and_then(|oss| {
            if oss.is_empty() {
                log::debug!("CFD Loads have dried out!");
                None
            } else {
                let data: Vec<f64> = oss.drain(..self.n_fm).collect();
                if data.is_empty() {
                    None
                } else {
                    Some(data.into())
                }
            }
        })
    }
}
impl Write<MountLoads> for CfdLoads<FOH> {
    fn write(&mut self) -> Option<Data<MountLoads>> {
        self.oss.as_mut().and_then(|oss| {
            self.upsampling
                .sample(oss, self.n_fm)
                .map(|data| data.into())
        })
    }
}
impl Size<MountLoads> for CfdLoads<FOH> {
    fn len(&self) -> usize {
        self.n_fm
    }
}
impl Size<MountLoads> for CfdLoads<ZOH> {
    fn len(&self) -> usize {
        self.n_fm
    }
}

impl Write<CFDMountWindLoads> for CfdLoads<FOH> {
    fn write(&mut self) -> Option<Data<CFDMountWindLoads>> {
        self.oss.as_mut().and_then(|oss| {
            self.upsampling
                .sample(oss, self.n_fm)
                .map(|data| data.into())
        })
    }
}
impl Write<CFDMountWindLoads> for CfdLoads<ZOH> {
    fn write(&mut self) -> Option<Data<CFDMountWindLoads>> {
        self.oss.as_mut().and_then(|oss| {
            if oss.is_empty() {
                log::debug!("CFD Loads have dried out!");
                None
            } else {
                let data: Vec<f64> = oss.drain(..self.n_fm).collect();
                if data.is_empty() {
                    None
                } else {
                    Some(data.into())
                }
            }
        })
    }
}
impl Size<CFDMountWindLoads> for CfdLoads<ZOH> {
    fn len(&self) -> usize {
        self.n_fm
    }
}
impl Size<CFDMountWindLoads> for CfdLoads<FOH> {
    fn len(&self) -> usize {
        self.n_fm
    }
}

impl Write<CFDM1WindLoads> for CfdLoads<ZOH> {
    fn write(&mut self) -> Option<Data<CFDM1WindLoads>> {
        self.m1.as_mut().and_then(|m1| {
            if m1.is_empty() {
                log::debug!("CFD Loads have dried out!");
                None
            } else {
                let data: Vec<f64> = m1.drain(..42).collect();
                if data.is_empty() {
                    None
                } else {
                    Some(data.into())
                }
            }
        })
    }
}
impl Write<CFDM1WindLoads> for CfdLoads<FOH> {
    fn write(&mut self) -> Option<Data<CFDM1WindLoads>> {
        self.m1
            .as_mut()
            .and_then(|m1| self.upsampling.sample(m1, 42).map(|data| data.into()))
    }
}
impl Size<CFDM1WindLoads> for CfdLoads<FOH> {
    fn len(&self) -> usize {
        42
    }
}
impl Size<CFDM1WindLoads> for CfdLoads<ZOH> {
    fn len(&self) -> usize {
        42
    }
}

impl Write<CFDM2WindLoads> for CfdLoads<ZOH> {
    fn write(&mut self) -> Option<Data<CFDM2WindLoads>> {
        self.m2.as_mut().and_then(|m2| {
            if m2.is_empty() {
                log::debug!("CFD Loads have dried out!");
                None
            } else {
                let data: Vec<f64> = m2.drain(..42).collect();
                if data.is_empty() {
                    None
                } else {
                    Some(data.into())
                }
            }
        })
    }
}
impl Write<CFDM2WindLoads> for CfdLoads<FOH> {
    fn write(&mut self) -> Option<Data<CFDM2WindLoads>> {
        self.m2
            .as_mut()
            .and_then(|m2| self.upsampling.sample(m2, 42).map(|data| data.into()))
    }
}
impl Size<CFDM2WindLoads> for CfdLoads<ZOH> {
    fn len(&self) -> usize {
        42
    }
}
impl Size<CFDM2WindLoads> for CfdLoads<FOH> {
    fn len(&self) -> usize {
        42
    }
}

#[derive(UID)]
pub enum MountM2M1Loads {}
impl Write<MountM2M1Loads> for CfdLoads<FOH> {
    fn write(&mut self) -> Option<Data<MountM2M1Loads>> {
        let v: Vec<f64> = self
            .oss
            .as_mut()
            .and_then(|oss| self.upsampling.sample(oss, self.n_fm))
            .into_iter()
            .chain(
                self.m2
                    .as_mut()
                    .and_then(|m2| self.upsampling.sample(m2, 42)),
            )
            .chain(
                self.m1
                    .as_mut()
                    .and_then(|m1| self.upsampling.sample(m1, 42)),
            )
            .flatten()
            .collect();
        if v.is_empty() { None } else { Some(v.into()) }
    }
}
