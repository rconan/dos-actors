/*!
# CFD wind loads client implementation

The wind forces and moments derived from the CFD simulations are saved into a file called `monitors.csv.z`.

This file is loaded into the client [CfdLoads] and resampled at a given sampling frequency.

For example:
```no_run
use gmt_dos_clients_windloads::CfdLoads;

let cfd_loads = CfdLoads::foh(".", 100)
    .duration(100.0)
    .mount(None)
    .m1_segments()
    .m2_segments()
    .build()?;
# Ok::<(), gmt_dos_clients_windloads::WindLoadsError>(())
```
The CFD wind loads are loaded from the current directory, resampled at 100Hz, truncated to the first 100s of data, and includes the loads on the mount, on M1 and on M2.

The version of the wind loads is selected by setting feature to either `cfd2021` or `cdf2025`.

Note that if the environment variable `FEM_REPO` is set to a valid GMT FEM folder, then the version of the wind loads is derived from the FEM CFD inputs and no feature is required.

If the wind loads are applied to a GMT FEM, then the FEM CFD inputs must be matched against the CFD loads, like so:
```no_run
use gmt_fem::FEM;
use gmt_dos_clients_windloads::CfdLoads;

let mut fem = FEM::from_env()?;
let cfd_loads = CfdLoads::foh(".", 1000)
    .duration(30.0)
    .windloads(&mut fem, Default::default())
    .build()?;
# Ok::<(), anyhow::Error>(())
```

The [WindLoadsBuilder](windloads::WindLoadsBuilder) can be used to select wind loads between wind loads applied to the mount, the M1 assembly or the M2 assembly:
```no_run
use gmt_fem::FEM;
use gmt_dos_clients_windloads::{CfdLoads, windloads::WindLoadsBuilder};

let mut fem = FEM::from_env()?;
let cfd_loads = CfdLoads::foh(".", 1000)
    .duration(30.0)
    .windloads(&mut fem, WindLoadsBuilder::new().m1_assembly())
    .build()?;
# Ok::<(), anyhow::Error>(())
```
*/

use geotrans::{M1, M2, Segment, SegmentTrait, Transform};
use interface::filing::Codec;
use parse_monitors::Vector;
use serde::{Deserialize, Serialize};
use std::fmt;

mod actors_interface;
#[cfg(fem)]
pub mod system;

#[derive(Debug, thiserror::Error)]
pub enum WindLoadsError {
    #[error("loading the windloads failed")]
    Load(#[from] parse_monitors::MonitorsError),
    #[error("coordinates transformation failed")]
    Coordinates(#[from] geotrans::Error),
}
pub type Result<T> = std::result::Result<T, WindLoadsError>;

const MAX_DURATION: usize = 400;

#[cfg(any(cfd2021, cfd2025, feature = "cfd2021", feature = "cfd2025"))]
pub mod windloads;

#[cfg(any(cfd2021, cfd2025, feature = "cfd2021", feature = "cfd2025"))]
pub mod builder;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum CS {
    OSS(Vec<f64>),
    M1S(i32),
    M2S(i32),
}

pub type M1S = Segment<M1>;
pub type M2S = Segment<M2>;

/// Zero-order hold wind loads interpolation
///
/// Staircase interpolation between 2 CFD timestamps
#[derive(Clone, Default, Debug, Serialize, Deserialize)]
pub struct ZOH(usize);

/// First-order hold wind loads interpolation
///
/// Linear interpolation between 2 CFD timestamps
#[derive(Clone, Default, Debug, Serialize, Deserialize, PartialEq)]
pub struct FOH {
    rate: usize,
    i: usize,
    u: f64,
}
impl FOH {
    /// Creates a new first-order hold wind loads interpolator
    pub fn new(rate: usize) -> Self {
        Self {
            rate,
            ..Default::default()
        }
    }
    pub fn update(&mut self, step: usize) {
        self.i = step / self.rate;
        self.u = (step - self.i * self.rate) as f64 / self.rate as f64;
    }
    /// Interpolates linearly between 2 samples
    pub fn sample(&self, x: &[f64], n: usize) -> Option<Vec<f64>> {
        if let (Some(y0), Some(y1)) = (x.chunks(n).nth(self.i), x.chunks(n).nth(self.i + 1)) {
            Some(
                y0.iter()
                    .zip(y1.iter())
                    .map(|(y0, y1)| (y1 - y0) * self.u + y0)
                    .collect(),
            )
        } else {
            None
        }
    }
}
/// The CFD loads
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct CfdLoads<S> {
    oss: Option<Vec<f64>>,
    m1: Option<Vec<f64>>,
    m2: Option<Vec<f64>>,
    nodes: Option<Vec<(String, CS)>>,
    n_fm: usize,
    step: usize,
    upsampling: S,
    max_step: usize,
}

impl<S: Serialize + for<'de> Deserialize<'de>> Codec for CfdLoads<S> {}

impl<S> CfdLoads<S> {
    pub fn oss_mean(&self) -> Option<Vec<f64>> {
        self.oss.as_ref().map(|oss| {
            let n_step = (oss.len() / self.n_fm) as f64;
            oss.chunks(self.n_fm)
                .fold(vec![0f64; self.n_fm], |mut a, x| {
                    a.iter_mut().zip(x.iter()).for_each(|(a, x)| *a += x);
                    a
                })
                .into_iter()
                .map(|x| x / n_step)
                .collect::<Vec<f64>>()
        })
    }
    pub fn m1_mean(&self) -> Option<Vec<f64>> {
        self.m1.as_ref().map(|oss| {
            let n_step = (oss.len() / 42) as f64;
            oss.chunks(42)
                .fold(vec![0f64; 42], |mut a, x| {
                    a.iter_mut().zip(x.iter()).for_each(|(a, x)| *a += x);
                    a
                })
                .into_iter()
                .map(|x| x / n_step)
                .collect::<Vec<f64>>()
        })
    }
    pub fn stop_after(&mut self, max_step: usize) -> &mut Self {
        self.max_step = max_step;
        self
    }
    pub fn start_from(&mut self, step: usize) -> &mut Self {
        self.max_step = usize::MAX;
        self.step = step + 1;
        self
    }
}
impl<S> fmt::Display for CfdLoads<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(oss) = self.oss_mean() {
            writeln!(f, "CFD loads in OSS ({}):", oss.len() / 6)?;
            for (oss, (key, loc)) in oss.chunks(6).zip(
                self.nodes
                    .as_ref()
                    .expect("CFD loads locations missing")
                    .iter(),
            ) {
                if let CS::OSS(loc) = loc {
                    writeln!(
                        f,
                        " - {:<20} @ {:>5.1?}m : <{:>6.0?}>N <{:>6.0?}>N.m",
                        key,
                        loc,
                        &oss[..3],
                        &oss[3..]
                    )?;
                }
            }
        }
        if let Some(oss) = self.m1_mean() {
            writeln!(f, "CFD loads in M1 local:")?;
            let mut force = Vector::zero();
            let mut moment = Vector::zero();
            for (i, oss) in oss.chunks(6).enumerate() {
                writeln!(
                    f,
                    " - M1S{:} : <{:>6.0?}>N <{:>6.0?}>N.m",
                    i + 1,
                    &oss[..3],
                    &oss[3..]
                )?;
                let u: Vector = (&oss[..3])
                    .to_vec()
                    .vtov(M1S::new(i as i32 + 1))
                    .unwrap()
                    .into();
                let t: [f64; 3] = M1S::new(i as i32 + 1).unwrap().translation().into();
                let r: Vector = t.into();
                let mu = r.cross(&u).unwrap();
                force = force + u;
                let u: Vector = (&oss[3..])
                    .to_vec()
                    .vtov(M1S::new(i as i32 + 1))
                    .unwrap()
                    .into();
                moment = moment + u + mu;
            }
            let u: Option<Vec<f64>> = force.into();
            writeln!(f, " - sum mean forces (OSS) : {:6.0?}N", u.unwrap())?;
            let v: Option<Vec<f64>> = moment.into();
            writeln!(f, " - sum mean moments (OSS): {:6.0?}N.m", v.unwrap())?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gmt_fem::FEM;
    use std::env;

    #[test]
    fn loading() -> anyhow::Result<()> {
        let Ok(mut fem) = FEM::from_env() else {
            return Ok(());
        };
        let _cfd_loads = CfdLoads::foh(env::var("MONITORS").unwrap_or(".".to_owned()), 1000)
            .duration(30.0)
            .windloads(&mut fem, Default::default())
            .build();
        Ok(())
    }
}
