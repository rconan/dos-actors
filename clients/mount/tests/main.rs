use std::{fmt::Display, ops::Range};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Signal, Signals};
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix, fem_io::actors_outputs::*};
use gmt_dos_clients_io::{
    // gmt_m1::M1RigidBodyMotions,
    // gmt_m2::M2RigidBodyMotions,
    mount::{MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_mount::Mount;
use gmt_fem::FEM;
// use gmt_lom::{OpticalMetrics, LOM};
use skyangle::Conversion;
use tokio::sync::MutexGuard;

const SETTLING: f64 = 0.02;

/*
DATA:
 * FEM 2nd order model: FEM_REPO
 * linear optical sensitivity matrices: LOM

MOUNT_MODEL=... cargo test --release --package gmt_dos-clients_mount --test main -- --nocapture
*/

/// Mount axis
#[derive(Default, Clone, Copy)]
pub enum Axes {
    #[default]
    None,
    Azimuth,
    Elevation,
    GIR,
}
impl Axes {
    pub fn idx(self) -> Range<usize> {
        match self {
            Self::None => 0..0,
            Self::Azimuth => 0..4,
            Self::Elevation => 4..10,
            Self::GIR => 10..14,
        }
    }
}
impl Display for Axes {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::None => write!(f, "None"),
            Self::Azimuth => write!(f, "Azimuth"),
            Self::Elevation => write!(f, "Elevation"),
            Self::GIR => write!(f, "GIR"),
        }
    }
}

/// Mount encoder stats
#[derive(Default)]
pub struct Stats {
    axis: Axes,
    pub mean: f64,
    pub var: f64,
    pub upper: f64,
    pub lower: f64,
}
impl Stats {
    pub fn std(&self) -> f64 {
        self.var.sqrt()
    }
}
impl Display for Stats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "{}:", self.axis)?;
        writeln!(
            f,
            " * mean: {:.3}mas (1 sigma={:.3}mas)",
            self.mean.to_mas(),
            self.std().to_mas()
        )?;
        writeln!(
            f,
            " * relative-to-mean min/max: {:.3}/{:.3} mas",
            self.upper.to_mas(),
            self.lower.to_mas()
        )
    }
}

/// Mount encoders data for a given axis
pub struct Data<'a> {
    axis: Axes,
    data: &'a [Vec<f64>],
}
impl<'a> Data<'a> {
    pub fn new(axis: Axes, data: &'a [Vec<f64>]) -> Self {
        Self { axis, data }
    }
    pub fn axis_mean_var(&self) -> (Vec<f64>, Vec<f64>) {
        self.data
            .iter()
            .map(|x| {
                let y = &x[self.axis.idx()];
                let (mut mean, mut var) = y.iter().fold((0f64, 0f64), |(a, b), x| {
                    let a = a + x;
                    let b = b + x * x;
                    (a, b)
                });
                let n = y.len() as f64;
                mean /= n;
                var /= n;
                var -= mean * mean;
                (mean, var)
            })
            .unzip()
    }
    pub fn stats(&self) -> Stats {
        let (axis_mean, axis_var) = self.axis_mean_var();
        let n = axis_mean.len() as f64;
        let mean = axis_mean.iter().sum::<f64>() / n;
        let var = axis_var.iter().sum::<f64>() / n;
        let upper = axis_mean.iter().max_by(|a, b| a.total_cmp(b)).unwrap() - mean;
        let lower = axis_mean.iter().min_by(|a, b| a.total_cmp(b)).unwrap() - mean;
        Stats {
            axis: self.axis,
            mean,
            var,
            upper,
            lower,
        }
    }
}

/// Mount encoders
#[derive(Default)]
struct Encoders {
    #[allow(dead_code)]
    data: Vec<Vec<f64>>,
    duration: f64,
    pub elevation: Stats,
    pub azimuth: Stats,
    pub gir: Stats,
}
// impl TryFrom<MutexGuard<Arrow>> for Encoders {
//     type Error;

//     fn try_from(mut logs: MutexGuard<Arrow>) -> Result<Self, Self::Error> {
//         Ok(Encoders {
//             data: logs.iter("MountEncoders")?.collect(),
//         })
//     }
// }
impl Encoders {
    pub fn new(mut logs: MutexGuard<Arrow>, last_seconds: f64) -> Result<Self, anyhow::Error> {
        let data: Vec<Vec<f64>> = logs
            .iter("MountEncoders")?
            .rev()
            .take(
                (last_seconds * gmt_dos_clients_mount::sampling_frequency() as f64).ceil() as usize,
            )
            .rev()
            .collect();
        Ok(Self {
            elevation: Data::new(Axes::Elevation, &data).stats(),
            azimuth: Data::new(Axes::Azimuth, &data).stats(),
            gir: Data::new(Axes::GIR, &data).stats(),
            data,
            duration: last_seconds,
        })
    }
    pub fn assert_elevation(&self, bound: f64) {
        assert!(self.elevation.upper.abs() < bound);
        assert!(self.elevation.lower.abs() < bound);
    }
    pub fn assert_azimuth(&self, bound: f64) {
        assert!(self.azimuth.upper.abs() < bound);
        assert!(self.azimuth.lower.abs() < bound);
    }
    pub fn assert_gir(&self, bound: f64) {
        assert!(self.gir.upper.abs() < bound);
        assert!(self.gir.lower.abs() < bound);
    }
    pub fn assert_all(&self, bound: f64) {
        self.assert_elevation(bound);
        self.assert_azimuth(bound);
        self.assert_gir(bound);
    }
}
impl Display for Encoders {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Mount encoders last {}s stats", self.duration)?;
        write!(f, "{}", self.azimuth)?;
        write!(f, "{}", self.elevation)?;
        writeln!(f, "{}", self.gir)
    }
}

async fn set_mount(sim_sampling_frequency: usize, setpoint: Signals) -> anyhow::Result<Encoders> {
    // FEM MODEL
    let state_space = {
        let fem = FEM::from_env()?;
        // println!("{fem}");
        DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(2. / 100.)
            .including_mount()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .use_static_gain_compensation()
            .build()?
    };
    // println!("{state_space}");

    // FEM
    let fem = state_space;
    // MOUNT CONTROL
    let mount = Mount::new();

    actorscript! {
        #[labels(fem = "FEM", mount = "Mount\nControl")]
        1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]!${14} -> mount
        // 1: fem[M1RigidBodyMotions]$
        // 1: fem[M2RigidBodyMotions]$
    }

    let logs = logging_1.lock().await;

    /*     // Linear optical sensitivities to derive segment tip and tilt
    let lom = LOM::builder()
        .rigid_body_motions_record(
            logs.record()?,
            Some("M1RigidBodyMotions"),
            Some("M2RigidBodyMotions"),
        )?
        .build()?;
    let segment_tiptilt = lom.segment_tiptilt();
    let stt = segment_tiptilt.items().last().unwrap();
    let tiptilt = lom.tiptilt();
    let tt = tiptilt.items().last().unwrap();

    println!("Segment TT: {:.3?}mas", stt.to_mas());
    println!("TT: {:.3?}mas", tt.to_mas());
    // assert!(tt[0].hypot(tt[1]).to_mas() - 1000. < 1.); */

    Encoders::new(logs, 0.5)
}

/// Moves the mount 1arcsec along the elevation axis of the telescope
///
/// The test is succesfull if the mount has settled to the target within 2% of the step after 3.5s
#[tokio::test(flavor = "multi_thread")]
async fn elevation() -> anyhow::Result<()> {
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 4_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;
    // SET POINT
    let step = 1000_f64;
    let setpoint = Signals::new(3, n_step).channel(1, Signal::Constant(step.from_mas()));
    let encs = set_mount(sim_sampling_frequency, setpoint).await?;
    println!("{encs}");
    let bound = SETTLING * step;
    encs.assert_all(bound);
    Ok(())
}

/// Moves the mount 1arcsec along the azimuth axis of the telescope
///
/// The test is succesfull if the mount has settled to the target within 2% of the step after 3.5s
#[tokio::test(flavor = "multi_thread")]
async fn azimuth() -> anyhow::Result<()> {
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 4_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;
    // SET POINT
    let step = 1000_f64;
    let setpoint = Signals::new(3, n_step).channel(0, Signal::Constant(step.from_mas()));
    let encs = set_mount(sim_sampling_frequency, setpoint).await?;
    println!("{encs}");
    let bound = SETTLING * step;
    encs.assert_all(bound);
    Ok(())
}

/// Moves the mount 1arcsec along the GIR axis of the telescope
///
/// The test is succesfull if the mount has settled to the target within 2% of the step after 3.5s
#[tokio::test(flavor = "multi_thread")]
async fn gir() -> anyhow::Result<()> {
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 4_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;
    // SET POINT
    let step = 1000_f64;
    let setpoint = Signals::new(3, n_step).channel(2, Signal::Constant(step.from_mas()));
    let encs = set_mount(sim_sampling_frequency, setpoint).await?;
    println!("{encs}");
    let bound = SETTLING * step;
    encs.assert_all(bound);
    Ok(())
}

/// Zero command test
///
/// The test is succesfull if the last position of the 3 axis is less than 1mas to zero
#[tokio::test(flavor = "multi_thread")]
async fn zero() -> anyhow::Result<()> {
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 4_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;
    // SET POINT
    let setpoint = Signals::new(3, n_step);
    let encs = set_mount(sim_sampling_frequency, setpoint).await?;
    println!("{encs}");
    let bound = 1f64;
    encs.assert_all(bound);
    Ok(())
}
