use std::{env, fs, path::Path};

mod rigid_body;
use rigid_body::{KinematicsCalibration, RigidBodyRemoval, load_azimuth_calibration};

use gmt_dos_actors::actorscript;
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    mount::{MountEncoders, MountSetPoint},
};
use gmt_dos_clients_io::optics::SegmentTipTilt;
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_mount::Mount;
use gmt_dos_clients_servos::{GmtFem, GmtMount, GmtServoMechanisms, KinematicOutput, MountAxis};
use gmt_fem::FEM;
use interface::{Data, Size, TimerMarker, UniqueIdentifier, Update, Write};
// use interface::units::Arcsec;

// Drive the mount along the ATP default tracking trajectory
//  * pointing: alt 45deg, az 0deg, rotator 0deg, 2018-01-01T04:00:00 UTC, Las Campanas
//  * 5 minutes of azimuth, elevation & rotator (GIR) samples at 1Hz in `atp_trajectory.csv`,
//    computed with astropy, the rotator following the change in parallactic angle
//  * the setpoints are the trajectory offsets from the initial pointing,
//    linearly interpolated to the mount sampling frequency
//  * the rigid telescope motion is removed from all the FEM outputs (but the mount encoders)
//    by the FEM kinematic compensation, the kinematic gain from the mount encoders derived
//    from the FEM DC gain, with the M1 & M2 azimuth gains calibrated in closed-loop
//    (`make calibrate`, `rigid_body.rs`)
//  * the mount encoders, the compensated M1 & M2 rigid body motions and the segment tip-tilt
//    from the linear optical model are saved to `model_data_1.parquet` in this folder
// DATA:
//  * FEM 2nd order model: FEM_REPO

/*
make run (see Makefile for the parameters)
MOUNT_MODEL=MOUNT_FDR_1kHz FEM_REPO=... CUDACXX=/usr/local/cuda/bin/nvcc cargo run --release --features "cuda gmt_dos-clients_arrow" --bin trajectory-servos-mount
SIM_DURATION=10 (optional, in seconds, default: 300)
*/

/// Mount setpoints [azimuth, elevation, GIR] in radians, interpolated from 1Hz samples
pub struct Trajectory {
    samples: Vec<[f64; 3]>,
    upsampling: usize,
    step: usize,
    n_step: usize,
}
impl Trajectory {
    /// Loads the trajectory (in degrees) and removes the initial pointing
    pub fn from_csv(path: impl AsRef<Path>, upsampling: usize, n_step: usize) -> anyhow::Result<Self> {
        let samples: Vec<[f64; 3]> = fs::read_to_string(path)?
            .lines()
            .skip(1)
            .map(|line| {
                let v: Vec<f64> = line
                    .split(',')
                    .skip(1)
                    .map(|x| x.trim().parse::<f64>())
                    .collect::<Result<_, _>>()?;
                Ok([v[0].to_radians(), v[1].to_radians(), v[2].to_radians()])
            })
            .collect::<anyhow::Result<_>>()?;
        let s0 = samples[0];
        let samples = samples
            .into_iter()
            .map(|s| [s[0] - s0[0], s[1] - s0[1], s[2] - s0[2]])
            .collect();
        Ok(Self {
            samples,
            upsampling,
            step: 0,
            n_step,
        })
    }
    /// Keeps only the azimuth, ramping along the trajectory for `ramp` samples and then holding
    pub fn azimuth_ramp_hold(mut self, ramp: usize) -> Self {
        let hold = self.samples[ramp.min(self.samples.len() - 1)][0];
        self.samples
            .iter_mut()
            .enumerate()
            .for_each(|(i, s)| *s = [if i < ramp { s[0] } else { hold }, 0., 0.]);
        self
    }
}
impl TimerMarker for Trajectory {}
impl Update for Trajectory {}
impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for Trajectory {
    fn write(&mut self) -> Option<Data<U>> {
        if self.step >= self.n_step {
            return None;
        }
        let i = self.step / self.upsampling;
        let a = (self.step % self.upsampling) as f64 / self.upsampling as f64;
        let s0 = self.samples.get(i)?;
        let s1 = self.samples.get(i + 1).unwrap_or(s0);
        self.step += 1;
        Some(Data::new(
            s0.iter().zip(s1).map(|(x0, x1)| x0 + a * (x1 - x0)).collect(),
        ))
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    unsafe {
        env::set_var(
            "DATA_REPO",
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("src")
                .join("bin")
                .join("trajectory-servos-mount"),
        );
    }
    // simulation sampling frequency
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency(); // Hz

    // simulation duration
    let sim_duration = env::var("SIM_DURATION")
        .ok()
        .map_or(Ok(300_usize), |s| s.parse())?; // second

    // simulation discrete time steps #
    let n_step = sim_sampling_frequency * sim_duration;

    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("bin")
        .join("trajectory-servos-mount");
    // closed-loop azimuth kinematics calibration file
    let calibration = env::var("KINEMATICS_CALIBRATION")
        .map_or_else(|_| data_repo.join("kinematics_calibration.csv"), |path| path.into());

    let fem = FEM::from_env()?;
    // RIGID TELESCOPE MOTION REMOVAL (kinematics from the FEM DC gain)
    // * inputs: MountEncoders, M1RigidBodyMotions, M2RigidBodyMotions
    // * outputs: M1RigidBodyMotions, M2RigidBodyMotions
    let fem_kinematics = RigidBodyRemoval::from_fem(&fem)?;
    println!(
        "FEM vs geometric kinematics, max difference per radian of [az, el, GIR]: {:?}",
        fem_kinematics.max_difference(&RigidBodyRemoval::from_geometry())
    );

    // CLOSED-LOOP AZIMUTH KINEMATICS CALIBRATION (CALIBRATE env var)
    //  * azimuth only: ramping along the ATP trajectory for CALIBRATION_RAMP seconds (default: 15s)
    //    and holding still for 15s, the rigid body motions are averaged over the last 5s
    //  * saved to KINEMATICS_CALIBRATION (default: kinematics_calibration.csv in this folder)
    if env::var("CALIBRATE").is_ok() {
        let ramp = env::var("CALIBRATION_RAMP")
            .ok()
            .map_or(Ok(15_usize), |s| s.parse())?; // second
        let n_step = sim_sampling_frequency * (ramp + 15);
        let setpoint = Trajectory::from_csv(
            data_repo.join("atp_trajectory.csv"),
            sim_sampling_frequency,
            n_step,
        )?
        .azimuth_ramp_hold(ramp);
        // FEM without kinematic compensation
        let servos =
            GmtServoMechanisms::<10, 1>::new(sim_sampling_frequency as f64, fem).build()?;
        let calib = KinematicsCalibration::new(
            &fem_kinematics,
            n_step - 5 * sim_sampling_frequency..n_step,
            &calibration,
        );
        actorscript! {
            1: setpoint[MountSetPoint] -> {servos::GmtMount}
            1: {servos::GmtFem}[MountEncoders]! -> calib
            1: {servos::GmtFem}[M1RigidBodyMotions]! -> calib
            1: {servos::GmtFem}[M2RigidBodyMotions]! -> calib
        }
        return Ok(());
    }

    // FEM with kinematic compensation
    let servos = GmtServoMechanisms::<10, 1>::new(sim_sampling_frequency as f64, fem);
    let servos = if calibration.exists() {
        println!("loading the closed-loop azimuth kinematics from {calibration:?}");
        let [m1, m2] = load_azimuth_calibration(&calibration)?;
        servos
            .kinematic_calibration(KinematicOutput::M1RigidBodyMotions, MountAxis::Azimuth, m1)
            .kinematic_calibration(KinematicOutput::M2RigidBodyMotions, MountAxis::Azimuth, m2)
    } else {
        println!("no closed-loop azimuth kinematics calibration, using the FEM DC gain");
        servos.kinematic_compensation()
    }
    .build()?;

    // [SET POINT]
    // ATP trajectory for the 3 axes of the mount:
    //   * azimuth (channel #0)
    //   * elevation (channel #1)
    //   * GIR (channel #2)
    // * output: MountSetPoint
    // TRAJECTORY (optional): path to an alternative trajectory file
    let trajectory = env::var("TRAJECTORY").map_or_else(
        |_| {
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("src")
                .join("bin")
                .join("trajectory-servos-mount")
                .join("atp_trajectory.csv")
        },
        |path| path.into(),
    );
    let setpoint = Trajectory::from_csv(
        trajectory,
        sim_sampling_frequency,
        n_step,
    )?;
    // number of mount encoders (logged size of MountEncoders)
    let n_encoders = <Mount as Size<MountEncoders>>::len(&Mount::new());

    // [LOM (Linear Optical Model)](crates.io/crates/gmt_dos-clients_lom)
    // * inputs: M1RigidBodyMotions, M2RigidBodyMotions
    // * output: SegmentTipTilt
    let lom = LinearOpticalModel::new()?;

    actorscript! {
        1: setpoint[MountSetPoint] -> {servos::GmtMount}
        1: {servos::GmtFem}[MountEncoders]${n_encoders}
        1: {servos::GmtFem}[M1RigidBodyMotions]$
        1: {servos::GmtFem}[M2RigidBodyMotions]$
        1: {servos::GmtFem}[M1RigidBodyMotions]! -> lom
        1: {servos::GmtFem}[M2RigidBodyMotions]! -> lom
        1: lom[SegmentTipTilt]$
    }

    Ok(())
}
