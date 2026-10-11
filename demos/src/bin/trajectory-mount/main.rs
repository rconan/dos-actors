use std::{env, fs, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::signals::Signals;
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::outputs::{MCM2Lcl6D, OSSHardpointForce, OSSM1Lcl},
    gmt_m1::{M1RigidBodyMotions, assembly},
    gmt_m2::M2RigidBodyMotions,
    mount::{MountEncoders, MountSetPoint, MountTorques},
    optics::SegmentTipTilt,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_mount::Mount;
use gmt_dos_systems_m1::M1;
use gmt_fem::FEM;

// M1 actuators control loop sampling rate divider
const M1_RATE: usize = 10;
use interface::{Data, Size, TimerMarker, UniqueIdentifier, Update, Write};

// Drive the mount of the FEM, with the M1 control system (hardpoints & actuators), along the ATP
// tracking trajectory
//  * trajectory: `../trajectory-servos-mount/atp_trajectory.csv`
//  * the M1 control system (gmt_dos-systems_m1) holds the M1 segments with zero rigid body
//    motion and actuator force commands
//  * the FEM rigid telescope motion is removed by the FEM kinematic compensation from all the
//    FEM outputs but the mount encoders, including the M1 hardpoints forces
//  * DCGC=1: with the FEM DC gain compensation (default: without)
//  * RIGID_FREQ=f: eigen frequency [Hz] of the 3 mount rigid body modes (default: FEM)
//  * HP_FORCE_COMPENSATION=0: no kinematic compensation of the hardpoint forces (the M1
//    actuators then offload their kinematic part and the M1 segments drift with the azimuth)
//  * M1_FORCE_LOOP=0: without the M1 actuator force loop, the actuator forces are not applied
//  * the mount encoders, the M1 & M2 rigid body motions and the segment tip-tilt from the
//    linear optical model are saved to `model_data_1.parquet` in this folder
// DATA:
//  * FEM 2nd order model: FEM_REPO

/*
MOUNT_MODEL=MOUNT_FDR_1kHz FEM_REPO=... cargo run --release --features gmt_dos-clients_arrow --bin trajectory-mount
SIM_DURATION=60 (optional, in seconds, default: 60), DCGC=1 (optional)
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
    let bin = Path::new(env!("CARGO_MANIFEST_DIR")).join("src").join("bin");
    unsafe {
        env::set_var("DATA_REPO", bin.join("trajectory-mount"));
    }
    // simulation sampling frequency
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency(); // Hz
    // simulation duration
    let sim_duration = env::var("SIM_DURATION")
        .ok()
        .map_or(Ok(60_usize), |s| s.parse())?; // second
    let n_step = sim_sampling_frequency * sim_duration;

    // FEM with the mount, the M1 & M2 rigid body motions and the kinematic compensation
    let dcgc = env::var("DCGC").is_ok_and(|v| v == "1");
    let mut fem = FEM::from_env()?;
    // M1 control system
    let m1 = M1::<M1_RATE>::new(&mut fem)?;
    let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
    let state_space = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .including_m1(Some(sids))?
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .kinematic_compensation();
    // HP_FORCE_COMPENSATION=0: the hardpoint forces are excluded from the kinematic compensation
    let state_space = if env::var("HP_FORCE_COMPENSATION").is_ok_and(|v| v == "0") {
        println!("no kinematic compensation of the hardpoint forces");
        state_space.kinematic_exclusion::<OSSHardpointForce>()
    } else {
        state_space
    };
    // RIGID_FREQ (optional): eigen frequency [Hz] of the 3 mount rigid body modes
    let state_space = match env::var("RIGID_FREQ") {
        Ok(f) => {
            let f: f64 = f.parse()?;
            println!("mount rigid body modes eigen frequency set to {f}Hz");
            state_space.eigen_frequencies(vec![(0, f), (1, f), (2, f)])
        }
        Err(_) => state_space,
    };
    let state_space = if dcgc {
        println!("FEM with DC gain compensation");
        state_space.use_static_gain_compensation()
    } else {
        println!("FEM without DC gain compensation");
        state_space
    };
    let fem = state_space.build()?;

    // ATP trajectory setpoints [azimuth, elevation, GIR]
    let setpoint = Trajectory::from_csv(
        bin.join("trajectory-servos-mount").join("atp_trajectory.csv"),
        sim_sampling_frequency,
        n_step,
    )?;
    let mount = Mount::new();
    let n_encoders = <Mount as Size<MountEncoders>>::len(&mount);
    let lom = LinearOpticalModel::new()?;
    // M1 rigid body motions & actuator forces commands
    let m1_rbm = Signals::new(6 * 7, n_step);
    let m1_actuators = Signals::new(6 * 335 + 306, n_step);

    // M1_FORCE_LOOP=0: the M1 actuator forces (load cells offload) are not applied to the FEM
    if env::var("M1_FORCE_LOOP").is_ok_and(|v| v == "0") {
        println!("M1 actuator force loop off");
        actorscript! {
            1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount
            1: m1_rbm[assembly::M1RigidBodyMotions]
                -> {m1}[assembly::M1HardpointsMotion]
                    -> fem[assembly::M1HardpointsForces]! -> {m1}
            1: m1_actuators[assembly::M1ActuatorCommandForces] -> {m1}
            1: fem[MountEncoders]${n_encoders}
            1: fem[M1RigidBodyMotions]$
            1: fem[M2RigidBodyMotions]$
            1: fem[M1RigidBodyMotions]! -> lom
            1: fem[M2RigidBodyMotions]! -> lom
            1: lom[SegmentTipTilt]$
        }
    } else {
        actorscript! {
            1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount
            1: m1_rbm[assembly::M1RigidBodyMotions]
                -> {m1}[assembly::M1HardpointsMotion]
                    -> fem[assembly::M1HardpointsForces]! -> {m1}
            1: m1_actuators[assembly::M1ActuatorCommandForces]
                -> {m1}[assembly::M1ActuatorAppliedForces] -> fem
            1: fem[MountEncoders]${n_encoders}
            1: fem[M1RigidBodyMotions]$
            1: fem[M2RigidBodyMotions]$
            1: fem[M1RigidBodyMotions]! -> lom
            1: fem[M2RigidBodyMotions]! -> lom
            1: lom[SegmentTipTilt]$
        }
    }

    Ok(())
}
