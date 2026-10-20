use std::{env, fs, path::Path};

mod feedforward;
use feedforward::SpringFeedForward;

#[cfg(feature = "gmt_dos-clients_crseo")]
mod gir_rotation;
#[cfg(feature = "gmt_dos-clients_crseo")]
mod gmt_pupil;
#[cfg(feature = "gmt_dos-clients_crseo")]
mod pointing;

use gmt_dos_actors::actorscript;
use gmt_dos_clients_io::optics::SegmentTipTilt;
#[cfg(feature = "gmt_dos-clients_crseo")]
use gmt_dos_clients_io::optics::{
    Frame, Host, PointingError, PointingRotationError, TipTilt, Wavefront,
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    mount::{AverageMountEncoders, MountEncoders, MountSetPoint, MountTorquesFeedForward},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_mount::Mount;
use gmt_dos_clients_servos::{GmtFem, GmtMount, GmtServoMechanisms};
use gmt_fem::FEM;
use interface::{Data, Size, TimerMarker, UID, Update, Write};
// use interface::units::Arcsec;

// Drive the mount along the ATP default tracking trajectory
//  * pointing: alt 45deg, az 0deg, rotator 0deg, 2018-01-01T04:00:00 UTC, Las Campanas
//  * 5 minutes of azimuth, elevation & rotator (GIR) samples at 1Hz in `atp_trajectory.csv`,
//    computed with astropy, the rotator following the change in parallactic angle
//  * the setpoints are the trajectory offsets from the initial pointing,
//    linearly interpolated to the mount sampling frequency
//  * the FEM rigid body springs torque is fed forward to the mount drives (`feedforward.rs`)
//  * the rigid telescope motion is removed from all the FEM outputs (but the mount encoders)
//    by the FEM kinematic compensation, the kinematic gain from the mount encoders derived
//    from the FEM DC gain
//  * the mount encoders, the compensated M1 & M2 rigid body motions and the segment tip-tilt
//    from the linear optical model are saved to `data/model_data_1.parquet`
// DATA:
//  * FEM 2nd order model: FEM_REPO

/*
make run (see Makefile for the parameters)
MOUNT_MODEL=MOUNT_FDR_1kHz FEM_REPO=... CUDACXX=/usr/local/cuda/bin/nvcc cargo run --release --features "cuda gmt_dos-clients_arrow" --bin trajectory-servos-mount
SIM_DURATION=10 (optional, in seconds, default: 300)
KINEMATIC_COMPENSATION=0 (optional, FEM without kinematic compensation)
*/

/// Field rotation \[rd\]: the change in parallactic angle that the GIR follows
#[derive(UID)]
pub enum FieldRotation {}

/// Mount setpoints [azimuth, elevation, GIR] in radians, interpolated from 1Hz samples
///
/// The GIR setpoint is the field rotation, also given as [FieldRotation];
/// without GIR tracking ([Trajectory::gir_tracking_off]) the GIR setpoint is 0
/// but the field still rotates
pub struct Trajectory {
    samples: Vec<[f64; 3]>,
    upsampling: usize,
    step: usize,
    n_step: usize,
    gir_tracking: bool,
    // [azimuth, elevation, field rotation] at the current step
    current: Option<[f64; 3]>,
}
impl Trajectory {
    /// Loads the trajectory (in degrees) and removes the initial pointing
    pub fn from_csv(
        path: impl AsRef<Path>,
        upsampling: usize,
        n_step: usize,
    ) -> anyhow::Result<Self> {
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
            gir_tracking: true,
            current: None,
        })
    }
    /// Sets the GIR setpoint to 0, the GIR not following the field rotation
    pub fn gir_tracking_off(mut self) -> Self {
        self.gir_tracking = false;
        self
    }
}
impl TimerMarker for Trajectory {}
impl Update for Trajectory {
    fn update(&mut self) {
        self.current = if self.step < self.n_step {
            let i = self.step / self.upsampling;
            let a = (self.step % self.upsampling) as f64 / self.upsampling as f64;
            self.samples.get(i).map(|s0| {
                let s1 = self.samples.get(i + 1).unwrap_or(s0);
                [0, 1, 2].map(|k| s0[k] + a * (s1[k] - s0[k]))
            })
        } else {
            None
        };
        self.step += 1;
    }
}
impl Write<MountSetPoint> for Trajectory {
    fn write(&mut self) -> Option<Data<MountSetPoint>> {
        let [az, el, gir] = self.current?;
        Some(Data::new(vec![az, el, if self.gir_tracking { gir } else { 0. }]))
    }
}
impl Write<FieldRotation> for Trajectory {
    fn write(&mut self) -> Option<Data<FieldRotation>> {
        let [.., gir] = self.current?;
        Some(Data::new(vec![gir]))
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // the simulation data (parquet files and flowchart) are saved in the `data` folder
    let data_repo = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("bin")
        .join("trajectory-servos-mount")
        .join("data");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }
    // simulation sampling frequency
    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency(); // Hz

    // simulation duration
    let sim_duration = env::var("SIM_DURATION")
        .ok()
        .map_or(Ok(300_usize), |s| s.parse())?; // second

    // simulation discrete time steps #
    let n_step = sim_sampling_frequency * sim_duration;

    let fem = FEM::from_env()?;
    // FEM with the kinematic compensation of the mount rigid body motion, applied to all the
    // FEM outputs but the mount encoders
    // MOUNT DRIVES FEED-FORWARD of the FEM rigid body springs (MOUNT_FEEDFORWARD=0 to disable)
    // * input: MountSetPoint
    // * output: MountTorquesFeedForward (added to the mount drive torques after the saturation)
    let mut ff = SpringFeedForward::from_fem(&fem)?;
    if env::var("MOUNT_FEEDFORWARD").is_ok_and(|v| v == "0") {
        println!("mount drives feed-forward off");
        ff = ff.off();
    } else {
        println!(
            "mount drives feed-forward [az, el, GIR]: {:?}N.m/rad per drive",
            ff.stiffness()
        );
    }
    // KINEMATIC_COMPENSATION=0: FEM without kinematic compensation
    let servos = GmtServoMechanisms::<10, 1>::new(sim_sampling_frequency as f64, fem);
    let servos = if env::var("KINEMATIC_COMPENSATION").is_ok_and(|v| v == "0") {
        println!("FEM without kinematic compensation");
        servos
    } else {
        servos.kinematic_compensation()
    }
    .build()?;

    // [SET POINT]
    // ATP trajectory for the 3 axes of the mount:
    //   * azimuth (channel #0)
    //   * elevation (channel #1)
    //   * GIR (channel #2)
    // * outputs: MountSetPoint, FieldRotation (the GIR setpoint without GIR_TRACKING=0)
    // TRAJECTORY (optional): path to an alternative trajectory file
    // GIR_TRACKING=0 (optional): GIR setpoint set to 0, the field still rotating
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
    let setpoint = Trajectory::from_csv(trajectory, sim_sampling_frequency, n_step)?;
    let setpoint = if env::var("GIR_TRACKING").is_ok_and(|v| v == "0") {
        println!("GIR tracking off");
        setpoint.gir_tracking_off()
    } else {
        setpoint
    };
    // number of mount encoders (logged size of MountEncoders)
    let n_encoders = <Mount as Size<MountEncoders>>::len(&Mount::new());

    // [LOM (Linear Optical Model)](crates.io/crates/gmt_dos-clients_lom)
    // * inputs: M1RigidBodyMotions, M2RigidBodyMotions
    // * output: SegmentTipTilt
    let lom = LinearOpticalModel::new()?;

    #[cfg(not(feature = "gmt_dos-clients_crseo"))]
    actorscript! {
        1: setpoint[MountSetPoint] -> {servos::GmtMount}
        1: setpoint[MountSetPoint] -> ff[MountTorquesFeedForward] -> {servos::GmtMount}
        1: {servos::GmtFem}[MountEncoders]${n_encoders}
        1: {servos::GmtFem}[M1RigidBodyMotions]$
        1: {servos::GmtFem}[M2RigidBodyMotions]$
        1: {servos::GmtFem}[M1RigidBodyMotions]! -> lom
        1: {servos::GmtFem}[M2RigidBodyMotions]! -> lom
        1: lom[SegmentTipTilt]$
    }

    // GIR ROTATION & OPTICAL MODEL (feature gmt_dos-clients_crseo)
    //  * the M1 & M2 rigid body motions are rotated by the GIR angle (`gir_rotation.rs`)
    //  * the crseo optical model ray traces the GMT with the rotated rigid body motions
    //    at 50Hz (PUPIL_SAMPLING, default: 513) for a J band source (1.215micron)
    //    0.25arcsec off-axis along x (OFF_AXIS) and images it on a camera with a pixel scale
    //    of lambda/(5D) binned 2x2 (~3.9mas) over a 256x256 pixel frame (~1arcsec field-of-view)
    //  * the camera is in absolute pointing along the optical axis: the image is offset by
    //    the source direction seen from the instrument
    //  * the telescope pointing error from the mount tracking error (encoders - setpoint),
    //    rotated by the GIR angle, offsets the source direction and the GIR tracking error
    //    rotates it about the optical axis (`pointing.rs`, POINTING_ERROR=0 to disable):
    //    the GIR compensating the field rotation, the image moves only by the pointing error
    //    and by the GIR tracking error x off-axis angle; with GIR_TRACKING=0, the GIR does not
    //    move and the image rotates with the field
    //  * the camera frame, the tip-tilt from the structure deformations and the pointing errors
    //    are saved to `data/model_data_20.parquet`
    #[cfg(feature = "gmt_dos-clients_crseo")]
    {
        use gmt_dos_clients_crseo::{
            OpticalModel,
            crseo::{
                FromBuilder, Source,
                imaging::{Detector, LensletArray},
            },
            sensors::Camera,
        };
        use skyangle::Conversion;
        let pupil_sampling = env::var("PUPIL_SAMPLING")
            .ok()
            .map_or(Ok(513_usize), |s| s.parse())?;
        let rotation = gir_rotation::GirRotation::new();
        // ATP pointing elevation
        let pointing = pointing::Pointing::new(45f64.to_radians());
        let pointing = if env::var("POINTING_ERROR").is_ok_and(|v| v == "0") {
            println!("telescope pointing error off");
            pointing.off()
        } else {
            pointing
        };
        // off-axis source
        let off_axis = env::var("OFF_AXIS")
            .ok()
            .map_or(Ok(0.25_f64), |s| s.parse())?
            .from_arcsec();
        // camera: the pupil sampling is set by the lenslet array (n_px_lenslet+1),
        // the DFT sampling is lambda/(osf.D) and the 512px imagelet is binned to the 256px frame
        let camera = Camera::<1>::builder()
            .lenslet_array(LensletArray::default().n_px_lenslet(pupil_sampling - 1))
            .detector(Detector {
                n_px_framelet: 256,
                n_px_imagelet: Some(512),
                osf: 5,
                noise_specs: None,
            });
        let mut om = OpticalModel::<Camera<1>>::builder()
            .source(
                Source::builder()
                    .band("J")
                    .zenith_azimuth(vec![off_axis as f32], vec![0.]),
            )
            .sensor(camera)
            .build()?;
        // camera pointing along the optical axis (relative to the source)
        om.sensor_pointing(&[(-off_axis, 0.)])?;
        println!("{om}");
        let n_frame = <OpticalModel<Camera<1>> as Size<Frame<Host>>>::len(&om);
        // flowchart image of the optical model: the GMT pupil with the segments of the assembly (ASSEMBLY) highlighted
        let pupil = Path::new(&env::var("DATA_REPO")?).join("gmt_pupil.svg");
        fs::write(&pupil, gmt_pupil::svg())?;
        println!("GMT assembly: {:?}", gmt_pupil::assembly());
        let pupil = pupil.to_string_lossy().into_owned();
        actorscript! {
            #[images(om = pupil)]
            1: setpoint[MountSetPoint]${3} -> {servos::GmtMount}
            1: setpoint[MountSetPoint] -> ff[MountTorquesFeedForward] -> {servos::GmtMount}
            // 1: {servos::GmtFem}[MountEncoders]${n_encoders}
            // 1: {servos::GmtFem}[M1RigidBodyMotions]$
            // 1: {servos::GmtFem}[M2RigidBodyMotions]$
            // 1: {servos::GmtFem}[M1RigidBodyMotions]! -> lom
            // 1: {servos::GmtFem}[M2RigidBodyMotions]! -> lom
            // 1: lom[SegmentTipTilt]$
            1: {servos::GmtFem}[AverageMountEncoders]!$
            1: setpoint[MountSetPoint] -> pointing
            1: setpoint[FieldRotation]${1} -> pointing
            1: {servos::GmtFem}[AverageMountEncoders]! -> pointing
            20: pointing[PointingError]${2} -> om
            20: pointing[PointingRotationError]${1} -> om
            1: {servos::GmtFem}[MountEncoders]! -> rotation
            1: {servos::GmtFem}[M1RigidBodyMotions]! -> rotation
            1: {servos::GmtFem}[M2RigidBodyMotions]! -> rotation
            20: rotation[M1RigidBodyMotions] -> om
            20: rotation[M2RigidBodyMotions] -> om //[Wavefront]$
            // 1000: om[SegmentTipTilt]$
            20: om[Frame<Host>]${n_frame}
            20: om[TipTilt]$
        }
    }

    Ok(())
}
