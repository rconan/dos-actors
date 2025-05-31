// - - - Example description - - -
// In this example, ASM modal commands defined in variable asms_mode_cmd_vec are applied to the GMT servomechanisms system (with the M2 axial displacement outputs of the structural dynamics model projected onto the KL modal basis. The modal coefficients of some M2 segments are logged into a parquet file.
/*  * * * Warning * * *
If using a PDR mount design (model before 2023)
$ cargo run --release --example modal-step-servo --features s8000d002ze30 --no-default-features
Otherwise, use:
$ cargo run --release --example modal-step-servo
*/

use std::{env, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::Signals;
use gmt_dos_clients_io::gmt_m2::asm::{M2ASMAsmCommand, segment::FaceSheetFigure};
use gmt_dos_clients_servos::{AsmsServo, GmtM2, GmtServoMechanisms, asms_servo};
//use gmt_dos_clients_arrow;
use gmt_fem::FEM;

use matio_rs::MatFile;
use nalgebra as na;

const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 8000;
    // let sim_duration = 1_usize; // second
    let n_step = 400;

    // $FEM_REPO-related variables
    let fem = FEM::from_env()?;
    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set!");
    let fem_path = Path::new(&fem_var);

    // ASMS modal commands
    let mat_file = MatFile::load(&fem_path.join("KLmodesGS36p90.mat"))?;
    let kl_mat: Vec<na::DMatrix<f64>> = (1..=7)
        .map(|i| mat_file.var(format!("KL_{i}")).unwrap())
        .collect();
    let asms_mode_cmd_vec: Vec<usize> = vec![7, 6, 5, 4, 3, 2, 1];
    let asms_cmd_vec: Vec<_> = kl_mat
        .into_iter()
        .zip(asms_mode_cmd_vec.into_iter()) // Create the tuples (kl_mat[i], asms_mode_cmd_vec[i])
        .flat_map(|(kl_mat, i)| kl_mat.column(i - 1).as_slice().to_vec())
        .collect();
    let asms_cmd: Signals<_> = Signals::from((asms_cmd_vec, n_step));
    //let asms_cmd: Signals<_> = Signals::new(675 * 7, n_step);

    // GMT Servomechanisms system
    let gmt_servos =
        GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
            .asms_servo(
                AsmsServo::new().facesheet(
                    asms_servo::Facesheet::new()
                        .filter_piston_tip_tilt()
                        .transforms(fem_path.join("KLmodesGS36p90.mat")),
                ),
            )
            .build()?;

    actorscript! {
    // 1: setpoint[MountSetPoint] -> {gmt_servos::GmtMount}
    // 1: m1_rbm[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
    // 1: actuators[assembly::M1ActuatorCommandForces] -> {gmt_servos::GmtM1}
    // 1: m2_rbm[M2RigidBodyMotions]-> {gmt_servos::GmtM2Hex}

    1: asms_cmd[M2ASMAsmCommand] -> {gmt_servos::GmtM2}
    1: {gmt_servos::GmtFem}[FaceSheetFigure<1>]${500}
    1: {gmt_servos::GmtFem}[FaceSheetFigure<2>]${500}
    1: {gmt_servos::GmtFem}[FaceSheetFigure<5>]${500}
    1: {gmt_servos::GmtFem}[FaceSheetFigure<6>]${500}
    1: {gmt_servos::GmtFem}[FaceSheetFigure<7>]${500}
    }

    Ok(())
}
