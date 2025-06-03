// - - - Example description - - -
// This example builds the GMT servomechanisms system with the M2 axial displacements outputs of the structural dynamics model (namely, "M2_segment_<i>_axial_d") projected onto the KL modal basis, which is provided as an argument to the GmtServoMechanisms' crate builder.

use std::{env, path::Path};

use gmt_dos_clients_servos::{AsmsServo, GmtServoMechanisms, asms_servo};
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80;

// RUST_LOG=info cargo run --release --example karhunen-loeve

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 8000;
    let fem = FEM::from_env()?;

    let fem_var = env::var("FEM_REPO").expect("`FEM_REPO` is not set");
    let fem_path = Path::new(&fem_var);

    let _gmt_servos =
        GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
            .asms_servo(
                AsmsServo::new().facesheet(
                    asms_servo::Facesheet::new()
                        .filter_piston_tip_tilt()
                        .transforms(fem_path.join("KLmodesGS36p90.mat")),
                ),
            )
            .build()?;
    Ok(())
}
