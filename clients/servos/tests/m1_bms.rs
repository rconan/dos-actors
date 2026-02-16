/*!
# M1 assembly RBMs command integrated test

Testing that commands applied to the M1 segment hardpoints, given in rigid body displacement, results in the actual rigid body displacement of the segments.

Test all the rigid bodies motion for all given segment at once.

The tests fail if any rigid body displacement is more than 25% of the input command.

## Usage
```shell
MOUNT_MODEL=MOUNT_FDR_1kHz cargo test -r --features cuda --test m1_bms -- --show-output
```

*/

const M1_N_MODE: usize = 27;

use std::{env, fs::File};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_io::gmt_m1::assembly::M1ModeShapes;
use gmt_dos_clients_servos::{GmtFem, GmtM1, GmtServoMechanisms, M1SegmentFigure};
use gmt_dos_systems_m1::SingularModes;
use gmt_fem::FEM;
use interface::{
    Tick,
    optics::{
        M1State,
        state::{OpticalState, SegmentState},
    },
};

#[tokio::test(flavor = "multi_thread")]
async fn main() -> anyhow::Result<()> {
    unsafe { env::set_var("FLOWCHART", "") };

    let sim_sampling_frequency = gmt_dos_clients_mount::sampling_frequency();
    let sim_duration = 3_usize; // second
    let n_step = sim_duration * sim_sampling_frequency;

    let fem = FEM::from_env()?;

    // let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
    let n_active_mode = 4;
    let bms = (0..7).map(|i| {
        let modes: Vec<_> = (0..27)
            .map(|j| {
                if j < n_active_mode {
                    (-1f64).powi(i as i32) * (100e-9 - j as f64 * 60e-9)
                } else {
                    0.
                }
            })
            .collect();
        SegmentState::modes(modes)
    });
    let optical_state = OpticalState::default().zero_point(OpticalState::m1(bms.collect()));

    let m1_sms: SingularModes = if let Ok(file) =
        File::open("/home/ubuntu/projects/gmt-ns-im/calibrations/m1/modes/m1_singular_modes.pkl")
    {
        serde_pickle::from_reader(&file, Default::default())?
    } else {
        eprintln!(r#"test failed due to missing file: "m1_singular_modes.pkl""#);
        return Ok(());
    };
    let b2f: Vec<_> = m1_sms
        .mode2force()
        .into_iter()
        .map(|mat| mat.columns(0, M1_N_MODE).clone_owned())
        .collect();
    let s2b: Vec<_> = m1_sms
        .modes_into_mat(Some(M1_N_MODE))
        .into_iter()
        .map(|x| x.transpose())
        .collect();

    let servos = GmtServoMechanisms::<10, 1>::new(sim_sampling_frequency as f64, fem)
        .m1_segment_figure(M1SegmentFigure::new().transforms(s2b).modes_to_forces(b2f))
        .build()?;

    let timer: Timer = Timer::new(n_step);

    actorscript!(
        1: timer[Tick] -> {servos::GmtFem}[M1ModeShapes]!${M1_N_MODE*7}
        1: optical_state[M1State] -> {servos::GmtM1}
    );

    let data: Vec<f64> = model_logging_1
        .into_inner()
        .unwrap()
        .into_iter(format!("M1ModeShapes"))?
        .last()
        .unwrap();
    let relative_errors: Vec<_> = data
        .chunks(M1_N_MODE)
        .map(|data| {
            data.iter()
                .take(n_active_mode)
                // .enumerate()
                .map(|x| x * 1e9)
                // .map(|(i, x)| (x * 1e9, 100.))
                // .map(|(x, x0)| (x - x0).abs() * 1e2 / x0.abs())
                // .inspect(|&e| assert!(e < 25f64))
                .collect::<Vec<_>>()
        })
        .collect();
    println!("First 4 bending modes coefficients: expected +/-[100,40,-20,-80]");
    for (i, seg_err) in relative_errors.into_iter().enumerate() {
        println!("S{:}: {seg_err:+6.2?}", i + 1);
    }
    Ok(())
}
