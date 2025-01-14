/*!
# M2 RBMS

In this example, we move the ASMS S1 reference body by 1 micron along the local z-axis (Tz)
and record the rigid body motions and axial displacements of the ASMS S1 facesheet

Run the example with:

```bash
cargo run --release --example m2-rbms --features s8000d002ze30 --no-default-features
```

and post-process with:

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_parquet("model-data_40.parquet")
plt.figure();plt.plot(np.vstack(df["M2RigidBodyMotions"]));

df = pd.read_parquet("model-data_3999.parquet")
plt.figure();plt.plot(np.vstack(df["FaceSheetFigure#1"])[-1,:],'.');
```
*/

use std::{env, path::Path};

use gmt_dos_actors::{actorscript, framework::model::SystemFlowChart, system::Sys};
use gmt_dos_clients::{Signal, Signals, Tick, Timer};
use gmt_dos_clients_io::{
    gmt_fem::outputs::{
        M1Segment1AxialD, M1Segment2AxialD, M1Segment3AxialD, M1Segment4AxialD, M1Segment5AxialD,
        M1Segment6AxialD, M1Segment7AxialD,
    },
    gmt_m1::{assembly, segment::ModeShapes, M1RigidBodyMotions},
};
use gmt_dos_clients_servos::{asms_servo, GmtFem, GmtM1, GmtServoMechanisms, M1SegmentFigure};
//asms_servo
use gmt_fem::FEM;
use interface::filing::Filing;
use nanorand::{Rng, WyRand};

const ACTUATOR_RATE: usize = 80; //100Hz

#[derive(Debug, Clone)]
struct MyFacesheet;
impl asms_servo::FacesheetOptions for MyFacesheet {
    fn remove_rigid_body_motions(&self) -> bool {
        false
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    env::set_var(
        "DATA_REPO",
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples")
            .join("m1-rbms"),
    );

    let mut rng = WyRand::new();

    let sim_sampling_frequency = 8000;
    let n_step = sim_sampling_frequency * 3;

    let fem = FEM::from_env()?;

    // M2 S1
    let m1_rbm: Signals = (0..42).fold(Signals::new(42, n_step), |signals, i| {
        signals.channel(
            i,
            Signal::Sigmoid {
                amplitude: 1e-6 * (rng.generate::<f64>() * 2. - 1.),
                sampling_frequency_hz: sim_sampling_frequency as f64,
            },
        )
    });
    // let sigmoid = Signal::Sigmoid {
    //     amplitude: 1e-6,
    //     sampling_frequency_hz: sim_sampling_frequency as f64,
    // };
    // let m1_rbm = Signals::new(42, n_step).channel(2, sigmoid);

    // GMT Servo-mechanisms system

    let gmt_servos =
        Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path_or_else("servos.bin", || {
            GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
                // .asms_servo(AsmsServo::new().facesheet(Default::default()))
                .m1_segment_figure(M1SegmentFigure::new())
        })?;

    // let gmt_servos =
    //     GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem)
    //         .asms_servo(
    //             AsmsServo::new()
    //                 .facesheet(asms_servo::Facesheet::new().options(Box::new(MyFacesheet))),
    //         )
    //         .build()?;

    actorscript! {
        1: m1_rbm[assembly::M1RigidBodyMotions] -> {gmt_servos::GmtM1}
    };

    // gmt_servos.to_path("servos.bin")?;
    // let gmt_servos = Sys::<GmtServoMechanisms<ACTUATOR_RATE, 1>>::from_path("servos.bin")?;

    let nope: Timer = Timer::new(0);
    actorscript! {
        #[model(name=m1_rbms)]
        1: nope[Tick] -> {gmt_servos::GmtFem}[M1RigidBodyMotions]!$
        1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<1>]!${602}
        1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<2>]!${602}
        1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<3>]!${602}
        1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<4>]!${602}
        1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<5>]!${602}
        1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<6>]!${602}
        1: nope[Tick] -> {gmt_servos::GmtFem}[ModeShapes<7>]!${579}
    }

    let mut logs = m1_rbms_logging_1.lock().await;
    println!("{logs}");
    for i in 1..=7 {
        let data: Vec<f64> = logs.iter(format!("ModeShapes<{i}>"))?.last().unwrap();
        let n = data.len() as f64;
        let (mut var, mut mean) = data
            .into_iter()
            .fold((0f64, 0f64), |(mut var, mut mean), x| {
                var += x * x;
                mean += x;
                (var, mean)
            });
        mean /= n;
        var /= n;
        var -= mean * mean;
        println!(
            "ASM#{i}: [Mean, Std]: {:+0.3?} nm",
            (mean * 1e9, var.sqrt() * 1e9)
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use geotrans::{Quaternion, Vector};
    use gmt_dos_clients_arrow::{Arrow, ArrowBuilder};
    use gmt_dos_clients_io::gmt_m2::asm::segment::FaceSheetFigure;
    use interface::{Entry, Read};
    use matio_rs::MatFile;
    use rayon::prelude::*;
    use std::{env, error::Error, path::Path, time::Instant};

    pub fn rbm_removal(rbm: &[f64], nodes: &mut [f64], figure: &[f64]) -> Vec<f64> {
        let tz = rbm[2];
        let q = Quaternion::unit(rbm[5], Vector::k())
            * Quaternion::unit(rbm[4], Vector::j())
            * Quaternion::unit(rbm[3], Vector::i());
        nodes
            .chunks_mut(3)
            .zip(figure)
            .map(|(u, dz)| {
                u[2] = dz - tz;
                let p: Quaternion = From::<&[f64]>::from(u);
                let pp = q.complex_conjugate() * p * &q;
                let v: Vec<f64> = pp.vector_as_slice().to_vec();
                v[2]
            })
            .collect()
    }

    #[test]
    fn cs_transform() -> Result<(), Box<dyn Error>> {
        let data_repo = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples")
            .join("m2-rbms");
        env::set_var("DATA_REPO", &data_repo);

        let mut nodes: Vec<f64> = MatFile::load(data_repo.join("M2S1nodes.mat"))?.var("nodes")?;
        dbg!(nodes.len());

        let mut data = Arrow::from_parquet(data_repo.join("m2_rbms-Rx_1.parquet"))?;
        println!("{data}");
        let rbms: Vec<f64> = data.iter("M2RigidBodyMotions")?.last().unwrap();
        let figure: Vec<f64> = data.iter("FaceSheetFigure#1")?.last().unwrap();

        let now = Instant::now();
        let data = rbm_removal(&rbms[..6], &mut nodes, &figure);
        println!("Elapsed time: {:?}", now.elapsed());

        let mut arrow = ArrowBuilder::new(1).build();
        <Arrow as Entry<FaceSheetFigure<1>>>::entry(&mut arrow, 675);
        <Arrow as Read<FaceSheetFigure<1>>>::read(&mut arrow, data.into());

        Ok(())
    }
}
