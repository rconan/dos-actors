use arrow::Arrow;
use crseo::FromBuilder;
use dos_actors::prelude::*;
use gmt_dos_clients_crseo::{OpticalModel, OpticalModelOptions, Wavefront, WfeRms};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    std::env::set_var(
        "DATA_REPO",
        std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
            .join("examples")
            .join("atmosphere"),
    );

    let atm_sampling_frequency = 50usize; // Hz
    let n_sample = 100;

    let atm_model = {
        let mut atm: Actor<_> = (
            OpticalModel::builder()
                .source(crseo::Source::builder().pupil_sampling(769))
                .options(vec![OpticalModelOptions::Atmosphere {
                    builder: crseo::Atmosphere::builder(),
                    time_step: (atm_sampling_frequency as f64).recip(),
                }])
                .build()?,
            "Atmosphere",
        )
            .into();
        let mut sink: Terminator<_> = Arrow::builder(n_sample)
            .filename("atmosphere")
            .build()
            .into();
        let mut timer: Initiator<_> = Timer::new(n_sample).into();

        timer.add_output().build::<Tick>().into_input(&mut atm);
        atm.add_output().build::<WfeRms>().log(&mut sink).await;
        atm.add_output().build::<Wavefront>().log(&mut sink).await;

        Model::new(vec![Box::new(timer), Box::new(atm), Box::new(sink)])
            .name("atmosphere")
            .check()?
            .flowchart()
            .run()
    };

    let free_atm = {
        let mut atm: Actor<_> = (
            OpticalModel::builder()
                .source(crseo::Source::builder().pupil_sampling(769))
                .options(vec![OpticalModelOptions::Atmosphere {
                    builder: crseo::Atmosphere::builder().remove_turbulence_layer(0),
                    time_step: (atm_sampling_frequency as f64).recip(),
                }])
                .build()?,
            "Free Atmosphere ",
        )
            .into();
        let mut sink: Terminator<_> = Arrow::builder(n_sample)
            .filename("free-atmosphere")
            .build()
            .into();
        let mut timer: Initiator<_> = Timer::new(n_sample).into();

        timer.add_output().build::<Tick>().into_input(&mut atm);
        atm.add_output().build::<WfeRms>().log(&mut sink).await;
        atm.add_output().build::<Wavefront>().log(&mut sink).await;

        Model::new(vec![Box::new(timer), Box::new(atm), Box::new(sink)])
            .name("free-atmosphere")
            .check()?
            .flowchart()
            .run()
    };

    let dome_seeing_free_atm = {
        let free_atm = OpticalModelOptions::Atmosphere {
            builder: crseo::Atmosphere::builder().remove_turbulence_layer(0),
            time_step: (atm_sampling_frequency as f64).recip(),
        };
        let dome_seeing = OpticalModelOptions::DomeSeeing {
            cfd_case: "/fsx/CASES/zen30az000_OS7".to_string(),
            upsampling_rate: atm_sampling_frequency / 5,
        };
        let mut atm: Actor<_> = (
            OpticalModel::builder()
                .source(crseo::Source::builder().pupil_sampling(769))
                .options(vec![free_atm, dome_seeing])
                .build()?,
            "Dome Seeing + Free Atmosphere",
        )
            .into();
        let mut sink: Terminator<_> = Arrow::builder(n_sample)
            .filename("dome-seeing_free-atmosphere")
            .build()
            .into();
        let mut timer: Initiator<_> = Timer::new(n_sample).into();

        timer.add_output().build::<Tick>().into_input(&mut atm);
        atm.add_output().build::<WfeRms>().log(&mut sink).await;
        atm.add_output().build::<Wavefront>().log(&mut sink).await;

        Model::new(vec![Box::new(timer), Box::new(atm), Box::new(sink)])
            .name("dome-seeing_free-atmosphere")
            .check()?
            .flowchart()
            .run()
    };

    atm_model.wait().await?;
    free_atm.wait().await?;
    dome_seeing_free_atm.wait().await?;

    Ok(())
}
