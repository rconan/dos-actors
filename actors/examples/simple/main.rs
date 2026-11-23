use gmt_dos_actors::prelude::*;
use std::ops::Deref;

mod feedback;
mod filter;
mod logging;
mod sampler;
mod signal;

use feedback::DifferentiatorToIntegrator;
use filter::{Filter, FilterToDifferentiator, FilterToSampler, FilterToSink};
use logging::Logging;
use sampler::SamplerToSink;
use signal::{Signal, SignalToFilter};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let n_sample = 2001;
    let sim_sampling_frequency = 1000f64;

    let signal = Signal {
        sampling_frequency: sim_sampling_frequency,
        period: 1f64,
        n_step: n_sample,
        step: 0,
        value: None,
    };
    let logging = Logging::default().into_arcx();

    #[cfg(not(feature = "sampler"))]
    const R: usize = 1;
    #[cfg(feature = "sampler")]
    const R: usize = 50;

    let mut source: Actor<_, 0, 1> = signal.into();
    let mut filter: Actor<_, 1, R> = Filter::default().into();
    let mut sink = Actor::<_, 1, 0>::new(logging.clone());

    #[cfg(not(any(feature = "sampler", feature = "feedback")))]
    let model = {
        source
            .add_output()
            .multiplex(2)
            .build::<SignalToFilter>()
            .into_input(&mut filter)
            .into_input(&mut sink)?;

        filter
            .add_output()
            .build::<FilterToSink>()
            .into_input(&mut sink)?;

        Model::new(vec![Box::new(source), Box::new(filter), Box::new(sink)])
    };

    #[cfg(feature = "sampler")]
    let model = {
        use sampler::Sampler;

        let mut sampler: Actor<_, R, 1> = Sampler::default().into();

        source
            .add_output()
            .build::<SignalToFilter>()
            .into_input(&mut filter)?;

        filter
            .add_output()
            .build::<FilterToSampler>()
            .into_input(&mut sampler)?;

        sampler
            .add_output()
            .build::<SamplerToSink>()
            .into_input(&mut sink)?;

        Model::new(vec![
            Box::new(source),
            Box::new(filter),
            Box::new(sampler),
            Box::new(sink),
        ])
    };

    #[cfg(feature = "feedback")]
    let model = {
        use feedback::{Differentiator, Integrator, IntegratorToDifferentiator};

        let mut compensator: Actor<_, 1, 1> = Differentiator::default().into();
        let mut integrator: Actor<_, 1, 1> = {
            let gain = rand::random_range(0f64..1f64);
            println!("Integrator gain: {:.3}", gain);
            Integrator::new(gain, 1).into()
        };

        source
            .add_output()
            .multiplex(2)
            .build::<SignalToFilter>()
            .into_input(&mut filter)
            .into_input(&mut sink)?;

        filter
            .add_output()
            .build::<FilterToDifferentiator>()
            .into_input(&mut compensator)?;
        compensator
            .add_output()
            .multiplex(2)
            .build::<DifferentiatorToIntegrator>()
            .into_input(&mut integrator)
            .into_input(&mut sink)?;
        integrator
            .add_output()
            .bootstrap()
            .build::<IntegratorToDifferentiator>()
            .into_input(&mut compensator)?;

        Model::new(vec![
            Box::new(source),
            Box::new(filter),
            Box::new(compensator),
            Box::new(integrator),
            Box::new(sink),
        ])
    };

    model
        .name("simple")
        .flowchart()
        .check()?
        .run()
        .wait()
        .await?;

    let _: complot::Plot = (
        logging
            .lock()
            .await
            .deref()
            .chunks(2)
            .enumerate()
            .map(|(i, x)| (i as f64 * sim_sampling_frequency.recip(), x.to_vec())),
        None,
    )
        .into();

    Ok(())
}
