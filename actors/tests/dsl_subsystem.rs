//! Subsystems declared in actorscript with #[system(...)] and #[sys(...)]
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    gain::Gain,
    logging::Logging,
    signals::{Signal, Signals},
};
use interface::UID;

#[derive(UID)]
pub enum A {}
#[derive(UID)]
pub enum B {}
#[derive(UID)]
pub enum C {}

#[tokio::test]
async fn dsl_subsystem() -> anyhow::Result<()> {
    let n = 100;
    let setpoint: Signals = Signals::new(1, n).channels(Signal::Constant(1.0));
    let pre = Gain::new(vec![3.0]);
    let ff = Gain::new(vec![2.0]);
    let plant = Logging::<f64>::new(1);

    actorscript! {
        #[model(name = dsl_subsystem)]
        #[system(name = sds, label = "SDS")]
        1: setpoint[A] -> pre
        #[sys(sds)]
        1: pre[C] -> ff[B] -> plant
    };

    let log = plant.lock().await;
    assert_eq!(log.len(), n);
    assert!(log.as_slice().iter().all(|x| *x == 6.0));
    Ok(())
}

#[tokio::test]
async fn dsl_subsystem_first_flow() -> anyhow::Result<()> {
    let n = 100;
    let setpoint: Signals = Signals::new(1, n).channels(Signal::Constant(1.0));
    let ff = Gain::new(vec![2.0]);
    let plant = Logging::<f64>::new(1);

    actorscript! {
        #[model(name = dsl_subsystem_first_flow)]
        #[system(name = sds)]
        #[sys(sds)]
        1: ff[B] -> plant
        1: setpoint[A] -> ff
    };

    let log = plant.lock().await;
    assert_eq!(log.len(), n);
    assert!(log.as_slice().iter().all(|x| *x == 2.0));
    Ok(())
}

#[tokio::test]
async fn dsl_subsystem_rate_transition() -> anyhow::Result<()> {
    let n = 100;
    let setpoint: Signals = Signals::new(1, n).channels(Signal::Constant(1.0));
    // the output of slow is sampled at 1/10th of the nominal rate and upsampled into plant
    let slow: Signals = Signals::new(1, n / 10).channels(Signal::Constant(5.0));
    let plant = Logging::<f64>::new(2);

    actorscript! {
        #[model(name = dsl_subsystem_rate_transition)]
        #[system(name = sds)]
        1: setpoint[A] -> plant
        #[sys(sds)]
        10: slow[C] -> plant
    };

    let log = plant.lock().await;
    assert_eq!(log.len(), n);
    assert!(log.as_slice().chunks(2).all(|x| x.iter().sum::<f64>() == 6.0));
    Ok(())
}

mod inner {
    use std::{fmt::Display, sync::Arc};

    use gmt_dos_actors::{
        actor::{Actor, PlainActor},
        framework::{
            model::{Check, Task},
            network::{AddActorOutput, AddOuput, TryIntoInputs},
        },
        prelude::FlowChart,
        system::{System, SystemError, SystemInput},
    };
    use gmt_dos_clients::{gain::Gain, logging::Logging};
    use tokio::sync::Mutex;

    use super::B;

    pub type Gw = Gain<f64>;

    /// A system with an input gateway feeding an internal logger
    #[derive(Clone)]
    pub struct Inner {
        gw: Actor<Gw, 1, 1>,
        log: Actor<Logging<f64>, 1, 0>,
    }
    impl Inner {
        pub fn new(log: Arc<Mutex<Logging<f64>>>) -> Self {
            Self {
                gw: Gain::new(vec![2.0]).into(),
                log: Actor::new(log),
            }
        }
    }
    impl Display for Inner {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "INNER")
        }
    }
    impl System for Inner {
        fn name(&self) -> String {
            "INNER".to_string()
        }
        fn build(&mut self) -> Result<&mut Self, SystemError> {
            self.gw
                .add_output()
                .build::<B>()
                .into_input(&mut self.log)?;
            Ok(self)
        }
        fn plain(&self) -> PlainActor {
            PlainActor::new(self.name())
                .inputs(PlainActor::from(&self.gw).inputs().unwrap())
                .graph(self.graph())
                .build()
        }
    }
    impl<'a> IntoIterator for &'a Inner {
        type Item = Box<&'a dyn Check>;
        type IntoIter = std::vec::IntoIter<Self::Item>;
        fn into_iter(self) -> Self::IntoIter {
            vec![
                Box::new(&self.gw as &dyn Check),
                Box::new(&self.log as &dyn Check),
            ]
            .into_iter()
        }
    }
    impl IntoIterator for Box<Inner> {
        type Item = Box<dyn Task>;
        type IntoIter = std::vec::IntoIter<Self::Item>;
        fn into_iter(self) -> Self::IntoIter {
            vec![
                Box::new(self.gw) as Box<dyn Task>,
                Box::new(self.log) as Box<dyn Task>,
            ]
            .into_iter()
        }
    }
    impl SystemInput<Gw, 1, 1> for Inner {
        fn input(&mut self) -> &mut Actor<Gw, 1, 1> {
            &mut self.gw
        }
    }
}

#[tokio::test]
async fn dsl_subsystem_with_system_member() -> anyhow::Result<()> {
    use gmt_dos_actors::{ArcMutex, system::Sys};
    use inner::{Gw, Inner};

    let n = 100;
    let setpoint: Signals = Signals::new(1, n).channels(Signal::Constant(1.0));
    let pre = Gain::new(vec![3.0]);
    let log = Logging::<f64>::new(1).into_arcx();
    let inner = Sys::new(Inner::new(log.clone())).build()?;

    actorscript! {
        #[model(name = dsl_subsystem_with_system_member)]
        #[system(name = outer, label = "OUTER")]
        1: setpoint[A] -> pre
        #[sys(outer)]
        1: pre[C] -> {inner::Gw}
    };

    let log = log.lock().await;
    assert_eq!(log.len(), n);
    assert!(log.as_slice().iter().all(|x| *x == 6.0));
    Ok(())
}
