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
