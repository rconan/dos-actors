use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    integrator::Integrator, logging::Logging, signals::{Signal, Signals},
};
use interface::{Data, Read, Update, Write, UID};

// ANCHOR: io
#[derive(UID)]
#[uid(port = 5001)]
enum U {}
#[derive(UID)]
#[uid(port = 5002)]
enum Y {}
#[derive(UID)]
enum E {}
// ANCHOR_END: io

// ANCHOR: sum_client
pub struct Sum {
    left: Data<U>,
    right: Data<Y>,
}
impl Default for Sum {
    fn default() -> Self {
        Self {
            left: Data::new(vec![]),
            right: Data::new(vec![]),
        }
    }
}
impl Update for Sum {}
impl Read<U> for Sum {
    fn read(&mut self, data: Data<U>) {
        self.left = data.clone();
    }
}
impl Read<Y> for Sum {
    fn read(&mut self, data: Data<Y>) {
        self.right = data.clone();
    }
}
impl Write<E> for Sum {
    fn write(&mut self) -> Option<Data<E>> {
        Some(Data::new(
            self.left
                .iter()
                .zip(self.right.iter())
                .map(|(l, r)| l + r)
                .collect(),
        ))
    }
}
// ANCHOR_END: sum_client

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .format_timestamp(None)
        .format_target(false)
        .init();

    // ANCHOR: actorscript
    let n_step = 100;
    let signal = Signals::new(1, n_step).channel(0, Signal::Constant(1f64));
    let sum = Sum::default();
    let integrator = Integrator::new(1).gain(0.5);
    let logging = Logging::<f64>::new(3);

    actorscript!(
        #[model(name = feedback)]
        #[labels(sum = "+")]
        1: signal[U] -> sum[E] -> integrator[Y]! -> sum
        1: signal[U] -> logging
        1: sum[E] -> logging
        1: integrator[Y]! -> logging
    );
    // ANCHOR_END: actorscript

    Ok(())
}
