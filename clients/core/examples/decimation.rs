use gmt_dos_actors::actorscript;
use gmt_dos_clients::{average, print::Print, Average, Sampler, Signal, Signals};
use interface::UID;

#[derive(UID)]
enum U {}

#[derive(UID)]
enum A {}

#[derive(UID)]
enum D {}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let n_step = 10;
    let unit = Signals::new(1, n_step).channel(0, Signal::Ramp { a: 1., b: 0. });
    let print = Print::new(3);
    let average = Average::new(1);
    let decimator = Sampler::default();

    actorscript!(
        1: unit[U] -> print
        1: unit[U] -> average
        3: average[A]! -> print
        1: unit[U] -> decimator
        3: decimator[D] -> print
    );

    Ok(())
}
