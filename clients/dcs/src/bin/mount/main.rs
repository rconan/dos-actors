use gmt_dos_actors::actorscript;
use gmt_dos_clients::{foh::FirstOrderHold, sampler::Sampler, timer::Timer};
use gmt_dos_clients_dcs::{
    Dcs, Pull, Push,
    mount_trajectory::{
        ImMountTrajectory, MountTrajectory, OcsMountTrajectory, RelativeMountTrajectory,
    },
};
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::outputs::{MCM2Lcl6D, OSSM1Lcl},
    mount::{AverageMountEncoders, MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_mount::Mount;
use interface::{Tick, UID};
use nanomsg::Socket;

const PULL: &str = "tcp://127.0.0.1:4242";
const PUSH: &str = "tcp://127.0.0.1:4243";

#[derive(UID)]
pub enum ScopeAverageMountEncoders {}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .format_level(false)
        .format_timestamp_millis()
        .init();

    let fem = DiscreteModalSolver::<ExponentialMatrix>::try_from("gmt-fem.bin")?;

    let mount = Mount::new();

    let dcs_pull = Dcs::<Pull, Socket, MountTrajectory>::new(PULL)?;
    let dcs_push = Dcs::<Push, Socket, MountTrajectory>::new(PUSH)?;

    let rmt = RelativeMountTrajectory::default();

    // let sampler = Sampler::default();
    let sampler = Sampler::default();

    let foh = FirstOrderHold::<f64, 50, 1>::new();

    let metronome: Timer = Timer::new(100);

    actorscript!(
        #[model(name = mount)]
        #[labels(fem = "60deg EL\n0deg AZ",
            mount = "Mount Controller\n& Driver Models",
            dcs_pull = "From OCS", dcs_push = "To OCS",
            // scope_mountsetpoint = "Scope",// scope_scopeaveragemountencoders = "Scope",
            rmt = "Relative\nTrajectory",
            foh = "First-Order\nHold",
             sampler = "1:50"
        )]
        #[images(fem = "gmt-pretty4.png")]

        50: metronome[Tick] -> dcs_pull[OcsMountTrajectory]${3} -> foh
        1: foh[OcsMountTrajectory]${3} ->  rmt[MountSetPoint]~
             -> mount[MountTorques] -> fem[MountEncoders]! -> mount
        1: fem[AverageMountEncoders]!~ -> rmt
        1: rmt[ImMountTrajectory] -> sampler
        50: sampler[ImMountTrajectory]${3} -> dcs_push
    );

    Ok(())
}
