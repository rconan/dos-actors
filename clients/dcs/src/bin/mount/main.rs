use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Sampler, Timer};
use gmt_dos_clients_dcs::{
    mount_trajectory::{
        ImMountTrajectory, MountTrajectory, OcsMountTrajectory, RelativeMountTrajectory,
    },
    Dcs, Pull, Push,
};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::outputs::{MCM2Lcl6D, OSSM1Lcl},
    mount::{AverageMountEncoders, MountEncoders, MountSetPoint, MountTorques},
};
use gmt_dos_clients_mount::Mount;
use interface::{Tick, UID};
use nanomsg::Socket;

const PULL: &str = "tcp://127.0.0.1:5001";
const PUSH: &str = "tcp://127.0.0.1:5002";

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

    let metronome: Timer = Timer::new(100);

    let sampler = Sampler::default();

    actorscript!(
        #[model(name = mount)]
        #[labels(fem = "60deg EL\n0deg AZ",
            mount = "Mount Controller\n& Driver Models",
            dcs_pull = "From OCS", dcs_push = "To OCS",
            scope_mountsetpoint = "Scope", scope_scopeaveragemountencoders = "Scope",
            rmt = "Relative\nTrajectory",
            sampler = "1:50"
        )]
        #[images(fem = "gmt-pretty4.png")]

        50: metronome[Tick] -> dcs_pull[OcsMountTrajectory]${3} -> rmt[MountSetPoint]..~
        50: rmt[ImMountTrajectory]${3} -> dcs_push
        1: rmt[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount
        1: fem[AverageMountEncoders]! -> rmt
        1: fem[AverageMountEncoders]! -> sampler
        50: sampler[ScopeAverageMountEncoders]..~
    );

    Ok(())
}