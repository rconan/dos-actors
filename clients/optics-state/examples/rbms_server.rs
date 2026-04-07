use gmt_dos_actors::{actor::Terminator, actorscript};
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_io::gmt_m2::M2RigidBodyMotions;
use gmt_dos_clients_optics_state::{M2State, MirrorState, OpticalState, SegmentState};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use interface::Tick;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let os = OpticalState::default().set_zero_point(OpticalState::m2(
        MirrorState::default()
            .set_segment_state(1, SegmentState::rbms(vec![1., 0., 0., 0., 0., 0.])),
    ));
    dbg!(&os);

    let mirror = MirrorState::default();

    let mut monitor = Monitor::new();

    let rbms_server = Scope::<M2RigidBodyMotions>::builder(&mut monitor).build()?;

    let timer: Timer = Timer::new(100);
    actorscript!(
        1: timer[Tick] -> os[M2State] -> mirror[M2RigidBodyMotions] -> rbms_server
    );

    monitor.await?;
    Ok(())
}
