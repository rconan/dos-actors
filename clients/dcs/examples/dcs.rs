use gmt_dos_clients_dcs::{
    Dcs, Pull, Push,
    mount_trajectory::{MountTrajectory, OcsMountTrajectory},
};
use nanomsg::Socket;

use interface::{Read, Update, Write};

const PULL: &str = "tcp://127.0.0.1:4242";
const PUSH: &str = "tcp://127.0.0.1:4243";

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let mut dcs_pull = Dcs::<Pull, Socket, MountTrajectory>::new(PULL)?;
    let mut dcs_push = Dcs::<Push, Socket, MountTrajectory>::new(PUSH)?;

    dcs_pull.update();
    let data =
        <Dcs<Pull, Socket, MountTrajectory> as Write<OcsMountTrajectory>>::write(&mut dcs_pull);

    <Dcs<Push, Socket, MountTrajectory> as Read<OcsMountTrajectory>>::read(
        &mut dcs_push,
        data.unwrap(),
    );
    dcs_push.update();

    Ok(())
}
