#![allow(unreachable_code)]

use gmt_dos_clients_io::optics::SegmentD21PistonRSS;
use gmt_dos_clients_scope_client::Scope;
use std::env;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env::set_var("SCOPE_SERVER_IP", "100.21.63.28");
    loop {
        Scope::new()
            .name("Segment 21 Differential Piston RSS")
            .signal::<SegmentD21PistonRSS<-9>>()?
            .show();
    }
    Ok(())
}
