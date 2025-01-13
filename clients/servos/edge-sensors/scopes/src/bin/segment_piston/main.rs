#![allow(unreachable_code)]

use gmt_dos_clients_io::optics::SegmentPiston;
use gmt_dos_clients_scope_client::Scope;
use io::{M1SegmentPiston, M2RBSegmentPiston, M2SegmentMeanActuator, M2SegmentPiston};
use std::env;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env::set_var("SCOPE_SERVER_IP", "44.235.124.92");
    loop {
        if let Ok(scope) = env::var("SCOPE") {
            match scope.as_str() {
                "M1" => Scope::new()
                    .name("Segment Piston from M1 RBM")
                    .signal::<M1SegmentPiston>()?
                    .show(),
                "M2" => Scope::new()
                    .name("Segment Piston from M2 RBM")
                    .signal::<M2SegmentPiston>()?
                    .show(),
                "M2RB" => Scope::new()
                    .name("Segment Piston from M2 Reference Body RBM")
                    .signal::<M2RBSegmentPiston>()?
                    .show(),
                "M2S" => Scope::new()
                    .name("Segment Piston from M2 Shell Voice Coils")
                    .signal::<M2SegmentMeanActuator>()?
                    .show(),
                other => panic!("expected M1, M2, M2RB or M2S, found {other}"),
            }
        } else {
            Scope::new()
                .name("Segment Piston from M1 & M2 RBM")
                .signal::<SegmentPiston<-9>>()?
                .show();
        };
    }
    Ok(())
}
