use gmt_dos_clients_io::optics::{SegmentPiston, SegmentTipTilt};
use gmt_dos_clients_scope_client::GridScope;
use interface::units::Mas;
use io::{M2S1Tz, M2S1VcAsTz};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    GridScope::new((1, 3))
        .pin::<SegmentPiston<-9>>((0, 0))?
        .pin::<Mas<SegmentTipTilt>>((0, 1))?
        .pin::<M2S1VcAsTz>((0, 2))?
        .pin::<M2S1Tz>((0, 2))?
        .show();
    Ok(())
}
