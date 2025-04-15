pub mod agws;
pub mod builder;
pub mod kernels;
#[doc(inline)]
pub use agws::Agws;
#[doc(inline)]
pub use builder::AgwsBuilder;

#[cfg(test)]
mod test {

    use builder::shack_hartmann::ShackHartmannBuilder;

    use gmt_dos_clients_io::optics::{Frame, Host};
    use interface::{Update, Write};
    use skyangle::Conversion;

    use super::*;

    #[tokio::test]
    async fn sh24() {
        let mut agws = Agws::<1, 1>::builder()
            .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
            .build()
            .unwrap();
        println!("{}", agws.sh24.client().lock().await);
        agws.sh24_pointing((4f64.from_arcmin() + 4000f64.from_mas(), 180f64.to_radians()))
            .await;
        agws.sh24.client().lock().await.update();
        let frame =
            <_ as Write<Frame<Host>>>::write(&mut *agws.sh24.client().lock().await).unwrap();
        let n_px = 24 * 12;
        let _: complot::Heatmap = ((frame.as_arc().as_slice(), (n_px, n_px)), None).into();
    }
}
