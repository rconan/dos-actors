/*!
# GMT AGWS Integrated Model

A [gmt_dos-actors] [system] that models the optical paths from the AGWS guide stars through the GMT to the Shack-Hartmann wavefront sensors (WFS) 48x48 and 24x24.
The model includes [kernels] that compute the WFS centroids from the frames of the cameras, transform the centroids in commmands for M1 and M2 segments and apply temporal filters to the commands.

## Examples

 1. Building an optical model for the 3 AGWS 48x48 Shack-Hartmann wavefront sensors:
```
use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    sensors::builders::CameraBuilder,
    calibration::Reconstructor};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;

let sh48 = ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src();
let omb = OpticalModelBuilder::<CameraBuilder>::from(&sh48);
println!("{}", omb.clone().build()?);
# Ok::<(),Box<dyn std::error::Error>>(())
```

 2. Building an optical model for the AGWS 24x24 Shack-Hartmann wavefront sensor:
```
use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    sensors::builders::CameraBuilder,
    calibration::Reconstructor};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;

let sh24 = ShackHartmannBuilder::<Reconstructor>::sh24().use_calibration_src();
let omb = OpticalModelBuilder::<CameraBuilder>::from(&sh24);
println!("{}", omb.clone().build()?);
# Ok::<(),Box<dyn std::error::Error>>(())
```
 3. Building the default AGWS systems: one 24x24 and three 48x48 SH WFSs
 ```
use gmt_dos_systems_agws::Agws;

let agws = Agws::<5000,5>::builder().build()?;
# Ok::<(),Box<dyn std::error::Error>>(())
 ```

[gmt_dos-actors]: https://docs.rs/gmt_dos-actors
[system]: https://docs.rs/gmt_dos-actors/system
*/

pub mod agws;
pub mod builder;
pub mod kernels;
#[cfg(feature = "qp")]
pub mod qp;
#[doc(inline)]
pub use agws::Agws;
#[doc(inline)]
pub use builder::AgwsBuilder;

#[cfg(test)]
mod tests {

    use builder::shack_hartmann::ShackHartmannBuilder;

    use gmt_dos_clients_io::optics::{Frame, Host};
    use interface::{Update, Write};
    use skyangle::Conversion;

    #[cfg(feature = "qp")]
    use crate::qp::ActiveOptics;

    use super::*;

    #[test]
    fn builder() {
        let _ = Agws::<1, 1>::builder().build();
    }

    #[cfg(feature = "qp")]
    #[test]
    fn aco_builder() {
        let _ = Agws::<1, 1, ActiveOptics<1, 41, 41, 27, 271>>::builder().build();
    }

    #[tokio::test(flavor = "multi_thread")]
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
