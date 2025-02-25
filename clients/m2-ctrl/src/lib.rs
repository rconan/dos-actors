/*!
# ASMS Control Systems

Models of the control systems of the ASMS positioners and voice coil actuators

## Example

```no_run
use gmt_dos_actors::system::Sys;
use gmt_dos_clients_m2_ctrl::AsmsPositioners;
use gmt_fem::FEM;

let mut fem = FEM::from_env()?;
let positioners = AsmsPositioners::new(&mut fem)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

 */

#[cfg(feature = "serde")]
pub mod nodes;

mod actors_interface;
mod positioner;
pub use positioner::AsmsPositioners;

pub use actors_interface::AsmSegmentInnerController;

pub mod preprocessor;
pub use preprocessor::Preprocessor;
