/*!
# ASMS Control Systems

Models of the control systems of the ASMS positioners and voice coil actuators

## Example

```no_run
use gmt_dos_actors::system::Sys;
use gmt_dos_clients_m2_ctrl::AsmsPositioners;
use gmt_dos_systems_m2::ASMS;
use gmt_fem::FEM;

let mut fem = FEM::from_env()?;
let positioners = AsmsPositioners::new(&mut fem)?;
let asms: Sys<ASMS> = ASMS::new(&mut fem)?.build()?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

 */

mod assembly;

#[cfg(feature = "serde")]
pub mod nodes;

// // mod actors_interface;
// mod positioner;
// pub use positioner::AsmsPositioners;

// pub use actors_interface::AsmSegmentInnerController;

// #[cfg(fem)]
// mod calibration;
// #[cfg(fem)]
// pub use calibration::{Calibration, DataSource, SegmentCalibration};

// pub mod preprocessor;
use gmt_dos_actors::system::Sys;
use gmt_dos_clients_fem::{Model, Switch};
use gmt_fem::FEM;
// #[doc(inline)]
// pub use preprocessor::Preprocessor;
#[derive(Debug, thiserror::Error)]
pub enum M2CtrlError {
    #[error("failed to load data from matfile")]
    MatFile(#[from] matio_rs::MatioError),
    #[error("failed to compute the stiffness")]
    Stiffness,
    #[error("FEM error")]
    Fem(#[from] gmt_fem::FemError),
    #[error("expected (file_name, vec[var_name]) data source, found other data source")]
    DataSourceMatFile,
    #[error(transparent)]
    IOError(#[from] std::io::Error),
    #[cfg(feature = "bincode")]
    #[error(transparent)]
    Encode(#[from] bincode::error::EncodeError),
    #[cfg(feature = "bincode")]
    #[error(transparent)]
    Decode(#[from] bincode::error::DecodeError),
    #[error("expected matrix size {0:?}, found {1:?}")]
    MatrixSizeMismatch((usize, usize), (usize, usize)),
    #[error("failed to inverse ASMS stiffness matrices")]
    InverseStiffness,
}
mod builder;
pub use assembly::{DispatchIn, DispatchOut, ASMS};
pub use builder::AsmsBuilder;

impl<const R: usize> ASMS<R> {
    /// Creates a new ASMS [builder](AsmsBuilder)
    pub fn new<'a>(fem: &mut FEM) -> anyhow::Result<AsmsBuilder<'a, R>> {
        let mut vc_f2d = vec![];
        for i in 1..=7 {
            fem.switch_inputs(Switch::Off, None)
                .switch_outputs(Switch::Off, None);

            vc_f2d.push(
                fem.switch_inputs_by_name(vec![format!("MC_M2_S{i}_VC_delta_F")], Switch::On)
                    .and_then(|fem| {
                        fem.switch_outputs_by_name(
                            vec![format!("MC_M2_S{i}_VC_delta_D")],
                            Switch::On,
                        )
                    })
                    .map(|fem| {
                        fem.reduced_static_gain()
                            .unwrap_or_else(|| fem.static_gain())
                    })?,
            );
        }
        fem.switch_inputs(Switch::On, None)
            .switch_outputs(Switch::On, None);

        Ok(AsmsBuilder {
            gain: vc_f2d,
            modes: None,
        })
    }
}

impl<'a, const R: usize> AsmsBuilder<'a, R> {
    /// Builds the [ASMS] system
    pub fn build(self) -> anyhow::Result<Sys<ASMS<R>>> {
        Ok(Sys::new(ASMS::<R>::try_from(self)?).build()?)
    }
}

impl<'a, const R: usize> TryFrom<AsmsBuilder<'a, R>> for Sys<ASMS<R>> {
    type Error = anyhow::Error;

    fn try_from(builder: AsmsBuilder<'a, R>) -> std::result::Result<Self, Self::Error> {
        builder.build()
    }
}
