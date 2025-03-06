use gmt_dos_actors::system::{Sys, SystemError};
use gmt_dos_clients_fem::{Model, Switch};
use gmt_fem::FEM;

mod assembly;
mod builder;
pub use assembly::{AsmsInnerControllers, DispatchIn, DispatchOut, ASMS};
pub use builder::AsmsBuilder;

use crate::M2Error;

impl<const R: usize> ASMS<R> {
    /// Creates a new ASMS [builder](AsmsBuilder)
    pub fn new<'a>(fem: &mut FEM) -> Result<AsmsBuilder<'a, R>, M2Error> {
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
    pub fn build(self) -> Result<Sys<ASMS<R>>, SystemError> {
        Ok(Sys::new(ASMS::<R>::try_from(self)?).build()?)
    }
}

impl<'a, const R: usize> TryFrom<AsmsBuilder<'a, R>> for Sys<ASMS<R>> {
    type Error = SystemError;

    fn try_from(builder: AsmsBuilder<'a, R>) -> std::result::Result<Self, Self::Error> {
        builder.build()
    }
}
