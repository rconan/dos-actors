use gmt_dos_clients_io::gmt_m2::asm::segment::{
    AsmCommand, FluidDampingForces, VoiceCoilsForces, VoiceCoilsMotion,
};
use interface::{Data, Read, Size, Update, Write};
use rayon::prelude::*;

use gmt_m2_ctrl_asm_pid_damping::AsmPidDamping;
use gmt_m2_ctrl_asm_preshape_filter::AsmPreshapeFilter;
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct AsmSegmentInnerController<const ID: u8> {
    n_mode: usize,
    preshape_filter: Vec<AsmPreshapeFilter>,
    pid_fluid_damping: Vec<AsmPidDamping>,
    km: f64,
    kb: f64,
    ks: Option<Vec<f64>>,
}

impl<const ID: u8> AsmSegmentInnerController<ID> {
    pub fn new(n_mode: usize, ks: Option<Vec<f64>>) -> Self {
        let (preshape_filter, pid_fluid_damping): (Vec<_>, Vec<_>) = (0..n_mode)
            .map(|_| (AsmPreshapeFilter::new(), AsmPidDamping::new()))
            .unzip();
        let km = 0.01120;
        let kb = 33.60;
        Self {
            n_mode,
            preshape_filter,
            pid_fluid_damping,
            km,
            kb,
            ks,
        }
    }
}

impl<const ID: u8> Size<VoiceCoilsForces<ID>> for AsmSegmentInnerController<ID> {
    fn len(&self) -> usize {
        675
    }
}
impl<const ID: u8> Size<VoiceCoilsMotion<ID>> for AsmSegmentInnerController<ID> {
    fn len(&self) -> usize {
        675
    }
}
impl<const ID: u8> Size<FluidDampingForces<ID>> for AsmSegmentInnerController<ID> {
    fn len(&self) -> usize {
        675
    }
}

impl<const ID: u8> Update for AsmSegmentInnerController<ID> {
    fn update(&mut self) {
        // ASM pre-shape Bessel filter
        let asm_preshape_bessel_filter =
            self.preshape_filter.par_iter_mut().map(|preshape_filter| {
                // preshape_filter.inputs.AO_cmd = *cmd;
                preshape_filter.step();
                (
                    preshape_filter.outputs.cmd_f,
                    // derivatives weighted sum:
                    self.km * preshape_filter.outputs.cmd_f_ddot // km * second_derivative
                    + self.kb * preshape_filter.outputs.cmd_f_dot, // kp * first_derivative
                )
            });

        if let Some(ks) = self.ks.as_ref() {
            let (filtered, filtered_derivatives): (Vec<_>, Vec<_>) =
                asm_preshape_bessel_filter.unzip();
            ks.par_chunks(self.n_mode)
                // dot product between ks row and filtered command: ksf = ks .* filtered_cmd
                .map(|ks_row| {
                    ks_row
                        .iter()
                        .zip(&filtered)
                        .map(|(k, f)| k * f)
                        .sum::<f64>()
                })
                .zip(filtered_derivatives.into_par_iter())
                // add derivatives
                .map(|(ksf, dd)| ksf + dd) // input: asm_FF
                .zip(&filtered) // input: asm_SP
                .zip(&mut self.pid_fluid_damping)
                // .zip(outputs) // ASM PID-fluid-damping outputs
                .for_each(|((asm_ff, asm_sp), pid_fluid_damping)| {
                    // pid_fluid_damping.inputs.asm_FB = *asm_fb;
                    pid_fluid_damping.inputs.asm_SP = *asm_sp;
                    pid_fluid_damping.inputs.asm_FF = asm_ff;
                    pid_fluid_damping.step();
                });
        } else {
            self.pid_fluid_damping
                .par_iter_mut()
                .zip(asm_preshape_bessel_filter) // inputs: asm_FB, asm_SP & asm_FF
                .for_each(|(pid_fluid_damping, (asm_sp, asm_ff))| {
                    // pid_fluid_damping.inputs.asm_FB = *asm_fb;
                    pid_fluid_damping.inputs.asm_SP = asm_sp;
                    pid_fluid_damping.inputs.asm_FF = asm_ff;
                    pid_fluid_damping.step();
                });
        }
    }
}

impl<const ID: u8> Read<AsmCommand<ID>> for AsmSegmentInnerController<ID> {
    fn read(&mut self, data: Data<AsmCommand<ID>>) {
        self.preshape_filter
            .iter_mut()
            .zip(&**data)
            .for_each(|(preshape_filter, data)| preshape_filter.inputs.AO_cmd = *data);
    }
}

impl<const ID: u8> Read<VoiceCoilsMotion<ID>> for AsmSegmentInnerController<ID> {
    fn read(&mut self, data: Data<VoiceCoilsMotion<ID>>) {
        self.pid_fluid_damping
            .iter_mut()
            .zip(&**data)
            .for_each(|(pid_fluid_damping, data)| pid_fluid_damping.inputs.asm_FB = *data);
    }
}

impl<const ID: u8> Write<VoiceCoilsForces<ID>> for AsmSegmentInnerController<ID> {
    fn write(&mut self) -> Option<Data<VoiceCoilsForces<ID>>> {
        let modal_forces = self
            .pid_fluid_damping
            .iter()
            .map(|pid_fluid_damping| pid_fluid_damping.outputs.asm_U);
        Some(Data::new(modal_forces.collect()))
    }
}
impl<const ID: u8> Write<FluidDampingForces<ID>> for AsmSegmentInnerController<ID> {
    fn write(&mut self) -> Option<Data<FluidDampingForces<ID>>> {
        let fluid_damping = self
            .pid_fluid_damping
            .iter()
            .map(|pid_fluid_damping| pid_fluid_damping.outputs.asm_Fd);
        Some(Data::new(fluid_damping.collect()))
    }
}

/* #[cfg(test)]
mod tests {
    use std::{path::Path, time::Instant};

    use crate::ASMS;

    use super::*;
    use gmt_dos_actors::system::Sys;
    use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix, Model, Switch};
    use gmt_dos_clients_io::gmt_m2::asm::{
        M2ASMAsmCommand, M2ASMFluidDampingForces, M2ASMVoiceCoilsForces, M2ASMVoiceCoilsMotion,
    };
    use matio_rs::MatFile;
    use nalgebra as na;
    use nanorand::buffer;

    const ATOL: f64 = 1e-10;

    //cargo test --release --package gmt_dos-clients_m2-ctrl --lib --features serde,polars -- actors_interface::tests::zonal_controller --exact --nocapture
    #[test]
    fn zonal_controller() -> std::result::Result<(), Box<dyn std::error::Error>> {
        let mut fem = gmt_fem::FEM::from_env().unwrap();
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);

        const SID: u8 = 1;
        let vc_d2f = fem
            .switch_inputs_by_name(vec![format!("MC_M2_S{SID}_VC_delta_F")], Switch::On)
            .and_then(|fem| {
                fem.switch_outputs_by_name(vec![format!("MC_M2_S{SID}_VC_delta_D")], Switch::On)
            })
            .map(|fem| {
                fem.reduced_static_gain()
                    .unwrap_or_else(|| fem.static_gain())
            })?
            .try_inverse()
            .unwrap();
        println!("{:?}", vc_d2f.shape());

        fem.switch_inputs(Switch::On, None)
            .switch_outputs(Switch::On, None);

        let mut plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(8e3)
            .proportional_damping(2. / 100.)
            // .truncate_hankel_singular_values(1.531e-3)
            // .hankel_frequency_lower_bound(50.)
    /*         .including_asms(Some(sids.clone()),
            Some(asms_calibration.modes(Some(sids.clone()))),
             Some(asms_calibration.modes_t(Some(sids.clone())))
            .expect(r#"expect some transposed modes, found none (have you called "Calibration::transpose_modes"#))? */
            .including_asms(Some(vec![SID]),
            None,
            None)?
            .use_static_gain_compensation()
            .build()?;

        let na = 675;
        let mut asm = AsmSegmentInnerController::<SID>::new(na, Some(vc_d2f.as_slice().to_vec()));

        let mut cmd = vec![0f64; na];
        cmd[0] = 1e-6;
        let mut i = 0;
        let mut step_runtime = 0;
        let err = loop {
            let now = Instant::now();
            <AsmSegmentInnerController<SID> as Read<AsmCommand<SID>>>::read(
                &mut asm,
                cmd.clone().into(),
            );
            <AsmSegmentInnerController<SID> as Update>::update(&mut asm);

            let data =
                <AsmSegmentInnerController<SID> as Write<VoiceCoilsForces<SID>>>::write(&mut asm)
                    .unwrap();
            <DiscreteModalSolver<ExponentialMatrix> as Read<VoiceCoilsForces<SID>>>::read(
                &mut plant, data,
            );

            let data =
                <AsmSegmentInnerController<SID> as Write<FluidDampingForces<SID>>>::write(&mut asm)
                    .unwrap();
            <DiscreteModalSolver<ExponentialMatrix> as Read<FluidDampingForces<SID>>>::read(
                &mut plant, data,
            );

            <DiscreteModalSolver<ExponentialMatrix> as Update>::update(&mut plant);

            let data =
                <DiscreteModalSolver<ExponentialMatrix> as Write<VoiceCoilsMotion<SID>>>::write(
                    &mut plant,
                )
                .unwrap();

            let err = (cmd
                .iter()
                .zip(data.as_slice())
                .map(|(&c, &p)| (c - p).powi(2))
                .sum::<f64>()
                / na as f64)
                .sqrt();
            if err < ATOL || i > 8000 {
                // dbg!(&data);
                break err;
            }
            <AsmSegmentInnerController<SID> as Read<VoiceCoilsMotion<SID>>>::read(&mut asm, data);

            step_runtime += now.elapsed().as_micros();
            i += 1;
        };
        println!(
            "reach commanded position with an error of {:e} in {} steps",
            err, i
        );
        println!("1 STEP in {}micros", step_runtime / (i + 1));
        assert!(err < ATOL);
        Ok(())
    }

    //cargo test --release --package gmt_dos-clients_m2-ctrl --lib --features serde,polars -- actors_interface::tests::modal_controller --exact --nocapture
    #[test]
    fn modal_controller() -> std::result::Result<(), Box<dyn std::error::Error>> {
        const SID: u8 = 1;

        let path = Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
            .join("examples")
            .join("asm-nodes")
            .join("KLmodesGS36p.mat");
        let n_mode = 6;
        let nkl = 500;
        let kl_modes: na::DMatrix<f64> = MatFile::load(path)?
            .var::<String, na::DMatrix<f64>>(format!("KL_{SID}"))?
            .remove_columns(n_mode, nkl - n_mode);
        dbg!(kl_modes.shape());

        let now = Instant::now();
        let mut fem = gmt_fem::FEM::from_env().unwrap();
        println!("FEM loaded in {}ms", now.elapsed().as_millis());

        let now = Instant::now();
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);

        let vc_d2f = fem
            .switch_inputs_by_name(vec![format!("MC_M2_S{SID}_VC_delta_F")], Switch::On)
            .and_then(|fem| {
                fem.switch_outputs_by_name(vec![format!("MC_M2_S{SID}_VC_delta_D")], Switch::On)
            })
            .map(|fem| {
                fem.reduced_static_gain()
                    .unwrap_or_else(|| fem.static_gain())
            })?
            .try_inverse()
            .unwrap();
        println!("{:?}", vc_d2f.shape());

        fem.switch_inputs(Switch::On, None)
            .switch_outputs(Switch::On, None);
        println!("stiffness from FEM in {}ms", now.elapsed().as_millis());

        let now = Instant::now();
        let kl_modes_t = kl_modes.transpose();
        let mut plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(8e3)
            .proportional_damping(2. / 100.)
            // .truncate_hankel_singular_values(1.531e-3)
            // .hankel_frequency_lower_bound(50.)
            .including_asms(
                Some(vec![SID]),
                Some(vec![kl_modes.as_view()]),
                Some(vec![kl_modes_t.as_view()]),
            )?
            /*             .including_asms(Some(vec![SID]),
            None,
            None)? */
            .use_static_gain_compensation()
            .build()?;
        println!("plant build up in {}ms", now.elapsed().as_millis());

        let modal_vc_d2f = &kl_modes_t * vc_d2f * &kl_modes;

        let (na, n_mode) = kl_modes.shape();
        let mut asm =
            AsmSegmentInnerController::<SID>::new(n_mode, Some(modal_vc_d2f.as_slice().to_vec()));

        // let mut kl_coefs = vec![0.; n_mode];
        // kl_coefs[6] = 1e-6;
        // let cmd = { &kl_modes * na::DVector::from_column_slice(&kl_coefs) }
        //     .as_slice()
        //     .to_vec();
        let mut cmd = vec![0f64; n_mode];
        cmd[5] = 1e-6;

        let mut i = 0;
        let mut step_runtime = 0;
        let err = loop {
            let now = Instant::now();
            <AsmSegmentInnerController<SID> as Read<AsmCommand<SID>>>::read(
                &mut asm,
                cmd.clone().into(),
            );
            <AsmSegmentInnerController<SID> as Update>::update(&mut asm);

            let data =
                <AsmSegmentInnerController<SID> as Write<VoiceCoilsForces<SID>>>::write(&mut asm)
                    .unwrap();
            <DiscreteModalSolver<ExponentialMatrix> as Read<VoiceCoilsForces<SID>>>::read(
                &mut plant, data,
            );

            let data =
                <AsmSegmentInnerController<SID> as Write<FluidDampingForces<SID>>>::write(&mut asm)
                    .unwrap();
            <DiscreteModalSolver<ExponentialMatrix> as Read<FluidDampingForces<SID>>>::read(
                &mut plant, data,
            );

            <DiscreteModalSolver<ExponentialMatrix> as Update>::update(&mut plant);

            let data =
                <DiscreteModalSolver<ExponentialMatrix> as Write<VoiceCoilsMotion<SID>>>::write(
                    &mut plant,
                )
                .unwrap();

            // let err = (kl_coefs
            //     .iter()
            //     .zip({ kl_modes.transpose() * na::DVector::from_column_slice(&data) }.as_slice())
            //     // .filter(|(c, _)| c.abs() > 0.)
            //     .map(|(&c, &p)| (c - p).powi(2))
            //     .sum::<f64>()
            //     / n_mode as f64)
            //     .sqrt();
            let err = (cmd
                .iter()
                .zip(data.as_slice())
                .map(|(&c, &p)| (c - p).powi(2))
                .sum::<f64>()
                / n_mode as f64)
                .sqrt();
            if err < ATOL || i > 8000 {
                dbg!(&data);
                break err;
            }
            <AsmSegmentInnerController<SID> as Read<VoiceCoilsMotion<SID>>>::read(&mut asm, data);

            step_runtime += now.elapsed().as_micros();
            i += 1;
        };
        println!(
            "reach commanded position with a relative error of {:e} in {} steps",
            err, i
        );
        println!("1 STEP in {}micros", step_runtime / (i + 1));
        assert!(err < ATOL);
        Ok(())
    }
}
 */