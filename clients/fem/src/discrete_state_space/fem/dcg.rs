use crate::{
    fem_io::{self, SplitFem},
    solvers::Solver,
    DiscreteStateSpace,
};
use nalgebra as na;

impl<'a, T: Solver + Default> DiscreteStateSpace<'a, T> {
    pub fn dc_gain_compensator(
        &mut self,
        state_space: &[T],
        w: Vec<f64>,
    ) -> Option<na::DMatrix<f64>> {
        let (_w, _n_modes, _zeta, n_io) = self.properties().ok()?;
        let n_modes = state_space.len();
        let q = self
            .fem
            .as_mut()
            .unwrap()
            .static_gain
            .as_ref()
            .map(|x| na::DMatrix::from_row_slice(n_io.1, n_io.0, x));
        let static_gain = self
            .reduce2io(&q.unwrap())
            .expect("Failed to produce FEM static gain");
        let d = na::DMatrix::from_diagonal(&na::DVector::from_row_slice(
            &w.iter()
                .skip(3)
                .take(n_modes - 3)
                .map(|x| x.recip())
                .map(|x| x * x)
                .collect::<Vec<f64>>(),
        ));
        let forces_2_modes = na::DMatrix::from_row_iterator(
            n_modes,
            state_space[0].n_input(),
            state_space.iter().flat_map(|ss| ss.get_b().to_vec()),
        );
        let modes_2_nodes = na::DMatrix::from_iterator(
            state_space[0].n_output(),
            n_modes,
            state_space.iter().flat_map(|ss| ss.get_c().to_vec()),
        );
        let dyn_static_gain = modes_2_nodes.clone().remove_columns(0, 3)
            * d
            * forces_2_modes.clone().remove_rows(0, 3);

        let mut psi_dcg = static_gain - dyn_static_gain;

        let az_torque = self
            .ins
            .iter()
            .find_map(|x| {
                x.as_any()
                    .downcast_ref::<SplitFem<fem_io::actors_inputs::OSSAzDriveTorque>>()
            })
            .map(|x| x.range());
        let az_encoder = self
            .outs
            .iter()
            .find_map(|x| {
                x.as_any()
                    .downcast_ref::<SplitFem<fem_io::actors_outputs::OSSAzEncoderAngle>>()
            })
            .map(|x| x.range());

        let el_torque = self
            .ins
            .iter()
            .find_map(|x| {
                x.as_any()
                    .downcast_ref::<SplitFem<fem_io::actors_inputs::OSSElDriveTorque>>()
            })
            .map(|x| x.range());
        let el_encoder = self
            .outs
            .iter()
            .find_map(|x| {
                x.as_any()
                    .downcast_ref::<SplitFem<fem_io::actors_outputs::OSSElEncoderAngle>>()
            })
            .map(|x| x.range());

        let rot_torque = self
            .ins
            .iter()
            .find_map(|x| {
                x.as_any()
                    .downcast_ref::<SplitFem<fem_io::actors_inputs::OSSRotDriveTorque>>()
            })
            .map(|x| x.range());
        let rot_encoder = self
            .outs
            .iter()
            .find_map(|x| {
                x.as_any()
                    .downcast_ref::<SplitFem<fem_io::actors_outputs::OSSRotEncoderAngle>>()
            })
            .map(|x| x.range());

        #[cfg(not(ground_acceleration))]
        let gnd_acc = vec![];
        #[cfg(ground_acceleration)]
        let gnd_acc = self
            .ins
            .iter()
            .find_map(|x| {
                x.as_any()
                    .downcast_ref::<SplitFem<fem_io::actors_inputs::OSS00GroundAcc>>()
            })
            .map(|x| x.range());

        let input_indices: Vec<_> = az_torque
            .into_iter()
            .chain(el_torque.into_iter())
            .chain(rot_torque.into_iter())
            .chain(gnd_acc.into_iter()) // <-- Crucial for large-mass models
            .flat_map(|x| x.to_owned().collect::<Vec<usize>>())
            .collect();
        let output_indices: Vec<_> = az_encoder
            .into_iter()
            .chain(el_encoder.into_iter())
            .chain(rot_encoder.into_iter())
            .flat_map(|x| x.to_owned().collect::<Vec<usize>>())
            .collect();

        let (n_row, n_col) = psi_dcg.shape();
        for j in input_indices {
            psi_dcg.set_column(j, &na::DVector::<f64>::zeros(n_row));
            log::info!(
                "Removing SGMC from input #{} of {} (all outputs)",
                j + 1,
                n_col
            );
        }
        for i in output_indices {
            psi_dcg.set_row(i, &na::DVector::<f64>::zeros(n_col).transpose());
            //println!("({})",j);
        }

        Some(psi_dcg)
    }
}
