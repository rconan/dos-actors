use crate::{
    actors_interface::RbmRemoval,
    fem_io::{FemIo, GetIn, GetOut, SplitFem},
};

use super::{DiscreteModalSolver, Solver};
use gmt_fem::{fem_io::Inputs, fem_io::Outputs, FEM};
use interface::UniqueIdentifier;
use na::DMatrixView;
use nalgebra as na;
use nalgebra::DMatrix;
use rayon::prelude::*;
use serde_pickle as pickle;
use std::{f64::consts::PI, fs::File, marker::PhantomData, path::Path};

#[derive(Debug, thiserror::Error)]
pub enum StateSpaceError {
    #[error("argument {0} is missing")]
    MissingArguments(String),
    #[error("sampling frequency not set")]
    SamplingFrequency,
    #[error("{0}")]
    Matrix(String),
    #[error("FEM IO error")]
    FemIO(#[from] gmt_fem::FemError),
    #[error(transparent)]
    IOError(#[from] std::io::Error),
    #[cfg(feature = "bincode")]
    #[error(transparent)]
    Encode(#[from] bincode::error::EncodeError),
    #[cfg(feature = "bincode")]
    #[error(transparent)]
    Decode(#[from] bincode::error::DecodeError),
    #[error("failed to find the index of the output: {0}")]
    IndexNotFound(String),
}

type Result<T> = std::result::Result<T, StateSpaceError>;

/// This structure is the state space model builder based on a builder pattern design
#[derive(Default)]
pub struct DiscreteStateSpace<'a, T: Solver + Default> {
    sampling: Option<f64>,
    fem: Option<Box<FEM>>,
    zeta: Option<f64>,
    eigen_frequencies: Option<Vec<(usize, f64)>>,
    max_eigen_frequency: Option<f64>,
    hankel_singular_values_threshold: Option<f64>,
    hankel_frequency_lower_bound: Option<f64>,
    #[allow(dead_code)]
    use_static_gain: bool,
    phantom: PhantomData<T>,
    ins: Vec<Box<dyn GetIn>>,
    outs: Vec<Box<dyn GetOut>>,
    ins_transform: Vec<Option<DMatrixView<'a, f64>>>,
    outs_transform: Vec<Option<DMatrixView<'a, f64>>>,
    pub facesheet_nodes: Option<RbmRemoval>,
    pub m1_figure_nodes: Option<RbmRemoval>,
}
impl<'a, T: Solver + Default> From<FEM> for DiscreteStateSpace<'a, T> {
    /// Creates a state space model builder from a FEM structure
    fn from(fem: FEM) -> Self {
        Self {
            fem: Some(Box::new(fem)),
            ..Self::default()
        }
    }
}
impl<'a, T: Solver + Default> DiscreteStateSpace<'a, T> {
    pub fn set_facesheet_nodes(mut self) -> Result<Self> {
        log::info!("setting facesheet nodes");
        let fem = self.fem.as_ref().unwrap();
        self.facesheet_nodes = Some(RbmRemoval::new(fem, "M2_segment_#_axial_d")?);
        Ok(self)
    }
    pub fn set_m1_figure_nodes(mut self) -> Result<Self> {
        log::info!("setting m1 figure nodes");
        let fem = self.fem.as_ref().unwrap();
        self.m1_figure_nodes = Some(RbmRemoval::new(fem, "M1_segment_#_axial_d")?);
        Ok(self)
    }
    /// Prints information about the FEM
    pub fn fem_info(&self) -> &Self {
        if let Some(fem) = self.fem.as_ref() {
            println!("{}", fem);
        } else {
            println!("FEM missing!");
        }
        self
    }
    /// Set the sampling rate on Hz of the discrete state space model
    pub fn sampling(self, sampling: f64) -> Self {
        Self {
            sampling: Some(sampling),
            ..self
        }
    }
    /// Set the same proportional damping coefficients to all the modes
    pub fn proportional_damping(self, zeta: f64) -> Self {
        Self {
            zeta: Some(zeta),
            ..self
        }
    }
    ///
    pub fn use_static_gain_compensation(self) -> Self {
        Self {
            use_static_gain: true,
            ..self
        }
    }
    /// Overwrites some eigen frequencies
    ///
    /// The overwritten frequencies are specified as `(index,value)` where
    /// index is the corresponding eigen mode index and
    /// value is the new eigen frequency in Hz
    pub fn eigen_frequencies(self, eigen_frequencies: Vec<(usize, f64)>) -> Self {
        Self {
            eigen_frequencies: Some(eigen_frequencies),
            ..self
        }
    }
    /// Truncates the eigen frequencies to and including `max_eigen_frequency`
    ///
    /// The number of modes is set accordingly
    pub fn max_eigen_frequency(self, max_eigen_frequency: f64) -> Self {
        Self {
            max_eigen_frequency: Some(max_eigen_frequency),
            ..self
        }
    }
    /// Truncates the hankel singular values
    pub fn truncate_hankel_singular_values(self, hankel_singular_values_threshold: f64) -> Self {
        Self {
            hankel_singular_values_threshold: Some(hankel_singular_values_threshold),
            ..self
        }
    }
    /// Frequency lower bound for Hankel singular value truncation (default: 0Hz)
    pub fn hankel_frequency_lower_bound(self, hankel_frequency_lower_bound: f64) -> Self {
        Self {
            hankel_frequency_lower_bound: Some(hankel_frequency_lower_bound),
            ..self
        }
    }
    /// Saves the eigen frequencies to a pickle data file
    pub fn dump_eigen_frequencies<P: AsRef<Path>>(self, path: P) -> Self {
        let mut file = File::create(path).unwrap();
        pickle::to_writer(
            &mut file,
            &self.fem.as_ref().unwrap().eigen_frequencies,
            Default::default(),
        )
        .unwrap();
        self
    }
    /// Sets the model input based on the input type
    pub fn ins<U>(self) -> Self
    where
        Vec<Option<Inputs>>: FemIo<U>,
        U: 'static + UniqueIdentifier + Send + Sync,
    {
        let Self {
            mut ins,
            mut ins_transform,
            ..
        } = self;
        ins.push(Box::new(SplitFem::<U>::new()));
        ins_transform.push(None);
        Self {
            ins,
            ins_transform,
            ..self
        }
    }
    pub fn ins_with<U>(self, transform: DMatrixView<'a, f64>) -> Self
    where
        Vec<Option<Inputs>>: FemIo<U>,
        U: 'static + UniqueIdentifier + Send + Sync,
    {
        let Self {
            mut ins,
            mut ins_transform,
            ..
        } = self;
        ins.push(Box::new(SplitFem::<U>::new()));
        ins_transform.push(Some(transform));
        Self {
            ins,
            ins_transform,
            ..self
        }
    }
    /// Sets the model inputs based on the FEM inputs nomenclature
    pub fn ins_by_name<S: Into<String>>(self, names: Vec<S>) -> Result<Self> {
        let Self {
            mut ins,
            mut ins_transform,
            ..
        } = self;
        for name in names {
            ins.push(Box::<dyn GetIn>::try_from(name.into())?);
            ins_transform.push(None);
        }
        Ok(Self {
            ins,
            ins_transform,
            ..self
        })
    }

    pub fn ins_with_by_name<S: Into<String>>(
        self,
        names: Vec<S>,
        transforms: Vec<DMatrixView<'a, f64>>,
    ) -> Result<Self> {
        let Self {
            mut ins,
            mut ins_transform,
            ..
        } = self;
        for (name, transform) in names.into_iter().zip(transforms.into_iter()) {
            ins.push(Box::<dyn GetIn>::try_from(name.into())?);
            ins_transform.push(Some(transform));
        }
        Ok(Self {
            ins,
            ins_transform,
            ..self
        })
    }
    /// Sets the model output based on the output type
    pub fn outs<U>(self) -> Self
    where
        Vec<Option<Outputs>>: FemIo<U>,
        U: 'static + UniqueIdentifier + Send + Sync,
    {
        let Self {
            mut outs,
            mut outs_transform,
            ..
        } = self;
        outs.push(Box::new(SplitFem::<U>::new()));
        outs_transform.push(None);
        Self {
            outs,
            outs_transform,
            ..self
        }
    }
    pub fn outs_with<U>(self, transform: DMatrixView<'a, f64>) -> Self
    where
        Vec<Option<Outputs>>: FemIo<U>,
        U: 'static + UniqueIdentifier + Send + Sync,
    {
        let Self {
            mut outs,
            mut outs_transform,
            ..
        } = self;
        outs.push(Box::new(SplitFem::<U>::new()));
        outs_transform.push(Some(transform));
        Self {
            outs,
            outs_transform,
            ..self
        }
    }
    /// Sets the model outputs based on the FEM outputs nomenclature
    pub fn outs_by_name<S: Into<String>>(self, names: Vec<S>) -> Result<Self> {
        let Self {
            mut outs,
            mut outs_transform,
            ..
        } = self;
        for name in names {
            outs.push(Box::<dyn GetOut>::try_from(name.into())?);
            outs_transform.push(None);
        }
        Ok(Self {
            outs,
            outs_transform,
            ..self
        })
    }
    pub fn outs_with_by_name<S: Into<String>>(
        self,
        names: Vec<S>,
        transforms: Vec<DMatrixView<'a, f64>>,
    ) -> Result<Self> {
        let Self {
            mut outs,
            mut outs_transform,
            ..
        } = self;
        for (name, transform) in names.into_iter().zip(transforms.into_iter()) {
            outs.push(Box::<dyn GetOut>::try_from(name.into())?);
            outs_transform.push(Some(transform));
        }
        Ok(Self {
            outs,
            outs_transform,
            ..self
        })
    }
    #[cfg(fem)]
    pub fn including_mount(self) -> Self {
        use crate::fem_io;

        self.ins::<fem_io::actors_inputs::OSSElDriveTorque>()
            .ins::<fem_io::actors_inputs::OSSAzDriveTorque>()
            .ins::<fem_io::actors_inputs::OSSRotDriveTorque>()
            .outs::<fem_io::actors_outputs::OSSElEncoderAngle>()
            .outs::<fem_io::actors_outputs::OSSAzEncoderAngle>()
            .outs::<fem_io::actors_outputs::OSSRotEncoderAngle>()
    }
    pub fn including_m1(self, sids: Option<Vec<u8>>) -> Result<Self> {
        let mut names: Vec<_> = if let Some(sids) = sids {
            sids.into_iter()
                .map(|i| {
                    assert!(i > 0 && i < 8, "expected 1≤sid≤7,found sid={}", i);
                    format!("M1_actuators_segment_{i}")
                })
                .collect()
        } else {
            (1..=7)
                .map(|i| format!("M1_actuators_segment_{i}"))
                .collect()
        };
        names.push("OSS_Harpoint_delta_F".to_string());
        self.ins_by_name(names)
            .and_then(|this| this.outs_by_name(vec!["OSS_Hardpoint_D"]))
    }
    pub fn including_asms(
        self,
        sids: Option<Vec<u8>>,
        ins_transforms: Option<Vec<DMatrixView<'a, f64>>>,
        outs_transforms: Option<Vec<DMatrixView<'a, f64>>>,
    ) -> Result<Self> {
        let mut ins1_names = vec![];
        let mut ins2_names = vec![];
        let mut outs_names = vec![];
        for i in sids.unwrap_or_else(|| (1..=7).collect()) {
            assert!(i > 0 && i < 8, "expected 1≤sid≤7,found sid={}", i);
            ins1_names.push(format!("MC_M2_S{i}_VC_delta_F"));
            ins2_names.push(format!("MC_M2_S{i}_fluid_damping_F"));
            outs_names.push(format!("MC_M2_S{i}_VC_delta_D"))
        }
        match (ins_transforms, outs_transforms) {
            (None, None) => self
                .ins_by_name(ins1_names)
                .and_then(|this| this.ins_by_name(ins2_names))
                .and_then(|this| this.outs_by_name(outs_names)),
            (None, Some(outs_transforms)) => self
                .ins_by_name(ins1_names)
                .and_then(|this| this.ins_by_name(ins2_names))
                .and_then(|this| this.outs_with_by_name(outs_names, outs_transforms)),
            (Some(ins_transforms), None) => self
                .ins_with_by_name(ins1_names, ins_transforms.clone())
                .and_then(|this| this.ins_with_by_name(ins2_names, ins_transforms))
                .and_then(|this| this.outs_by_name(outs_names)),
            (Some(ins_transforms), Some(outs_transforms)) => self
                .ins_with_by_name(ins1_names, ins_transforms.clone())
                .and_then(|this| this.ins_with_by_name(ins2_names, ins_transforms))
                .and_then(|this| this.outs_with_by_name(outs_names, outs_transforms)),
        }
    }
    /// Returns the Hankel singular value for a given eigen mode
    pub fn hankel_singular_value(w: f64, z: f64, b: &[f64], c: &[f64]) -> f64 {
        let norm_x = |x: &[f64]| x.iter().map(|x| x * x).sum::<f64>().sqrt();
        0.25 * norm_x(b) * norm_x(c) / (w * z)
    }
    /// Computes the Hankel singular values
    pub fn hankel_singular_values(&self) -> Result<Vec<f64>> {
        let fem = self
            .fem
            .as_ref()
            .map_or(Err(StateSpaceError::MissingArguments("FEM".to_owned())), Ok)?;
        let n_mode = fem.n_modes();
        let forces_2_modes = na::DMatrix::from_row_slice(
            n_mode,
            fem.inputs_to_modal_forces.len() / n_mode,
            &fem.inputs_to_modal_forces,
        );
        let modes_2_nodes = na::DMatrix::from_row_slice(
            fem.modal_disp_to_outputs.len() / n_mode,
            n_mode,
            &fem.modal_disp_to_outputs,
        );
        let w = fem.eigen_frequencies_to_radians();
        let zeta = match self.zeta {
            Some(zeta) => {
                log::info!("Proportional coefficients modified, new value: {:.4}", zeta);
                vec![zeta; fem.n_modes()]
            }
            None => fem.proportional_damping_vec.clone(),
        };
        Ok((0..fem.n_modes())
            .into_par_iter()
            .map(|k| {
                let b = forces_2_modes.row(k).clone_owned();
                let c = modes_2_nodes.column(k);
                // (
                // w[k],
                Self::hankel_singular_value(w[k], zeta[k], b.as_slice(), c.as_slice())
                // )
            })
            .collect())
    }
    /// Computes the Hankel singular values for the FEM reduced to some inputs and some outputs
    pub fn reduced_hankel_singular_values(&mut self) -> Result<Vec<f64>> {
        let (w, n_modes, zeta, _) = self.properties()?;
        let w_zeta = w.into_iter().zip(zeta.into_iter());
        match (self.in2mode(n_modes), self.mode2out(n_modes)) {
            (Some(forces_2_modes), Some(modes_2_nodes)) => Ok(w_zeta
                .zip(forces_2_modes.row_iter())
                .zip(modes_2_nodes.column_iter())
                .map(|(((w, zeta), b), c)| {
                    Self::hankel_singular_value(w, zeta, b.clone_owned().as_slice(), c.as_slice())
                })
                .collect()),
            _ => Err(StateSpaceError::Matrix(
                "Failed to build both modal transformation matrices".to_string(),
            )),
        }
    }
    fn in2mode(&mut self, n_mode: usize) -> Option<DMatrix<f64>> {
        if let Some(fem) = &self.fem {
            let v: Vec<f64> = self
                .ins
                .iter_mut()
                .zip(&self.ins_transform)
                .scan(0usize, |s, (x, t)| {
                    // let mat = x.get_in(fem).unwrap();
                    let mat = if let Some(t) = t {
                        x.get_in(fem).unwrap() * t
                    } else {
                        x.get_in(fem).unwrap()
                    };
                    let l = mat.ncols();
                    x.set_range(*s, *s + l);
                    *s += l;
                    Some(mat)
                })
                .flat_map(|x| {
                    x.column_iter()
                        .flat_map(|x| x.iter().take(n_mode).cloned().collect::<Vec<f64>>())
                        .collect::<Vec<f64>>()
                })
                .collect();
            Some(DMatrix::from_column_slice(n_mode, v.len() / n_mode, &v))
        } else {
            None
        }
    }
    fn mode2out(&mut self, n_mode: usize) -> Option<DMatrix<f64>> {
        if let Some(fem) = &self.fem {
            let v: Vec<f64> = self
                .outs
                .iter_mut()
                .zip(&self.outs_transform)
                .scan(0usize, |s, (x, t)| {
                    let mat = if let Some(t) = t {
                        t * x.get_out(fem).unwrap()
                    } else {
                        x.get_out(fem).unwrap()
                    };
                    let l = mat.nrows();
                    x.set_range(*s, *s + l);
                    *s += l;
                    Some(mat)
                })
                .flat_map(|x| {
                    x.row_iter()
                        .flat_map(|x| x.iter().take(n_mode).cloned().collect::<Vec<f64>>())
                        .collect::<Vec<f64>>()
                })
                .collect();
            Some(DMatrix::from_row_slice(v.len() / n_mode, n_mode, &v))
        } else {
            None
        }
    }
    /// Returns the FEM static gain
    pub fn static_gain(&self) -> Option<na::DMatrix<f64>> {
        let Some(fem) = self.fem.as_ref() else {
            return None;
        };
        let n_io = fem.n_io;
        fem.static_gain.as_ref().and_then(|x| {
            let g = DMatrix::from_row_slice(n_io.1, n_io.0, x);
            self.reduce2io(&g)
        })
    }
    #[allow(dead_code)]
    fn reduce2io(&self, matrix: &DMatrix<f64>) -> Option<DMatrix<f64>> {
        if let Some(fem) = &self.fem {
            let m = DMatrix::from_columns(
                &self
                    .ins
                    .iter()
                    .zip(&self.ins_transform)
                    .filter_map(|(x, t)| match (x.trim_in(fem, matrix), t) {
                        (Some(x), Some(t)) => Some(x * t),
                        (x, None) => x,
                        _ => None,
                    })
                    .flat_map(|x| x.column_iter().map(|x| x.clone_owned()).collect::<Vec<_>>())
                    .collect::<Vec<_>>(),
            );
            Some(DMatrix::from_rows(
                &self
                    .outs
                    .iter()
                    .zip(&self.outs_transform)
                    .filter_map(|(x, t)| match (x.trim_out(fem, &m), t) {
                        (Some(x), Some(t)) => Some(t * x),
                        (x, None) => x,
                        _ => None,
                    })
                    .flat_map(|x| x.row_iter().map(|x| x.clone_owned()).collect::<Vec<_>>())
                    .collect::<Vec<_>>(),
            ))
        } else {
            None
        }
    }
    fn properties(&self) -> Result<(Vec<f64>, usize, Vec<f64>, (usize, usize))> {
        let fem = self
            .fem
            .as_ref()
            .map_or(Err(StateSpaceError::MissingArguments("FEM".to_owned())), Ok)?;
        let mut w = fem.eigen_frequencies_to_radians();
        if let Some(eigen_frequencies) = &self.eigen_frequencies {
            log::info!("Eigen values modified");
            eigen_frequencies.into_iter().for_each(|(i, v)| {
                w[*i] = 2. * PI * v;
            });
        }
        let n_modes = match self.max_eigen_frequency {
            Some(max_ef) => {
                fem.eigen_frequencies
                    .iter()
                    .fold(0, |n, ef| if ef <= &max_ef { n + 1 } else { n })
            }
            None => fem.n_modes(),
        };
        if let Some(max_ef) = self.max_eigen_frequency {
            log::info!("Eigen frequencies truncated to {:.3}Hz, hence reducing the number of modes from {} down to {}",max_ef,fem.n_modes(),n_modes)
        }
        let zeta = match self.zeta {
            Some(zeta) => {
                log::info!("Proportional coefficients modified, new value: {:.4}", zeta);
                vec![zeta; n_modes]
            }
            None => fem.proportional_damping_vec.clone(),
        };
        let n_io = fem.n_io;
        Ok((w, n_modes, zeta, n_io))
    }
    #[cfg(fem)]
    pub fn build(mut self) -> Result<DiscreteModalSolver<T>> {
        use std::sync::Arc;

        use crate::fem_io;

        let tau = self.sampling.map_or(
            Err(StateSpaceError::MissingArguments("sampling".to_owned())),
            |x| Ok(1f64 / x),
        )?;

        let (w, n_modes, zeta, n_io) = self.properties()?;

        match (self.in2mode(n_modes), self.mode2out(n_modes)) {
            (Some(forces_2_modes), Some(modes_2_nodes)) => {
                log::info!("forces 2 modes: {:?}", forces_2_modes.shape());
                log::info!("modes 2 nodes: {:?}", modes_2_nodes.shape());

                let psi_dcg = if self.use_static_gain {
                    log::info!(
                        "The elements of psi_dcg corresponding to 
    - OSSAzDriveTorque
    - OSSElDriveTorque
    - OSSRotDriveTorque
and
    - OSSAzEncoderAngle
    - OSSElEncoderAngle
    - OSSRotEncoderAngle
are set to zero."
                    );
                    let q = self
                        .fem
                        .as_mut()
                        .unwrap()
                        .static_gain
                        .as_ref()
                        .map(|x| DMatrix::from_row_slice(n_io.1, n_io.0, x));
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
                                .downcast_ref::<SplitFem<fem_io::actors_inputs::OSSRotDriveTorque>>(
                                )
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
                        println!(
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
                } else {
                    None
                };

                let state_space: Vec<_> = match self.hankel_singular_values_threshold {
                    Some(hsv_t) => (0..n_modes)
                        .filter_map(|k| {
                            let b = forces_2_modes.row(k).clone_owned();
                            let c = modes_2_nodes.column(k);
                            let hsv = Self::hankel_singular_value(
                                w[k],
                                zeta[k],
                                b.as_slice(),
                                c.as_slice(),
                            );
                            if w[k]
                                < self
                                    .hankel_frequency_lower_bound
                                    .map(|x| 2. * PI * x)
                                    .unwrap_or_default()
                            {
                                Some(T::from_second_order(
                                    tau,
                                    w[k],
                                    zeta[k],
                                    b.as_slice().to_vec(),
                                    c.as_slice().to_vec(),
                                ))
                            } else {
                                if hsv > hsv_t {
                                    Some(T::from_second_order(
                                        tau,
                                        w[k],
                                        zeta[k],
                                        b.as_slice().to_vec(),
                                        c.as_slice().to_vec(),
                                    ))
                                } else {
                                    None
                                }
                            }
                        })
                        .collect(),
                    None => (0..n_modes)
                        .map(|k| {
                            let b = forces_2_modes.row(k).clone_owned();
                            let c = modes_2_nodes.column(k);
                            T::from_second_order(
                                tau,
                                w[k],
                                zeta[k],
                                b.as_slice().to_vec(),
                                c.as_slice().to_vec(),
                            )
                        })
                        .collect(),
                };
                Ok(DiscreteModalSolver {
                    u: vec![0f64; forces_2_modes.ncols()],
                    y: vec![0f64; modes_2_nodes.nrows()],
                    state_space,
                    ins: self.ins,
                    outs: self.outs,
                    psi_dcg: psi_dcg.map(|psi_dcg| Arc::new(psi_dcg)),
                    facesheet_nodes: self.facesheet_nodes,
                    m1_figure_nodes: self.m1_figure_nodes,
                    ..Default::default()
                })
            }
            (Some(_), None) => Err(StateSpaceError::Matrix(
                "Failed to build modes to nodes transformation matrix".to_string(),
            )),
            (None, Some(_)) => Err(StateSpaceError::Matrix(
                "Failed to build forces to nodes transformation matrix".to_string(),
            )),
            _ => Err(StateSpaceError::Matrix(
                "Failed to build both modal transformation matrices".to_string(),
            )),
        }
    }
    #[cfg(not(fem))]
    pub fn build(mut self) -> Result<DiscreteModalSolver<T>> {
        let tau = self.sampling.map_or(
            Err(StateSpaceError::MissingArguments("sampling".to_owned())),
            |x| Ok(1f64 / x),
        )?;

        let (w, n_modes, zeta, _) = self.properties()?;

        match (self.in2mode(n_modes), self.mode2out(n_modes)) {
            (Some(forces_2_modes), Some(modes_2_nodes)) => {
                log::info!("forces 2 modes: {:?}", forces_2_modes.shape());
                log::info!("modes 2 nodes: {:?}", modes_2_nodes.shape());

                let psi_dcg = None;

                let state_space: Vec<_> = match self.hankel_singular_values_threshold {
                    Some(hsv_t) => (0..n_modes)
                        .filter_map(|k| {
                            let b = forces_2_modes.row(k).clone_owned();
                            let c = modes_2_nodes.column(k);
                            let hsv = Self::hankel_singular_value(
                                w[k],
                                zeta[k],
                                b.as_slice(),
                                c.as_slice(),
                            );
                            if w[k]
                                < self
                                    .hankel_frequency_lower_bound
                                    .map(|x| 2. * PI * x)
                                    .unwrap_or_default()
                            {
                                Some(T::from_second_order(
                                    tau,
                                    w[k],
                                    zeta[k],
                                    b.as_slice().to_vec(),
                                    c.as_slice().to_vec(),
                                ))
                            } else {
                                if hsv > hsv_t {
                                    Some(T::from_second_order(
                                        tau,
                                        w[k],
                                        zeta[k],
                                        b.as_slice().to_vec(),
                                        c.as_slice().to_vec(),
                                    ))
                                } else {
                                    None
                                }
                            }
                        })
                        .collect(),
                    None => (0..n_modes)
                        .map(|k| {
                            let b = forces_2_modes.row(k).clone_owned();
                            let c = modes_2_nodes.column(k);
                            T::from_second_order(
                                tau,
                                w[k],
                                zeta[k],
                                b.as_slice().to_vec(),
                                c.as_slice().to_vec(),
                            )
                        })
                        .collect(),
                };
                Ok(DiscreteModalSolver {
                    u: vec![0f64; forces_2_modes.ncols()],
                    y: vec![0f64; modes_2_nodes.nrows()],
                    state_space,
                    ins: self.ins,
                    outs: self.outs,
                    psi_dcg,
                    facesheet_nodes: self.facesheet_nodes,
                    ..Default::default()
                })
            }
            (Some(_), None) => Err(StateSpaceError::Matrix(
                "Failed to build modes to nodes transformation matrix".to_string(),
            )),
            (None, Some(_)) => Err(StateSpaceError::Matrix(
                "Failed to build forces to nodes transformation matrix".to_string(),
            )),
            _ => Err(StateSpaceError::Matrix(
                "Failed to build both modal transformation matrices".to_string(),
            )),
        }
    }
}

impl<'a, S: Solver + Default> TryFrom<DiscreteStateSpace<'a, S>> for DiscreteModalSolver<S> {
    type Error = StateSpaceError;

    fn try_from(dss: DiscreteStateSpace<'a, S>) -> std::result::Result<Self, Self::Error> {
        dss.build()
    }
}
