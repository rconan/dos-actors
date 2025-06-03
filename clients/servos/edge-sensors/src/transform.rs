use faer::{linalg::solvers::SolveLstsqCore, prelude::*};
use faer_ext::IntoFaer;
use gmt_dos_clients_fem::{Model, Switch};
use gmt_fem::{FEM, FemError};

#[derive(thiserror::Error, Debug)]
pub enum TransformError {
    #[error("FEM error in Transform")]
    FEM(#[from] FemError),
    #[error("failed to compute from gain matrix")]
    FromGainMatrix,
    #[error("failed to compute to gain matrix")]
    ToGainMatrix,
}

type Result<T> = std::result::Result<T, TransformError>;

#[derive(Debug, Clone)]
pub struct RowsOrCols {
    first: usize,
    n: usize,
}
impl RowsOrCols {
    pub fn new(first: usize, n: usize) -> Self {
        Self { first, n }
    }
}
#[derive(Debug, Clone, Default)]
pub struct IO {
    name: String,
    rows: Option<RowsOrCols>,
    cols: Option<RowsOrCols>,
}
impl IO {
    pub fn new(name: impl ToString) -> Self {
        Self {
            name: name.to_string(),
            ..Default::default()
        }
    }
    pub fn rows(mut self, first: usize, n: usize) -> Self {
        self.rows = Some(RowsOrCols::new(first, n));
        self
    }
    pub fn cols(mut self, first: usize, n: usize) -> Self {
        self.cols = Some(RowsOrCols::new(first, n));
        self
    }
}
impl From<&str> for IO {
    fn from(value: &str) -> Self {
        Self {
            name: value.to_string(),
            ..Default::default()
        }
    }
}
impl From<String> for IO {
    fn from(name: String) -> Self {
        Self {
            name,
            ..Default::default()
        }
    }
}
#[derive(Debug, Default, Clone)]
pub struct Transform {
    input: IO,
    output_from: IO,
    output_to: IO,
}
impl Transform {
    pub fn new(output_from: impl Into<IO>, output_to: impl Into<IO>, input: impl Into<IO>) -> Self {
        Self {
            input: input.into(),
            output_from: output_from.into(),
            output_to: output_to.into(),
            ..Default::default()
        }
    }

    pub fn build(self, fem: &mut FEM) -> Result<Mat<f64>> {
        fem.switch_inputs(Switch::Off, None);
        let k_from = fem
            .switch_outputs(Switch::Off, None)
            .switch_inputs_by_name(vec![&self.input.name], Switch::On)?
            .switch_outputs_by_name(vec![self.output_from.name], Switch::On)?
            .reduced_static_gain()
            .map(|x| {
                match (self.output_from.rows, self.output_from.cols) {
                    (None, None) => x,
                    (None, Some(RowsOrCols { first, n })) => x.columns(first, n).into(),
                    (Some(RowsOrCols { first, n }), None) => x.rows(first, n).into(),
                    (
                        Some(RowsOrCols { first: r0, n: rn }),
                        Some(RowsOrCols { first: c0, n: cn }),
                    ) => x.rows(r0, rn).columns(c0, cn).into(),
                }
                .transpose()
            })
            .ok_or(TransformError::FromGainMatrix)?;
        let k_to = fem
            .switch_outputs(Switch::Off, None)
            .switch_inputs_by_name(vec![self.input.name], Switch::On)?
            .switch_outputs_by_name(vec![self.output_to.name], Switch::On)?
            .reduced_static_gain()
            .map(|x| {
                match (self.output_to.rows, self.output_to.cols) {
                    (None, None) => x,
                    (None, Some(RowsOrCols { first, n })) => x.columns(first, n).into(),
                    (Some(RowsOrCols { first, n }), None) => x.rows(first, n).into(),
                    (
                        Some(RowsOrCols { first: r0, n: rn }),
                        Some(RowsOrCols { first: c0, n: cn }),
                    ) => x.rows(r0, rn).columns(c0, cn).into(),
                }
                .transpose()
            })
            .ok_or(TransformError::FromGainMatrix)?;
        fem.switch_inputs(Switch::On, None);
        let a = k_to.view_range(.., ..).into_faer();
        let mut b = k_from.view_range(.., ..).into_faer().to_owned();
        let qr = a.qr();
        qr.solve_lstsq_in_place_with_conj(faer::Conj::No, b.as_mut());
        // let t = x.as_ref().into_nalgebra().clone_owned().transpose();
        Ok(b.transpose().to_owned())
    }
}
