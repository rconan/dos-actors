use std::fmt::Display;
use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::ops::Mul;
use std::path::Path;
use std::time::Instant;

use crseo::wavefrontsensor::PyramidBuilder;

use interface::UniqueIdentifier;
use nalgebra as na;
use serde::{Deserialize, Serialize};

use crate::ngao::{Calibrating, CalibratingError, Calibration};
use crate::pyramid::PyramidData;

mod builder;
pub use builder::PyramidCalibratorBuilder;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Segment {
    sid: u8,
    n_mode: usize,
    mask: na::DMatrix<bool>,
    calibration: na::DMatrix<f32>,
}

impl Display for Segment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Segment #{} with {} modes, calibration: {:?}, mask: {:?}",
            self.sid,
            self.n_mode,
            self.calibration.shape(),
            self.mask.shape(),
        )
    }
}

/// Pyramid calibration data
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PyramidCalibrator {
    pub n_mode: usize,
    segments: Vec<Segment>,
    piston_mask: (Vec<bool>, Vec<bool>),
    h_filter: Vec<bool>,
    p_filter: Vec<bool>,
    offset: Vec<f32>,
    h_matrix: na::DMatrix<f32>,
    p_matrix: na::DMatrix<f32>,
    estimator: Option<Estimator>,
}

impl TryFrom<&Path> for PyramidCalibrator {
    type Error = PyramidCalibratorError;

    fn try_from(value: &Path) -> Result<Self, Self::Error> {
        let file = File::open(value)?;
        let mut buffer = BufReader::new(file);
        Ok(bincode::serde::decode_from_std_read(
            &mut buffer,
            bincode::config::standard(),
        )?)
    }
}

impl TryFrom<&str> for PyramidCalibrator {
    type Error = PyramidCalibratorError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        let file = File::open(value)?;
        let mut buffer = BufReader::new(file);
        Ok(bincode::serde::decode_from_std_read(
            &mut buffer,
            bincode::config::standard(),
        )?)
    }
}

impl PyramidCalibrator {
    /// Serializes a [PyramidCalibrator] instance to [bincode]
    pub fn save<P: AsRef<Path>>(self, path: P) -> Result<Self, PyramidCalibratorError> {
        let file = File::create(path.as_ref())?;
        let mut buffer = BufWriter::new(file);
        bincode::serde::encode_into_std_write(&self, &mut buffer, bincode::config::standard())?;
        Ok(self)
    }
}

impl Display for PyramidCalibrator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let n = self.segments.len();
        let px = self
            .piston_mask
            .0
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        let py = self
            .piston_mask
            .0
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        let hn = self
            .h_filter
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        let pn = self
            .p_filter
            .iter()
            .filter(|&&f| f)
            .enumerate()
            .map(|(i, _)| i)
            .last()
            .unwrap()
            + 1;
        writeln!(
            f,
            r"
Pyramid calibrator for {} segments with {} modes:
 * masks:
   * piston non-zeros: [{},{}]
   * H filter non zeros: {}
   * P filter non zeros: {}
   * H matrix: {:?}
   * P matrix: {:?}
 * offset vector size: {}
 * {}
        ",
            n,
            self.n_mode,
            px,
            py,
            hn,
            pn,
            self.h_matrix.shape(),
            self.p_matrix.shape(),
            self.offset.len(),
            self.estimator
                .as_ref()
                .map_or("no estimator".to_string(), |estimator| estimator
                    .to_string())
        )?;
        for segment in &self.segments {
            writeln!(f, "{segment}")?;
        }
        Ok(())
    }
}

impl Calibrating for PyramidCalibrator {
    type ProcessorData = PyramidData<f32>;
    type Output = Vec<f64>;
}

/// The different kind of pyramid reconstructor
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum Estimator {
    H0(na::DMatrix<f32>),
    H00(na::DMatrix<f32>),
    H(na::DMatrix<f32>),
    ///
    P(na::DMatrix<f32>),
    HP(na::DMatrix<f32>),
    ConstrainedHP(na::DMatrix<f32>),
}

impl Display for Estimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Estimator::H00(mat) => write!(f, "H0 estimator size: {:?}", mat.shape()),
            Estimator::H0(mat) => write!(f, "H0 estimator size: {:?}", mat.shape()),
            Estimator::H(mat) => write!(f, "H estimator size: {:?}", mat.shape()),
            Estimator::P(mat) => write!(f, "P estimator size: {:?}", mat.shape()),
            Estimator::HP(mat) => write!(f, "HP estimator size: {:?}", mat.shape()),
            Estimator::ConstrainedHP(mat) => {
                write!(f, "constrained HP estimator size: {:?}", mat.shape())
            }
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum PyramidCalibratorError {
    #[error("failed to compute H matrix pseudo-inverse: {0}")]
    HEstimator(String),
    #[error("failed to compute P matrix pseudo-inverse: {0}")]
    PEstimator(String),
    #[error("failed to compute HP matrix inverse")]
    HPEstimator,
    #[error("file error")]
    File(#[from] std::io::Error),
    #[error("decoding error")]
    Decode(#[from] bincode::error::DecodeError),
    #[error("encoding error")]
    Encode(#[from] bincode::error::EncodeError),
}

impl PyramidCalibrator {
    /// Returns the pyramid builder
    pub fn builder(
        pym: PyramidBuilder,
        modes: impl Into<String>,
        n_mode: usize,
    ) -> PyramidCalibratorBuilder {
        PyramidCalibratorBuilder {
            pym,
            sids: vec![1, 2, 3, 4, 5, 6, 7],
            modes: modes.into(),
            n_mode,
            n_gpu: 1,
            n_thread: None,
            piston_mask_threshold: 0.55,
            stroke: 25e-9,
        }
    }
    /// Returns the pyramid data
    pub fn data(&self, rhs: &PyramidData<f32>) -> Vec<f32> {
        let PyramidData { sx, sy, .. } = rhs;
        sx.iter()
            .chain(sy.iter())
            .zip(self.h_filter.iter().cycle())
            .filter_map(|(s, h)| h.then_some(*s))
            .chain(
                sx.iter()
                    .chain(sy.iter())
                    .zip(&self.p_filter)
                    .filter_map(|(s, p)| p.then_some(*s)),
            )
            .collect()
    }
    #[cfg(feature = "faer")]
    /// Returns the condition number of the `P` matrix
    pub fn p_matrix_cond(&self) -> f32 {
        use faer::linalg::solvers::Svd;
        use faer_ext::IntoFaer;
        let mat = self.p_matrix.view_range(.., ..).into_faer();
        let svd = Svd::new(mat).expect("failed to compute SVD");
        let s_diag = svd.S();
        let k = s_diag.column_vector().nrows() - 1;
        s_diag[0] / s_diag[k]
    }
    #[cfg(feature = "faer")]
    /// Returns the condition number of the `H` matrix
    pub fn h_matrix_cond(&self) -> f32 {
        use faer::linalg::solvers::Svd;
        use faer_ext::IntoFaer;
        let mat = self.h_matrix.view_range(.., ..).into_faer();
        let svd = Svd::new(mat).expect("failed to compute SVD");
        let s_diag = svd.S();
        let k = s_diag.column_vector().nrows() - 1;
        s_diag[0] / s_diag[k]
    }
    #[cfg(not(feature = "faer"))]
    /// Computes the pyramid LSQ reconstructor excluding piston
    pub fn h0_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        println!("computing the H0 estimator (with nalgebra) ...");
        let now = Instant::now();
        let indices: Vec<_> = (0..7).map(|i| i * self.n_mode).collect();
        let h0_matrix = self.h_matrix.clone().remove_columns_at(&indices);
        let mat = h0_matrix
            .pseudo_inverse(0f32)
            .map_err(|e| PyramidCalibratorError::HEstimator(e.into()))?;
        self.estimator = Some(Estimator::H0(mat));
        println!("  ... in {}s", now.elapsed().as_secs());
        Ok(self)
    }
    #[cfg(feature = "faer")]
    /// Computes the pyramid LSQ reconstructor excluding piston
    pub fn h00_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        self.h0_estimator()?;
        if let Some(Estimator::H0(mat)) = self.estimator.take() {
            self.estimator = Some(Estimator::H00(
                (0..7).fold(mat, |mat, i| mat.insert_row(i * self.n_mode, 0f32)),
            ));
        }
        Ok(self)
    }
    #[cfg(feature = "faer")]
    /// Computes the pyramid LSQ reconstructor excluding piston
    pub fn h0_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        use faer::{linalg::solvers::Svd, Mat};
        use faer_ext::{IntoFaer, IntoNalgebra};

        println!("computing the H0 estimator (with faer) ...");
        let now = Instant::now();
        let indices: Vec<_> = (0..7).map(|i| i * self.n_mode).collect();
        let h0_matrix = self.h_matrix.clone().remove_columns_at(&indices);
        let h0_matrix = h0_matrix.view_range(.., ..).into_faer();

        let m = h0_matrix.nrows();
        let n = h0_matrix.ncols();
        let svd = Svd::new(h0_matrix).expect("failed to compute SVD");

        let s_diag = svd.S();
        let mut s_inv = Mat::zeros(n, m);
        for i in 0..Ord::min(m, n) {
            s_inv[(i, i)] = 1.0 / s_diag[i];
        }

        let mat = svd.V() * &s_inv * svd.U().adjoint();

        self.estimator = Some(Estimator::H0(mat.as_ref().into_nalgebra().into()));
        println!("  ... in {}s", now.elapsed().as_secs());
        Ok(self)
    }
    pub fn h_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        let mat = self
            .h_matrix
            .clone()
            .pseudo_inverse(0f32)
            .map_err(|e| PyramidCalibratorError::HEstimator(e.into()))?;
        self.estimator = Some(Estimator::H(mat));
        Ok(self)
    }
    pub fn p_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        let mat = self
            .p_matrix
            .clone()
            .pseudo_inverse(0f32)
            .map_err(|e| PyramidCalibratorError::PEstimator(e.into()))?;
        self.estimator = Some(Estimator::P(mat));
        Ok(self)
    }
    pub fn hp07_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        println!(
            "computing the pyramid LSQ reconstructor (removing the center segment column) ..."
        );
        let now = Instant::now();
        let h_matrix = self
            .h_matrix
            .view_range(.., ..)
            .remove_column(6 * self.n_mode);
        let p_matrix = self
            .p_matrix
            .view_range(.., ..)
            .remove_column(6 * self.n_mode);
        let hp_matrix = h_matrix.transpose() * &h_matrix + p_matrix.transpose() * &p_matrix;
        let columns: Vec<_> = h_matrix
            .transpose()
            .column_iter()
            .map(|column| column.clone_owned())
            .chain(
                p_matrix
                    .transpose()
                    .column_iter()
                    .map(|column| column.clone_owned()),
            )
            .collect();
        let m_matrix = na::DMatrix::from_columns(&columns);
        let reconstructor = hp_matrix
            .try_inverse()
            .ok_or(PyramidCalibratorError::HPEstimator)?
            * &m_matrix;
        println!(
            "Reconstructor: [{:?}] in {}s",
            reconstructor.shape(),
            now.elapsed().as_secs()
        );
        self.estimator = Some(Estimator::HP(reconstructor));
        Ok(self)
    }
    #[cfg(feature = "faer")]
    pub fn hp_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        use faer::{linalg::solvers::Svd, Mat};
        use faer_ext::{IntoFaer, IntoNalgebra};

        println!("computing the pyramid LSQ reconstructor (truncating the last eigen value) with faer ...");
        let now = Instant::now();
        let h_matrix = self.h_matrix.view_range(.., ..);
        let p_matrix = self.p_matrix.view_range(.., ..);
        // let hp_matrix = h_matrix.transpose() * &h_matrix + p_matrix.transpose() * &p_matrix;
        let columns: Vec<_> = h_matrix
            .transpose()
            .column_iter()
            .map(|column| column.clone_owned())
            .chain(
                p_matrix
                    .transpose()
                    .column_iter()
                    .map(|column| column.clone_owned()),
            )
            .collect();
        let m_matrix = na::DMatrix::from_columns(&columns).transpose();
        let m_matrix = m_matrix.view_range(.., ..).into_faer();
        // dbg!(m_matrix.size());

        let m = m_matrix.nrows();
        let n = m_matrix.ncols();
        let svd = Svd::new(m_matrix).expect("failed to compute SVD");

        let s_diag = svd.S();
        let mut s_inv = Mat::zeros(n, m);
        for i in 0..Ord::min(m, n) - 1 {
            s_inv[(i, i)] = 1.0 / s_diag[i];
        }

        let mat = svd.V() * &s_inv * svd.U().adjoint();

        let reconstructor = mat.as_ref().into_nalgebra();

        println!(
            "Reconstructor: [{:?}] in {}s",
            reconstructor.shape(),
            now.elapsed().as_secs()
        );
        self.estimator = Some(Estimator::HP(reconstructor.into()));
        Ok(self)
    }
    #[cfg(not(feature = "faer"))]
    pub fn hp_estimator(&mut self) -> Result<&mut Self, PyramidCalibratorError> {
        println!("computing the pyramid LSQ reconstructor (truncating the last eigen value) ...");
        let now = Instant::now();
        let h_matrix = self.h_matrix.view_range(.., ..);
        let p_matrix = self.p_matrix.view_range(.., ..);
        // let hp_matrix = h_matrix.transpose() * &h_matrix + p_matrix.transpose() * &p_matrix;
        let columns: Vec<_> = h_matrix
            .transpose()
            .column_iter()
            .map(|column| column.clone_owned())
            .chain(
                p_matrix
                    .transpose()
                    .column_iter()
                    .map(|column| column.clone_owned()),
            )
            .collect();
        let m_matrix = na::DMatrix::from_columns(&columns).transpose();
        dbg!(m_matrix.shape());
        let mut svd = m_matrix.svd(true, true);
        dbg!(svd
            .singular_values
            .iter()
            .rev()
            .take(10)
            .collect::<Vec<_>>());
        let n = svd.singular_values.len();
        for i in 0..n - 1 {
            let val = svd.singular_values[i].clone();
            svd.singular_values[i] = val.recip();
        }
        svd.singular_values[n - 1] = 0f32;
        let reconstructor = svd
            .recompose()
            .map(|m| m.adjoint())
            .map_err(|e| PyramidCalibratorError::HEstimator(e.into()))?;

        println!(
            "Reconstructor: [{:?}] in {}s",
            reconstructor.shape(),
            now.elapsed().as_secs()
        );
        self.estimator = Some(Estimator::HP(reconstructor));
        Ok(self)
    }
    pub fn set_hp_estimator(&mut self, reconstructor: na::DMatrix<f32>) -> &mut Self {
        let (n_h, m_h) = self.h_matrix.shape();
        let (n_p, _) = self.p_matrix.shape();
        let expected = (m_h, n_h + n_p);
        assert!(
            reconstructor.shape() == expected,
            "HP estimator shape mismatch: expected {:?}, found {:?}",
            expected,
            reconstructor.shape()
        );
        self.estimator = Some(Estimator::HP(reconstructor));
        self
    }
}

impl From<PyramidCalibrator> for Calibration<PyramidCalibrator> {
    fn from(calibrator: PyramidCalibrator) -> Self {
        Self {
            calibrator,
            output: Default::default(),
        }
    }
}

pub enum PyramidCommand {}
impl UniqueIdentifier for PyramidCommand {
    type DataType = Vec<f64>;
}

impl Mul<&PyramidData<f32>> for &PyramidCalibrator {
    type Output = <PyramidCalibrator as Calibrating>::Output;

    fn mul(self, rhs: &PyramidData<f32>) -> Self::Output {
        let sxy: Vec<_> = self
            .data(rhs)
            .into_iter()
            .zip(&self.offset)
            .map(|(s, s0)| s - *s0)
            .collect();
        self.estimator
            .as_ref()
            .map(|estimator| {
                match estimator {
                    Estimator::H0(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy[..self.h_matrix.nrows()]);
                        let mut v = v.as_slice().to_vec();
                        for i in 0..7 {
                            v.insert(i * self.n_mode, 0f32);
                        }
                        v
                    }
                    Estimator::H00(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy[..self.h_matrix.nrows()]);
                        v.as_slice().to_vec()
                    }
                    Estimator::H(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy[..self.h_matrix.nrows()]);
                        v.as_slice().to_vec()
                    }
                    Estimator::P(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy[self.h_matrix.nrows()..]);
                        let mut v = v.as_slice().to_vec();
                        v.insert(6 * self.n_mode, 0f32);
                        v
                    }
                    Estimator::HP(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy);
                        v.as_slice().to_vec()
                    }
                    Estimator::ConstrainedHP(mat) => {
                        let v = mat * na::DVector::from_column_slice(&sxy);
                        v.as_slice().to_vec()
                    }
                }
                .into_iter()
                .map(|x| x as f64)
                .collect::<Vec<_>>()
            })
            .unwrap_or_default()
    }
}
