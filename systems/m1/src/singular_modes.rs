use std::{env, fmt::Display, ops::Deref, path::Path};

use serde::Deserialize;

/// M1 segment singular modes (aka bending modes)
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub struct SegmentSingularModes {
    /// segment meshes vertex coordinates `[x,y,z]`
    mode_nodes: Vec<Vec<f64>>,
    /// segment actuator location `[x,y,z]`
    actuator_nodes: Vec<Vec<f64>>,
    /// segment left singular modes
    raw_modes: Vec<f64>,
    /// segment left singular modes restricted to the rigid body motions null space
    modes: Vec<f64>,
    /// modes to forces matrix transform
    mode_2_force: Vec<f64>,
    // /// segment right singular modes
    // right_modes: Vec<f64>,
    // /// segment singular values
    // singular_values: Vec<f64>,
    /// modes shape `[n_points,n_actuators]`
    shape: (usize, usize),
}

impl Display for SegmentSingularModes {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (ns, ..) = self.shape;
        write!(
            f,
            "raw modes #: {} ; null-spaced modes #: {}",
            self.raw_modes.len() / ns,
            self.modes.len() / ns
        )
    }
}

impl SegmentSingularModes {
    pub fn new(
        mode_nodes: Vec<Vec<f64>>,
        actuator_nodes: Vec<Vec<f64>>,
        raw_modes: Vec<f64>,
        modes: Vec<f64>,
        mode_2_force: Vec<f64>,
        // singular_values: Vec<f64>,
        shape: (usize, usize),
    ) -> Self {
        Self {
            mode_nodes,
            actuator_nodes,
            raw_modes,
            modes,
            mode_2_force,
            // right_modes,
            // singular_values,
            shape,
        }
    }
    pub fn nodes_iter(&self) -> impl Iterator<Item = &[f64]> {
        self.mode_nodes.iter().map(|x| x.as_slice())
    }
    #[cfg(feature = "faer")]
    pub fn mat_ref(&self) -> faer::mat::MatRef<'_, f64> {
        let (ns, na) = self.shape;
        faer::mat::MatRef::from_column_major_slice(&self.raw_modes, ns, na)
    }
    #[cfg(feature = "faer")]
    pub fn mode2force_mat_ref(&self) -> faer::mat::MatRef<'_, f64> {
        let (_, na) = self.shape;
        let ns = self.mode_2_force.len() / na;
        faer::mat::MatRef::from_column_major_slice(&self.mode_2_force, na, ns)
    }
    pub fn mode2force(&self) -> nalgebra::DMatrix<f64> {
        let (_, na) = self.shape;
        let ns = self.mode_2_force.len() / na;
        nalgebra::DMatrix::from_column_slice(na, ns, &self.mode_2_force)
    }
    pub fn truncated_mode2force(&self, n_mode: usize) -> nalgebra::DMatrix<f64> {
        let (_, na) = self.shape;
        let ns = self.mode_2_force.len() / na;
        if n_mode < ns {
            nalgebra::DMatrix::from_column_slice(na, ns, &self.mode_2_force)
                .columns(0, n_mode)
                .clone_owned()
        } else {
            nalgebra::DMatrix::from_column_slice(na, ns, &self.mode_2_force)
        }
    }
    pub fn raw_modes_into_mat(&self) -> nalgebra::DMatrix<f64> {
        let (ns, na) = self.shape;
        nalgebra::DMatrix::from_column_slice(ns, na, &self.raw_modes)
    }
    pub fn zeroed_raw_modes_into_mat(&self, n_mode: usize) -> nalgebra::DMatrix<f64> {
        let (ns, na) = self.shape;
        if na < n_mode {
            nalgebra::DMatrix::from_column_slice(ns, na, &self.raw_modes).insert_columns(
                na,
                n_mode - na,
                0f64,
            )
        } else {
            nalgebra::DMatrix::from_column_slice(ns, na, &self.raw_modes)
        }
    }
    pub fn modes_into_mat(&self, n_mode: Option<usize>) -> nalgebra::DMatrix<f64> {
        let (ns, ..) = self.shape;
        if let Some(n) = n_mode {
            nalgebra::DMatrix::from_column_slice(ns, n, &self.modes[..ns * n])
        } else {
            nalgebra::DMatrix::from_column_slice(ns, self.modes.len() / ns, &self.modes)
        }
    }
    pub fn shape(&self) -> (usize, usize) {
        self.shape
    }
    pub fn raw_modes_iter(&self) -> impl Iterator<Item = &f64> {
        self.raw_modes.iter()
    }
}

/// Null space of M1 singular modes
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
pub enum NullSpace {
    /// M1 rigid body motions
    Rbm,
    /// M1 rigid body & hardpoints motions
    RbmHp,
}

impl Display for NullSpace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NullSpace::Rbm => write!(f, "RBMs"),
            NullSpace::RbmHp => write!(f, "RBMs & Hardpoints"),
        }
    }
}

fn expected_fem<'de, D>(d: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let s = String::deserialize(d)?;
    let Ok(fem_id) = env::var("FEM_REPO").map(|v| {
        Path::new(&v)
            .file_name()
            .expect("invalid name for FEM folder")
            .to_string_lossy()
            .into_owned()
    }) else {
        return Ok(s);
    };
    if s != fem_id {
        return Err(serde::de::Error::custom(format!(
            r#"SingularModes FEM ID mismatch: expected "{fem_id}", found "{s}""#
        )));
    }
    Ok(s)
}
/// M1 singular modes (aka bending modes)
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct SingularModes {
    #[serde(deserialize_with = "expected_fem")]
    fem_id: String,
    modes: Vec<SegmentSingularModes>,
    null_space: NullSpace,
}

impl Display for SingularModes {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "M1 singular modes from {}, both raw & with {} null-space",
            self.fem_id, self.null_space
        )?;
        for (i, segment) in self.modes.iter().enumerate() {
            writeln!(f, " . {}: {}", i + 1, segment)?;
        }
        Ok(())
    }
}

impl Deref for SingularModes {
    type Target = [SegmentSingularModes];

    fn deref(&self) -> &Self::Target {
        self.modes.as_slice()
    }
}
impl SingularModes {
    pub fn new() -> Result<Self, env::VarError> {
        Ok(Self {
            fem_id: env::var("FEM_REPO").map(|v| {
                Path::new(&v)
                    .file_name()
                    .expect("invalid name for FEM folder")
                    .to_string_lossy()
                    .into_owned()
            })?,
            modes: Vec::new(),
            null_space: NullSpace::Rbm,
        })
    }
    pub fn push(&mut self, segment: SegmentSingularModes) {
        self.modes.push(segment);
    }
    pub fn modes_into_mat(&self, n_mode: Option<usize>) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter()
            .map(|segment| segment.modes_into_mat(n_mode))
            .collect()
    }
    pub fn raw_modes_into_mat(&self) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter()
            .map(|segment| segment.raw_modes_into_mat())
            .collect()
    }
    pub fn zeroed_raw_modes_into_mat(&self, n_mode: usize) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter()
            .map(|segment| segment.zeroed_raw_modes_into_mat(n_mode))
            .collect()
    }
    pub fn mode2force(&self) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter().map(|segment| segment.mode2force()).collect()
    }
    pub fn truncated_mode2force(&self, n_mode: usize) -> Vec<nalgebra::DMatrix<f64>> {
        self.iter()
            .map(|segment| segment.truncated_mode2force(n_mode))
            .collect()
    }
}

#[cfg(feature = "filing")]
impl interface::filing::Codec for SingularModes {}
