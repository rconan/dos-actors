use std::{
    fmt::Debug,
    fs::File,
    ops::{Deref, DerefMut},
    path::Path,
};

use gmt_fem::{Switch, FEM};
use matio_rs::MatFile;
use nalgebra::{DMatrix, DMatrixView};
use serde::{Deserialize, Serialize};

use crate::{M2CtrlError, Result};

#[derive(Debug, Default, Clone)]
pub enum DataSource {
    MatVar {
        file_name: String,
        var_name: String,
    },
    MatFile {
        file_name: String,
        var_names: Vec<String>,
    },
    #[default]
    Fem,
    Bin(String),
}

impl From<String> for DataSource {
    fn from(value: String) -> Self {
        DataSource::Bin(value)
    }
}
impl From<(String, String)> for DataSource {
    fn from((file_name, var_name): (String, String)) -> Self {
        DataSource::MatVar {
            file_name,
            var_name,
        }
    }
}
impl From<(String, Vec<String>)> for DataSource {
    fn from((file_name, var_names): (String, Vec<String>)) -> Self {
        DataSource::MatFile {
            file_name,
            var_names,
        }
    }
}
pub struct Data {
    data: DMatrix<f64>,
}
impl From<Data> for DMatrix<f64> {
    fn from(value: Data) -> Self {
        /*         DMatrix::from_column_slice(
            value.nrows.unwrap_or(1),
            value.ncols.unwrap_or(value.data.len()),
            &value.data,
        ) */
        value.data
    }
}
impl From<Data> for Vec<f64> {
    fn from(value: Data) -> Self {
        value.data.as_slice().to_vec()
    }
}

impl DataSource {
    pub fn load(self, nrows: Option<usize>, ncols: Option<usize>) -> Result<Data> {
        match self {
            DataSource::MatVar {
                file_name,
                var_name,
            } => {
                log::info!("loading {var_name} from {file_name}");
                let data: DMatrix<f64> = MatFile::load(file_name)?.var(var_name)?;
                let nrows = nrows.unwrap_or(data.nrows());
                let ncols = ncols.unwrap_or(data.ncols());
                if data.nrows() < nrows || data.ncols() < ncols {
                    return Err(M2CtrlError::MatrixSizeMismatch(
                        (nrows, ncols),
                        data.shape(),
                    ));
                }
                if nrows < data.nrows() && ncols < data.ncols() {
                    let rows = data.rows(0, nrows);
                    let cols = rows.columns(0, ncols);
                    return Ok(Data { data: cols.into() });
                }
                if nrows < data.nrows() {
                    let rows = data.rows(0, nrows);
                    return Ok(Data { data: rows.into() });
                }
                if ncols < data.ncols() {
                    let cols = data.columns(0, ncols);
                    return Ok(Data { data: cols.into() });
                }
                Ok(Data { data })
            }
            _ => unimplemented!(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum StiffnessKind {
    Zonal,
    Modal,
}
impl<'a> From<&'a str> for StiffnessKind {
    fn from(value: &'a str) -> Self {
        match value {
            "Zonal" => StiffnessKind::Zonal,
            "Modal" => StiffnessKind::Modal,
            other => unimplemented!(r#"expected "Zonal" or "Modal", found {:}"#, other),
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SegmentCalibration {
    pub(crate) sid: u8,
    pub(crate) n_mode: usize,
    pub(crate) n_actuator: usize,
    pub(crate) stiffness: Vec<f64>,
    pub(crate) modes: DMatrix<f64>,
    // #[serde(skip)]
    modes_t: Option<DMatrix<f64>>,
}
impl SegmentCalibration {
    pub fn new<M, S>(
        sid: u8,
        n_mode: usize,
        n_actuator: usize,
        modes_src: M,
        stiffness_src: S,
        stiffness_kind: StiffnessKind,
        maybe_fem: Option<&mut FEM>,
    ) -> Result<Self>
    where
        M: Into<DataSource> + Clone,
        S: Into<DataSource> + Clone,
    {
        let modes: DMatrix<f64> = modes_src
            .into()
            .load(Some(n_actuator), Some(n_mode))?
            .into();
        let stiffness: Vec<f64> = match stiffness_src.clone().into() {
            DataSource::Fem => {
                log::info!("computing ASM stiffness from FEM");
                let fem = maybe_fem.unwrap();
                fem.switch_inputs(Switch::Off, None)
                    .switch_outputs(Switch::Off, None);

                let vc_f2d = fem
                    .switch_inputs_by_name(vec![format!("MC_M2_S{sid}_VC_delta_F")], Switch::On)
                    .and_then(|fem| {
                        fem.switch_outputs_by_name(
                            vec![format!("MC_M2_S{sid}_VC_delta_D")],
                            Switch::On,
                        )
                    })
                    .map(|fem| {
                        fem.reduced_static_gain()
                            .unwrap_or_else(|| fem.static_gain())
                    })?;

                fem.switch_inputs(Switch::On, None)
                    .switch_outputs(Switch::On, None);

                match stiffness_kind {
                    StiffnessKind::Modal => modes.transpose() * vc_f2d * &modes,
                    StiffnessKind::Zonal => vc_f2d,
                }
                .try_inverse()
                .map(|stiffness_mat| {
                    stiffness_mat
                        .row_iter()
                        .flat_map(|row| row.iter().cloned().collect::<Vec<f64>>())
                        .collect::<Vec<f64>>()
                })
                .ok_or_else(|| M2CtrlError::Stiffness)?
            }
            _ => stiffness_src.into().load(None, None)?.into(),
        };
        Ok(Self {
            sid,
            n_mode,
            n_actuator,
            stiffness: stiffness.into(),
            modes,
            modes_t: None,
        })
    }
    pub fn modes(&self) -> &DMatrix<f64> {
        &self.modes
    }
}

#[derive(Debug)]
pub struct CalibrationBuilder<'a> {
    n_mode: usize,
    n_actuator: usize,
    modes_src: DataSource,
    fem: &'a mut FEM,
    stiffness_kind: StiffnessKind,
}
impl<'a> CalibrationBuilder<'a> {
    pub fn stiffness(mut self, kind: &str) -> Self {
        self.stiffness_kind = kind.into();
        self
    }
    pub fn build(self) -> Result<Calibration> {
        let DataSource::MatFile {
        file_name,
        var_names,
    } = self.modes_src.into() else {
        return Err(M2CtrlError::DataSourceMatFile)
    };
        let mut segment_calibration = vec![];
        for sid in 1..=7 {
            let i = sid as usize - 1;
            let calibration = SegmentCalibration::new(
                sid,
                self.n_mode,
                self.n_actuator,
                DataSource::MatVar {
                    file_name: file_name.clone(),
                    var_name: var_names[i].clone(),
                },
                DataSource::Fem,
                self.stiffness_kind,
                Some(self.fem),
            )?;
            segment_calibration.push(calibration);
        }
        Ok(Calibration(segment_calibration))
    }
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Calibration(Vec<SegmentCalibration>);
impl Deref for Calibration {
    type Target = Vec<SegmentCalibration>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for Calibration {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl Calibration {
    pub fn builder<'a, M>(
        n_mode: usize,
        n_actuator: usize,
        modes_src: M,
        fem: &'a mut FEM,
    ) -> CalibrationBuilder<'a>
    where
        M: Into<DataSource> + Clone,
    {
        CalibrationBuilder {
            n_mode,
            n_actuator,
            modes_src: modes_src.into(),
            fem,
            stiffness_kind: StiffnessKind::Modal,
        }
    }
    /*     pub fn new<M>(n_mode: usize, n_actuator: usize, modes_src: M, fem: &mut FEM) -> Result<Self>
    where
        M: Into<DataSource> + Clone,
    {
        let DataSource::MatFile {
            file_name,
            var_names,
        } = modes_src.into() else {
            return Err(M2CtrlError::DataSourceMatFile)
        };
        let mut segment_calibration = vec![];
        for sid in 1..=7 {
            let i = sid as usize - 1;
            let calibration = SegmentCalibration::new(
                sid,
                n_mode,
                n_actuator,
                DataSource::MatVar {
                    file_name: file_name.clone(),
                    var_name: var_names[i].clone(),
                },
                DataSource::Fem,
                Some(fem),
            )?;
            segment_calibration.push(calibration);
        }
        Ok(Self(segment_calibration))
    } */
    pub fn save<P: AsRef<Path> + Debug>(&self, file_name: P) -> Result<&Self> {
        log::info!("saving ASMS FEM calibration to {:?}", file_name);
        let mut file = File::create(file_name.as_ref())?;
        bincode::serialize_into(&mut file, self)?;
        Ok(self)
    }
    pub fn load<P: AsRef<Path> + Debug>(file_name: P) -> Result<Self> {
        log::info!("loading ASMS FEM calibration from {:?}", file_name);
        let file = File::open(file_name.as_ref())?;
        let this: Self = bincode::deserialize_from(file)?;
        Ok(this)
    }
    pub fn modes(&self, sids: Option<Vec<u8>>) -> Vec<DMatrixView<f64>> {
        sids.unwrap_or(vec![1, 2, 3, 4, 5, 6, 7])
            .into_iter()
            .map(|sid| sid as usize - 1)
            .map(|i| self.0[i].modes.as_view())
            .collect()
    }
    pub fn transpose_modes(&mut self) -> &mut Self {
        self.0.iter_mut().for_each(|x| {
            x.modes_t.get_or_insert(x.modes.transpose());
        });
        self
    }
    pub fn modes_t(&self, sids: Option<Vec<u8>>) -> Option<Vec<DMatrixView<f64>>> {
        sids.unwrap_or(vec![1, 2, 3, 4, 5, 6, 7])
            .into_iter()
            .map(|sid| sid as usize - 1)
            .map(|i| self.0[i].modes_t.as_ref().map(|x| x.as_view()))
            .collect()
    }
    pub fn stiffness(&self, sid: u8) -> &[f64] {
        self.0[sid as usize - 1].stiffness.as_slice()
    }
}
