use gmt_fem::FEM;
pub mod fem_io {
    pub use gmt_dos_clients_fem::fem_io::actors_inputs::*;
    pub use gmt_dos_clients_fem::fem_io::actors_outputs::*;
}
use gmt_dos_clients_fem::{Model, Switch};
use nalgebra as na;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default)]
pub struct Calibration {
    pub stiffness: f64,
    pub rbm_2_hp: Vec<M>,
    pub lc_2_cg: Vec<M>,
}

type M = nalgebra::Matrix6<f64>;

impl Calibration {
    pub fn new(fem: &mut FEM) -> Self {
        // Hardpoints stiffness
        log::info!("HARDPOINTS STIFFNESS");
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        let Some(gain) = fem
            .switch_input::<fem_io::OSSHarpointDeltaF>(Switch::On)
            .and_then(|fem| fem.switch_output::<fem_io::OSSHardpointD>(Switch::On))
            .and_then(|fem| fem.reduced_static_gain())
        else {
            panic!(
                r#"failed to derive hardpoints stiffness, check input "OSSHarpointDeltaF" and output "OSSHardpointD" or for the presence of the static gain matrix in the FEM model"#
            )
        };
        let mut stiffness = 0f64;
        for i in 0..7 {
            let rows = gain.rows(i * 12, 12);
            let segment = rows.columns(i * 6, 6);
            let cell = segment.rows(0, 6);
            let face = segment.rows(6, 6);
            stiffness += (face - cell).diagonal().map(f64::recip).mean();
        }
        stiffness /= 7f64;

        // RBM2HP
        log::info!("RBM 2 HP");
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        let Some(gain) = fem
            .switch_input::<fem_io::OSSHarpointDeltaF>(Switch::On)
            .and_then(|fem| fem.switch_output::<fem_io::OSSM1Lcl>(Switch::On))
            .and_then(|fem| fem.reduced_static_gain())
        else {
            panic!(
                r#"failed to derive hardpoints stiffness, check input "OSSHarpointDeltaF" and output "OSSM1Lcl""#
            )
        };
        let mut rbm_2_hp = vec![];
        for i in 0..7 {
            let rows = gain.rows(i * 6, 6);
            let segment = rows
                .columns(i * 6, 6)
                .try_inverse()
                .unwrap()
                .map(|x| x / stiffness);
            rbm_2_hp.push(na::Matrix6::from_column_slice(segment.as_slice()))
        }

        // LC2CG (include negative feedback)
        log::info!("LC 2 CG");
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        let Some(gain) = fem
            .switch_input::<fem_io::OSSM1Lcl6F>(Switch::On)
            .and_then(|fem| fem.switch_output::<fem_io::OSSHardpointD>(Switch::On))
            .and_then(|fem| fem.reduced_static_gain())
        else {
            panic!(
                r#"failed to derive hardpoints stiffness, check input "OSSM1Lcl6F" and output "OSSHardpointD""#
            )
        };
        let mut lc_2_cg = vec![];
        for i in 0..7 {
            let rows = gain.rows(i * 12, 12);
            let segment = rows.columns(i * 6, 6);
            let cell = segment.rows(0, 6);
            let face = segment.rows(6, 6);
            let mat = (cell - face).try_inverse().unwrap().map(|x| x / stiffness);
            lc_2_cg.push(na::Matrix6::from_column_slice(mat.as_slice()));
        }

        fem.switch_inputs(Switch::On, None)
            .switch_outputs(Switch::On, None);

        Self {
            stiffness,
            rbm_2_hp,
            lc_2_cg,
        }
    }
    #[cfg(feature = "serde")]
    pub fn save<P>(&self, path: P) -> Result<&Self, Box<dyn std::error::Error>>
    where
        P: AsRef<std::path::Path> + std::fmt::Debug,
    {
        let path =
            std::path::Path::new(&std::env::var("DATA_REPO").unwrap_or_else(|_| String::from(".")))
                .join(&path);
        log::info!("saving M1 FEM calibration to {:?}", path);
        let file = std::fs::File::create(path)?;
        let mut buffer = std::io::BufWriter::new(file);
        bincode::serde::encode_into_std_write(self, &mut buffer, bincode::config::standard())?;
        Ok(self)
    }
}

#[cfg(feature = "serde")]
impl TryFrom<String> for Calibration {
    type Error = Box<dyn std::error::Error>;
    fn try_from(path: String) -> Result<Self, Self::Error> {
        let path =
            std::path::Path::new(&std::env::var("DATA_REPO").unwrap_or_else(|_| String::from(".")))
                .join(&path);
        let file = std::fs::File::open(&path)?;
        log::info!("loading M1 FEM calibration from {:?}", path);
        let buffer = std::io::BufReader::new(file);
        let this: Self = bincode::serde::decode_from_reader(buffer, bincode::config::standard())?;
        Ok(this)
    }
}

#[cfg(feature = "serde")]
impl TryFrom<&str> for Calibration {
    type Error = Box<dyn std::error::Error>;
    fn try_from(path: &str) -> Result<Self, Self::Error> {
        let path =
            std::path::Path::new(&std::env::var("DATA_REPO").unwrap_or_else(|_| String::from(".")))
                .join(&path);
        let file = std::fs::File::open(&path)?;
        log::info!("loading M1 FEM calibration from {:?}", path);
        let buffer = std::io::BufReader::new(file);
        let this: Self = bincode::serde::decode_from_reader(buffer, bincode::config::standard())?;
        Ok(this)
    }
}

#[cfg(feature = "serde")]
impl TryFrom<std::path::PathBuf> for Calibration {
    type Error = Box<dyn std::error::Error>;
    fn try_from(path: std::path::PathBuf) -> Result<Self, Self::Error> {
        let path =
            std::path::Path::new(&std::env::var("DATA_REPO").unwrap_or_else(|_| String::from(".")))
                .join(&path);
        let file = std::fs::File::open(&path)?;
        log::info!("loading M1 FEM calibration from {:?}", path);
        let buffer = std::io::BufReader::new(file);
        let this: Self = bincode::serde::decode_from_reader(buffer, bincode::config::standard())?;
        Ok(this)
    }
}
