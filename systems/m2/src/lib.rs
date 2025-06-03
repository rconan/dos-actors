/*!
# M2 Assembly Control Systems
 */

#[cfg(topend = "ASM")]
mod asms;
#[cfg(topend = "ASM")]
pub use asms::*;
#[cfg(topend = "FSM")]
mod fsms;
#[cfg(topend = "FSM")]
pub use fsms::*;

#[cfg(feature = "serde")]
pub mod nodes;

#[derive(Debug, thiserror::Error)]
pub enum M2Error {
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

#[cfg(topend = "ASM")]
pub type M2<const R: usize> = ASMS<R>;
#[cfg(topend = "FSM")]
pub type M2<const R: usize> = FSMS<R>;

#[cfg(test)]
mod tests {
    use std::error::Error;

    use gmt_dos_actors::actorscript;
    use gmt_dos_clients::signals::Signals;
    use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};

    use super::*;

    // cargo t -r --lib -- tests::asms --exact --nocapture
    #[cfg(topend = "ASM")]
    #[tokio::test]
    async fn asms() -> Result<(), Box<dyn Error>> {
        let fem = gmt_fem::FEM::from_env()?;
        let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(1e3)
            .proportional_damping(2. / 100.)
            .including_asms(Some(vec![1, 2, 3, 4, 5, 6, 7]), None, None)?
            .use_static_gain_compensation()
            .build()?;
        Ok(())
    }

    // cargo t -r --lib -- tests::fsms --exact --nocapture
    #[cfg(topend = "FSM")]
    #[tokio::test]
    async fn fsms() -> Result<(), Box<dyn Error>> {
        use gmt_dos_clients_io::{
            gmt_fem::{inputs::MCM2PZTF, outputs::MCM2PZTD},
            gmt_m2::fsm::{M2FSMFsmCommand, M2FSMPiezoForces, M2FSMPiezoNodes},
        };
        let fem = gmt_fem::FEM::from_env()?;
        let plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(1e3)
            .proportional_damping(2. / 100.)
            .ins::<MCM2PZTF>()
            .outs::<MCM2PZTD>()
            .use_static_gain_compensation()
            .build()?;

        let m2 = M2::new()?;

        let pzt_cmd: Vec<_> = (1..8)
            .map(|sid| sid as f64 * 1e-6)
            .flat_map(|x| vec![x, -x - 1e-6, x + 1e-6])
            .collect();
        let cmd = Signals::from((pzt_cmd.as_slice(), 1000));

        actorscript!(
            1: cmd[M2FSMFsmCommand] -> {m2}[M2FSMPiezoForces] -> plant[M2FSMPiezoNodes]${42}! -> {m2}
        );

        let log = &mut *model_logging_1.lock().await;
        let data: Vec<_> = log
            .iter("M2FSMPiezoNodes")?
            .map(|data: Vec<f64>| data.chunks(2).map(|x| x[1] - x[0]).collect::<Vec<_>>())
            .collect();
        let cmd_err: Vec<_> = pzt_cmd
            .iter()
            .zip(data.last().unwrap())
            .map(|(x, y)| x - y)
            .collect();
        let rss_err =
            1e6 * (cmd_err.into_iter().map(|x| x * x).sum::<f64>() / pzt_cmd.len() as f64).sqrt();
        assert!(dbg!(rss_err) < 1e-3);

        #[cfg(feature = "complot")]
        {
            let _ = data
                .into_iter()
                .enumerate()
                .map(|(i, data)| {
                    (
                        i as f64 * 1e-3,
                        data.into_iter().map(|x| x * 1e6).collect::<Vec<_>>(),
                    )
                })
                .collect::<complot::Plot>();
        }
        Ok(())
    }
}
