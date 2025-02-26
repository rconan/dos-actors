use crate::{
    actors_interface::RbmRemoval,
    fem_io::{GetIn, GetOut},
    solvers::{Exponential, ExponentialMatrix, Solver},
    DiscreteStateSpace,
};

use gmt_fem::{Result, FEM};
use interface::TimerMarker;
use nalgebra as na;
use rayon::prelude::*;
use std::{
    fmt,
    sync::Arc,
    thread::{self, JoinHandle},
};

impl<T: Solver + Default> TimerMarker for DiscreteModalSolver<T> {}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Default)]
pub struct PsiTimesU {
    data: Vec<f64>,
    #[cfg_attr(feature = "serde", serde(skip))]
    handler: Option<JoinHandle<na::DVector<f64>>>,
}
impl PsiTimesU {
    pub fn mul(&mut self, u: &[f64], mat: &Arc<na::DMatrix<f64>>) {
        let clone_mat = Arc::clone(mat);
        let vec_u = na::DVector::from_column_slice(u);
        self.handler = Some(thread::spawn(move || &*clone_mat * vec_u));
    }
    pub fn join(&mut self) -> &[f64] {
        self.handler.take().map(|h| {
            let y = h.join().unwrap();
            if self.data.is_empty() {
                self.data = y.as_slice().to_vec();
            } else {
                self.data.copy_from_slice(y.as_slice());
            }
        });
        self.data.as_slice()
    }
}

/// This structure represents the actual state space model of the telescope
///
/// The state space discrete model is made of several discrete 2nd order different equation solvers, all independent and solved concurrently
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Default)]
pub struct DiscreteModalSolver<T: Solver + Default> {
    /// Model input vector
    pub u: Vec<f64>,
    /// Model output vector
    pub y: Vec<f64>,
    pub y_sizes: Vec<usize>,
    /// vector of state models
    pub state_space: Vec<T>,
    /// Static gain correction matrix
    pub psi_dcg: Option<Arc<na::DMatrix<f64>>>,
    /// Static gain correction vector
    pub psi_times_u: PsiTimesU,
    pub ins: Vec<Box<dyn GetIn>>,
    pub outs: Vec<Box<dyn GetOut>>,
    pub facesheet_nodes: Option<RbmRemoval>,
    pub m1_figure_nodes: Option<RbmRemoval>,
}
impl<T: Solver + Default> DiscreteModalSolver<T> {
    /*
      /// Serializes the model using [bincode](https://docs.rs/bincode/1.3.3/bincode/)
      fn dump(&self, filename: &str) -> REs {
      let file = File::create(filename)
      }
    */
    /// Returns the FEM state space builer
    pub fn from_fem(fem: FEM) -> DiscreteStateSpace<'static, T> {
        fem.into()
    }
    /// Loads a FEM model, saved in a second order form, from a zip archive file located in a directory given by the `FEM_REPO` environment variable
    pub fn from_env() -> Result<DiscreteStateSpace<'static, T>> {
        let fem = FEM::from_env()?;
        Ok(DiscreteModalSolver::from_fem(fem))
    }
}
impl Iterator for DiscreteModalSolver<Exponential> {
    type Item = ();
    fn next(&mut self) -> Option<Self::Item> {
        let n = self.y.len();
        //        match &self.u {
        let _u_ = &self.u;
        self.y = self
            .state_space
            .par_iter_mut()
            .fold(
                || vec![0f64; n],
                |mut a: Vec<f64>, m| {
                    a.iter_mut().zip(m.solve(_u_)).for_each(|(yc, y)| {
                        *yc += y;
                    });
                    a
                },
            )
            .reduce(
                || vec![0f64; n],
                |mut a: Vec<f64>, b: Vec<f64>| {
                    a.iter_mut().zip(b.iter()).for_each(|(a, b)| {
                        *a += *b;
                    });
                    a
                },
            );
        Some(())
    }
}

impl Iterator for DiscreteModalSolver<ExponentialMatrix> {
    type Item = ();
    fn next(&mut self) -> Option<Self::Item> {
        let n = self.y.len();
        //        match &self.u {
        let _u_ = &self.u;
        self.y = self
            .state_space
            .par_iter_mut()
            .fold(
                || vec![0f64; n],
                |mut a: Vec<f64>, m| {
                    a.iter_mut().zip(m.solve(_u_)).for_each(|(yc, y)| {
                        *yc += y;
                    });
                    a
                },
            )
            .reduce(
                || vec![0f64; n],
                |mut a: Vec<f64>, b: Vec<f64>| {
                    a.iter_mut().zip(b.iter()).for_each(|(a, b)| {
                        *a += *b;
                    });
                    a
                },
            );

        if let Some(psi_dcg) = &self.psi_dcg {
            let psi_times_u = self.psi_times_u.join();
            self.y
                .iter_mut()
                .zip(psi_times_u)
                .for_each(|(v1, v2)| *v1 += *v2);
            self.psi_times_u.mul(&self.u, psi_dcg);
            // let u_nalgebra = na::DVector::from_column_slice(&self.u);
            // self.psi_times_u = (psi_dcg * u_nalgebra).as_slice().to_vec();
        }

        Some(())
    }
}
impl<T: Solver + Default> fmt::Display for DiscreteModalSolver<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            r##"
DiscreteModalSolver:
 - inputs ({}):
{:}
 - outputs ({}):
{:}
 - {:} 2x2 state space models
"##,
            self.u.len(),
            self.ins
                .iter()
                .map(|x| x.fem_type())
                .collect::<Vec<String>>()
                .join("\n"),
            self.y.len(),
            self.outs
                .iter()
                .map(|x| x.fem_type())
                .collect::<Vec<String>>()
                .join("\n"),
            self.state_space.len(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fem_io::actors_inputs::OSSElDriveTorque;
    use crate::fem_io::actors_outputs::OSSElEncoderAngle;
    use gmt_fem::FEM;

    // #[test]
    // fn serde() {
    //     let state_space = {
    //         let fem = FEM::from_env().unwrap();
    //         DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
    //             .sampling(1e3)
    //             .max_eigen_frequency(0.1)
    //             .ins::<OSSElDriveTorque>()
    //             .outs::<OSSElEncoderAngle>()
    //             .build()
    //             .unwrap()
    //     };
    //     dbg!(&state_space);

    //     let json = serde_json::to_string(&state_space).unwrap();
    //     println!("{:#}", &json);
    //     let q: DiscreteModalSolver<ExponentialMatrix> = serde_json::from_str(&json).unwrap();
    //     dbg!(&q);
    // }
}
