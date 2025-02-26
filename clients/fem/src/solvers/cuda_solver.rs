use fem_cuda_solver::{mode_state_space, state_space};

use crate::{
    solvers::{Exponential, ExponentialMatrix, Solver},
    DiscreteModalSolver, DiscreteStateSpace, StateSpaceError,
};

impl From<&Exponential> for mode_state_space {
    fn from(so: &Exponential) -> Self {
        mode_state_space {
            x0: so.x.0,
            x1: so.x.1,
            a0: so.q.0,
            a1: so.q.2,
            a2: so.q.1,
            a3: so.q.3,
            b2: so.m.0,
            b3: so.m.1,
        }
    }
}

pub struct ModeStateSpace(mode_state_space, (Vec<f64>, Vec<f64>));

impl From<Exponential> for ModeStateSpace {
    fn from(so: Exponential) -> Self {
        ModeStateSpace((&so).into(), (so.b, so.c))
    }
}
impl From<&ExponentialMatrix> for mode_state_space {
    fn from(so: &ExponentialMatrix) -> Self {
        mode_state_space {
            x0: so.x.0,
            x1: so.x.1,
            a0: so.phi.0,
            a1: so.phi.2,
            a2: so.phi.1,
            a3: so.phi.3,
            b2: so.gamma.0,
            b3: so.gamma.1,
        }
    }
}
impl From<ExponentialMatrix> for ModeStateSpace {
    fn from(so: ExponentialMatrix) -> Self {
        ModeStateSpace((&so).into(), (so.b, so.c))
    }
}

/// State space model using [fem_cuda_solver]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Default, Clone)]
#[allow(dead_code)]
pub struct CuStateSpace {
    #[cfg_attr(feature = "serde", serde(skip))]
    mss: Vec<mode_state_space>,
    #[cfg_attr(feature = "serde", serde(skip))]
    cu_ss: state_space,
    n_mode: usize,
    n_input: usize,
    n_output: usize,
    i2m_rows: Vec<f64>,
    m2o_cols: Vec<f64>,
}
impl CuStateSpace {
    /// Creates a new instance of [CuStateSpace]
    pub fn new<T>(second_orders: Vec<T>) -> Self
    where
        ModeStateSpace: From<T>,
    {
        let n_mode = second_orders.len();
        let (mut mss, (i2m_rows, m2o_cols)): (Vec<_>, (Vec<_>, Vec<_>)) = second_orders
            .into_iter()
            .map(|so| ModeStateSpace::from(so))
            .map(|ModeStateSpace(data, bc)| (data, bc))
            .unzip();
        let n_input = i2m_rows[0].len();
        let n_output = m2o_cols[0].len();
        let mut i2m_rows: Vec<_> = i2m_rows.into_iter().flatten().collect();
        let mut m2o_cols: Vec<_> = m2o_cols.into_iter().flatten().collect();
        let mut cu_ss = state_space::default();
        unsafe {
            cu_ss.build(
                n_mode as i32,
                mss.as_mut_ptr(),
                n_input as i32,
                i2m_rows.as_mut_ptr(),
                n_output as i32,
                m2o_cols.as_mut_ptr(),
            )
        };
        Self {
            mss,
            cu_ss,
            n_mode,
            n_input,
            n_output,
            i2m_rows,
            m2o_cols,
        }
    }
    /// Set the DC gain compensation matrix
    pub fn set_dc_gain_compensator(&mut self, dcg: &[f64]) {
        unsafe {
            self.cu_ss.dc_gain_compensator(dcg.as_ptr() as *mut _);
        }
    }
    /// Steps the state space model
    pub fn step(&mut self, u: &mut [f64], y: &mut [f64]) {
        unsafe {
            self.cu_ss.step(u.as_mut_ptr(), y.as_mut_ptr());
        }
    }
}

impl Drop for CuStateSpace {
    fn drop(&mut self) {
        unsafe { self.cu_ss.free() };
    }
}

impl Solver for CuStateSpace {
    fn n_input(&self) -> usize {
        self.n_input
    }
    fn n_output(&self) -> usize {
        self.n_output
    }
    fn get_b(&self) -> &[f64] {
        self.i2m_rows.as_slice()
    }
    fn get_c(&self) -> &[f64] {
        self.m2o_cols.as_slice()
    }
}

impl<T> DiscreteModalSolver<T>
where
    T: Solver + Default,
    ModeStateSpace: From<T>,
{
    /// Replace the CPU based solver with a GPU based solver
    pub fn with_cuda_solver(self) -> DiscreteModalSolver<CuStateSpace> {
        let Self {
            u,
            y,
            y_sizes,
            state_space,
            psi_dcg,
            psi_times_u,
            ins,
            outs,
            facesheet_nodes,
            m1_figure_nodes,
        } = self;
        let mut cu_ss = CuStateSpace::new(state_space);
        if let Some(dcg) = &psi_dcg {
            cu_ss.set_dc_gain_compensator(dcg.as_slice());
        }
        DiscreteModalSolver {
            u,
            y,
            y_sizes,
            state_space: vec![cu_ss],
            psi_dcg,
            psi_times_u,
            ins,
            outs,
            facesheet_nodes,
            m1_figure_nodes,
        }
    }
}

impl Iterator for DiscreteModalSolver<CuStateSpace> {
    type Item = ();

    fn next(&mut self) -> Option<Self::Item> {
        self.state_space
            .get_mut(0)
            .map(|ss| ss.step(&mut self.u, &mut self.y))
    }
}

impl<'a, T> DiscreteStateSpace<'a, T>
where
    T: Solver + Default,
    ModeStateSpace: From<T>,
{
    pub fn build(self) -> Result<DiscreteModalSolver<CuStateSpace>, StateSpaceError> {
        self.builder().map(|dsm| dsm.with_cuda_solver())
    }
}
impl<'a, S> TryFrom<DiscreteStateSpace<'a, S>> for DiscreteModalSolver<CuStateSpace>
where
    S: Solver + Default,
    ModeStateSpace: From<S>,
{
    type Error = StateSpaceError;

    fn try_from(dss: DiscreteStateSpace<'a, S>) -> std::result::Result<Self, Self::Error> {
        dss.build()
    }
}
