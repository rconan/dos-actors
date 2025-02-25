use nalgebra as na;

/// Builder for the ASMS control system
#[derive(Debug, Clone)]
pub struct AsmsBuilder<'a, const R: usize> {
    pub(crate) gain: Vec<na::DMatrix<f64>>,
    pub(crate) modes: Option<Vec<na::DMatrixView<'a, f64>>>,
}

impl<'a, const R: usize> AsmsBuilder<'a, R> {
    /// Sets the matrices of the ASM modal shapes
    ///
    /// The control of the ASMS is converted to modal control meaning that the
    /// mirror inputs are the modes coefficients instead of the voice coils
    /// displacements.
    ///
    /// The matrices must have 675 rows, it panics otherwise.
    pub fn modes(mut self, modes: Vec<na::DMatrixView<'a, f64>>) -> Self {
        for mode in &modes {
            assert_eq!(mode.nrows(), 675);
        }
        self.modes = Some(modes);
        self
    }
}
