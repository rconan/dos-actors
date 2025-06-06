//! # Calibration mode
//!
//! The mode of M1 or M2 that are calibrated.

mod segment;

pub use segment::{CalibrationMode, SegmentMode};
mod mirror;
pub use mirror::MirrorMode;
mod mixed;
pub use mixed::MixedMirrorMode;

/// Modes common interface
pub trait Modality: std::fmt::Debug + Clone {
    fn n_cols(&self) -> usize;
    fn fill(&self, iter: impl Iterator<Item = f64>) -> Vec<f64>;
}

impl Modality for CalibrationMode {
    fn n_cols(&self) -> usize {
        match self {
            CalibrationMode::RBM(tr_xyz) => tr_xyz.iter().filter_map(|&x| x).count(),
            &CalibrationMode::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => end_id.unwrap_or(n_mode) - start_idx,
            CalibrationMode::GlobalTipTilt(_) => 2,
            CalibrationMode::Mount { .. } => 2,
            _ => unimplemented!(r#""n_cols" not implemented for "CalibrationMode""#),
        }
    }
    fn fill(&self, iter: impl Iterator<Item = f64>) -> Vec<f64> {
        match self {
            CalibrationMode::RBM(tr_xyz) => {
                let mut out = vec![0.; 6];
                out.iter_mut()
                    .zip(tr_xyz)
                    .filter_map(|(out, v)| v.and_then(|_| Some(out)))
                    .zip(iter)
                    .for_each(|(out, e)| *out = e);
                out
            }
            &CalibrationMode::Modes {
                n_mode,
                start_idx,
                end_id,
                ..
            } => {
                let end = end_id.unwrap_or(n_mode);
                vec![0.; start_idx]
                    .into_iter()
                    .chain(iter.take(end - start_idx))
                    .chain(vec![0.; n_mode - end])
                    .collect()
            }
            _ => iter.collect(),
        }
    }
}

impl Modality for SegmentMode {
    fn n_cols(&self) -> usize {
        self.rbm.n_cols() + self.modes.n_cols()
    }

    fn fill(&self, mut iter: impl Iterator<Item = f64>) -> Vec<f64> {
        let mut rbm = self.rbm.fill(iter.by_ref());
        let modes = self.modes.fill(iter);
        rbm.extend(modes);
        rbm
    }
}
impl SegmentMode {
    pub fn fill_split(&self, mut iter: impl Iterator<Item = f64>) -> (Vec<f64>, Vec<f64>) {
        (self.rbm.fill(iter.by_ref()), self.modes.fill(iter))
    }
}
impl Modality for MirrorMode {
    fn n_cols(&self) -> usize {
        self.iter()
            .filter_map(|segment| segment.as_ref().map(|s| s.n_cols()))
            .sum()
    }
    fn fill(&self, mut iter: impl Iterator<Item = f64>) -> Vec<f64> {
        self.iter()
            .filter_map(|segment| {
                segment.as_ref().map(|s| match s {
                    CalibrationMode::RBM(tr_xyz) => {
                        let mut out = vec![0.; 6];
                        out.iter_mut()
                            .zip(tr_xyz)
                            .filter_map(|(out, v)| v.and_then(|_| Some(out)))
                            .zip(iter.by_ref())
                            .for_each(|(out, e)| *out = e);
                        out
                    }
                    &CalibrationMode::Modes {
                        n_mode,
                        start_idx,
                        end_id,
                        ..
                    } => {
                        let end = end_id.unwrap_or(n_mode);
                        vec![0.; start_idx]
                            .into_iter()
                            .chain(iter.by_ref().take(end - start_idx))
                            .chain(vec![0.; n_mode - end])
                            .collect()
                    }
                    _ => iter.by_ref().collect(),
                })
            })
            .flatten()
            .collect()
    }
}

impl Modality for MixedMirrorMode {
    fn n_cols(&self) -> usize {
        self.iter().map(|mirror| mirror.n_cols()).sum()
    }
    fn fill(&self, mut iter: impl Iterator<Item = f64>) -> Vec<f64> {
        self.iter()
            .flat_map(|mirror| mirror.fill(iter.by_ref()))
            .collect()
    }
}

/* pub trait MirrorModality: Modality {}

impl MirrorModality for MirrorMode {}
impl MirrorModality for MixedMirrorMode {} */
