//! MOUNT CONTROL

use super::prelude::*;
use gmt_dos_clients_io::mount::{AverageMountEncoders, MountEncoders, MountTorques};

/* impl<S> Get<MountEncoders> for DiscreteModalSolver<S>
where
    S: Solver + Default,
{
    fn get(&self) -> Option<Vec<f64>> {
        let mut encoders = <DiscreteModalSolver<S> as Get<fem_io::OSSAzEncoderAngle>>::get(self)?;
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::OSSElEncoderAngle>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::OSSRotEncoderAngle>>::get(self)?.as_slice(),
        );
        Some(encoders)
    }
} */
impl<S> Write<MountEncoders> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<MountEncoders>> {
        let mut encoders = <DiscreteModalSolver<S> as Get<fem_io::OSSAzEncoderAngle>>::get(self)?;
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::OSSElEncoderAngle>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::OSSRotEncoderAngle>>::get(self)?.as_slice(),
        );
        Some(Data::new(encoders))
        //  <DiscreteModalSolver<S> as Get<MountEncoders>>::get(self)
        //     .map(|data| Arc::new(Data::new(data)))
    }
}

impl<S, const E: i32> Write<AverageMountEncoders<E>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<AverageMountEncoders<E>>> {
        let mut encoders = <DiscreteModalSolver<S> as Get<fem_io::OSSAzEncoderAngle>>::get(self)
            .map(|x| {
                let n = x.len() as f64;
                vec![x.into_iter().sum::<f64>() / n]
            })?;
        encoders.push(
            <DiscreteModalSolver<S> as Get<fem_io::OSSElEncoderAngle>>::get(self).map(|x| {
                let n = x.len() as f64;
                x.into_iter().sum::<f64>() / n
            })?,
        );
        encoders.push(
            <DiscreteModalSolver<S> as Get<fem_io::OSSRotEncoderAngle>>::get(self).map(|x| {
                let n = x.len() as f64;
                x.into_iter().sum::<f64>() / n
            })?,
        );
        Some(Data::new(if E != 0 {
            encoders
                .into_iter()
                .map(|x| x * 10f64.powi(-E))
                .collect::<Vec<f64>>()
        } else {
            encoders
        }))
    }
}

impl<S, const E: i32> Size<AverageMountEncoders<E>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        3
    }
}
/* impl<S: Solver + Default> Set<MountTorques> for DiscreteModalSolver<S> {
    fn set(&mut self, u: &[f64]) {
        let (azimuth, others) = u.split_at(12);
        <DiscreteModalSolver<S> as Set<fem_io::OSSAzDriveTorque>>::set(self, azimuth);
        let (elevation, gir) = others.split_at(4);
        <DiscreteModalSolver<S> as Set<fem_io::OSSElDriveTorque>>::set(self, elevation);
        <DiscreteModalSolver<S> as Set<fem_io::OSSRotDriveTorque>>::set(self, gir);
    }
} */
impl<S> Read<MountTorques> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn read(&mut self, data: Data<MountTorques>) {
        let n_azimuth = self
            .ins
            .iter()
            .find(|&x| {
                x.as_any()
                    .is::<crate::fem_io::SplitFem<fem_io::OSSAzDriveTorque>>()
            })
            .map(|io| io.len())
            .unwrap();
        let n_elevation = self
            .ins
            .iter()
            .find(|&x| {
                x.as_any()
                    .is::<crate::fem_io::SplitFem<fem_io::OSSElDriveTorque>>()
            })
            .map(|io| io.len())
            .unwrap();
        let (azimuth, others) = data.split_at(n_azimuth);
        <DiscreteModalSolver<S> as Set<fem_io::OSSAzDriveTorque>>::set(self, azimuth);
        let (elevation, gir) = others.split_at(n_elevation);
        <DiscreteModalSolver<S> as Set<fem_io::OSSElDriveTorque>>::set(self, elevation);
        <DiscreteModalSolver<S> as Set<fem_io::OSSRotDriveTorque>>::set(self, gir);
        //    <DiscreteModalSolver<S> as Set<MountTorques>>::set(self, &data);
    }
}
