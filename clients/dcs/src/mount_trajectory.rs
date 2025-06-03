use std::{collections::VecDeque, sync::Arc, time::Duration};

use gmt_dos_clients_io::mount::{AverageMountEncoders, MountSetPoint};
use interface::{Read, UID, UniqueIdentifier, Update, Write};
use tai_time::{MonotonicTime, TaiClock};

use crate::DcsIO;

/// DCS mount trajectory data
///
/// Data structure where the OCS mount trajectory is collating
#[derive(Debug, Clone, Default)]
pub struct MountTrajectory {
    pub azimuth: VecDeque<f64>,
    pub elevation: VecDeque<f64>,
    pub gir: VecDeque<f64>,
    pub tai: VecDeque<Duration>,
}

#[derive(UID)]
#[uid(port = 7777)]
pub enum OcsMountTrajectory {}

impl DcsIO for OcsMountTrajectory {}

impl Update for MountTrajectory {}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Write<U> for MountTrajectory {
    fn write(&mut self) -> Option<interface::Data<U>> {
        vec![
            self.azimuth.pop_front(),
            self.elevation.pop_front(),
            self.gir.pop_front(),
        ]
        .into_iter()
        .collect::<Option<Vec<f64>>>()
        .map(|x| x.into())
    }
}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for MountTrajectory {
    #[cfg(not(target_os = "macos"))]
    fn read(&mut self, data: interface::Data<U>) {
        self.azimuth.push_back(data[0]);
        self.elevation.push_back(data[1]);
        self.gir.push_back(data[2]);
        let now = MonotonicTime::now();
        self.tai.push_back(Duration::from_nanos(
            now.as_secs() as u64 * 1_000_000_000 + now.subsec_nanos() as u64,
        ));
    }
    #[cfg(target_os = "macos")]
    fn read(&mut self, data: interface::Data<U>) {
        self.azimuth.push_back(data[0]);
        self.elevation.push_back(data[1]);
        self.gir.push_back(data[2]);
        let clock: TaiClock<0> = TaiClock::init_from_utc(0);
        let now = clock.now();
        self.tai.push_back(Duration::from_nanos(
            now.as_secs() as u64 * 1_000_000_000 + now.subsec_nanos() as u64,
        ));
    }
}

/// Differential mount trajectory data
///
/// The trajectory is relative to the zero position
/// given by the 1st elements of the mount trajectory
#[derive(Debug, Clone, Default)]
pub struct RelativeMountTrajectory {
    trajectory: Arc<Vec<f64>>,
    zero: Option<Box<RelativeMountTrajectory>>,
    encoders: Option<Arc<Vec<f64>>>,
}

#[derive(UID)]
#[uid(port = 7778)]
pub enum RelativeMountAxes {}

impl Update for RelativeMountTrajectory {}

impl Read<OcsMountTrajectory> for RelativeMountTrajectory {
    fn read(&mut self, data: interface::Data<OcsMountTrajectory>) {
        self.trajectory = data.into_arc();
    }
}

impl Read<AverageMountEncoders> for RelativeMountTrajectory {
    fn read(&mut self, data: interface::Data<AverageMountEncoders>) {
        self.encoders = Some(data.into_arc());
    }
}

impl Write<MountSetPoint> for RelativeMountTrajectory {
    fn write(&mut self) -> Option<interface::Data<MountSetPoint>> {
        Some(
            self.zero
                .get_or_insert(Box::new(self.clone()))
                .trajectory
                .iter()
                .zip(self.trajectory.iter())
                .map(|(z, t)| t - z)
                .collect::<Vec<f64>>()
                .into(),
        )
    }
}

#[derive(UID)]
#[uid(port = 7779)]
pub enum ImMountTrajectory {}

impl DcsIO for ImMountTrajectory {}

impl Write<ImMountTrajectory> for RelativeMountTrajectory {
    fn write(&mut self) -> Option<interface::Data<ImMountTrajectory>> {
        log::info!("Writing IM mount trajectory");
        match (self.zero.as_ref(), self.encoders.as_ref()) {
            (Some(z), Some(e)) => Some(
                z.trajectory
                    .iter()
                    .zip(e.iter())
                    .map(|(z, e)| e + z)
                    .collect::<Vec<f64>>()
                    .into(),
            ),
            _ => Some(vec![0.; 3].into()),
        }
    }
}

// pub struct Absolute<T: UniqueIdentifier>(PhantomData<T>);
// impl<T: UniqueIdentifier> UniqueIdentifier for Absolute<T> {
//     type DataType = T::DataType;
//     const PORT: u16 = T::PORT;
// }
// impl<U: UniqueIdentifier> Read<Absolute<U>> for RelativeMountTrajectory {
//     fn read(&mut self, data: interface::Data<U>) {
//         todo!()
//     }
// }

/* #[derive(UID)]
#[uid(port = 7779)]
pub enum AbsoluteMountAxes {}

impl Write<AbsoluteMountAxes> for RelativeMountTrajectory {
    fn write(&mut self) -> Option<interface::Data<AbsoluteMountAxes>> {
        Some(
            self.zero
                .get_or_insert(Box::new(self.clone()))
                .trajectory
                .iter()
                .zip(self.trajectory.iter())
                .map(|(z, t)| t + z)
                .collect::<Vec<f64>>()
                .into(),
        )
    }
}
 */
