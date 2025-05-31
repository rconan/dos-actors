use std::{io, time::Duration};

use rmp_serde::{Deserializer, Serializer};
use serde::{Deserialize, Serialize};

use crate::{
    DcsError,
    mount_trajectory::MountTrajectory,
    pk_sys_types::{ImMountDemands, ImMountFeedback},
};

type Result<T> = std::result::Result<T, DcsError>;

/// Interface definition for data exchanged between the DCS and the OCS
pub trait DcsData: Default {
    /// Decode data from the OCS
    fn decode(&mut self, _bytes: &mut [u8]) -> Result<()> {
        Ok(())
    }
    /// Encode data to the OCS
    fn encode(&mut self) -> Result<Vec<u8>> {
        Ok(Vec::new())
    }
}

impl DcsData for MountTrajectory {
    fn decode(&mut self, bytes: &mut [u8]) -> Result<()> {
        let cur = io::Cursor::new(bytes);
        let mut de = Deserializer::new(cur);
        let ocs_data: ImMountDemands = Deserialize::deserialize(&mut de)?;
        log::debug!("Received OCS data: {:#?}", ocs_data);

        self.azimuth
            .push_back(ocs_data.azimuth_trajectory[0].position);
        self.elevation
            .push_back(ocs_data.elevation_trajectory[0].position);
        self.gir.push_back(ocs_data.gir_trajectory[0].position);
        self.tai.push_back(Duration::from_nanos(
            ocs_data.azimuth_trajectory[0].tai as u64,
        ));
        Ok(())
    }

    fn encode(&mut self) -> Result<Vec<u8>> {
        let ocs_data = ImMountFeedback::new(
            vec![self.azimuth.pop_front().unwrap_or_default()],
            vec![self.elevation.pop_front().unwrap_or_default()],
            vec![self.gir.pop_front().unwrap_or_default()],
            vec![self.tai.pop_front().unwrap_or_default().as_nanos() as f64],
        );
        log::debug!("Sending OCS data: {:#?}", ocs_data);
        let mut buffer = Vec::new();
        ocs_data.serialize(&mut Serializer::new(&mut buffer).with_struct_map())?;
        Ok(buffer)
    }
}
