use crate::CS;
use serde::{Deserialize, Serialize};

#[cfg(any(
    cfd2021,
    all(feature = "cfd2021", not(cfd2025), not(feature = "cfd2025"))
))]
mod cfd2021;
#[cfg(any(
    cfd2021,
    all(feature = "cfd2021", not(cfd2025), not(feature = "cfd2025"))
))]
pub use cfd2021::WindLoads;
#[cfg(any(
    cfd2025,
    all(feature = "cfd2025", not(cfd2021), not(feature = "cfd2021"))
))]
mod cfd2025;
#[cfg(any(
    cfd2025,
    all(feature = "cfd2025", not(cfd2021), not(feature = "cfd2021"))
))]
pub use cfd2025::WindLoads;

/// CFD wind loads builder
///
/// Selects the wind loads to apply to the FEM
///
/// Per default, all the mount, M1 and M2 wind loads are selected
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WindLoadsBuilder {
    pub(crate) windloads: Vec<WindLoads>,
    pub(crate) m1_nodes: Option<Vec<(String, CS)>>,
    pub(crate) m2_nodes: Option<Vec<(String, CS)>>,
}
impl Default for WindLoadsBuilder {
    fn default() -> Self {
        Self::new().mount(None).m1_assembly().m2_assembly()
    }
}
impl WindLoadsBuilder {
    /// Creates a new empty CFD wind loads builder
    pub fn new() -> Self {
        Self {
            windloads: vec![],
            m1_nodes: None,
            m2_nodes: None,
        }
    }
    /// Selects the wind loads and filters the FEM
    ///
    /// The input index of the  FEM windloads is given by `loads_index`
    /// The default CFD wind loads are:
    ///  * CFD 2021:
    ///    * TopEnd,
    ///    * M2Baffle,
    ///    * Trusses,
    ///    * M1Baffle,
    ///    * MirrorCovers,
    ///    * LaserGuideStars,
    ///    * CRings,
    ///    * GIR,
    ///    * Platforms,    
    ///  * CFD 2025:
    ///    * TopEnd,
    ///    * M2Baffle,
    ///    * Trusses,
    ///    * PrimeFocusArms
    ///    * M1Baffle,
    ///    * MirrorCovers,
    ///    * LaserGuideStars,
    ///    * CRings,
    ///    * CRingTrusses,
    ///    * GIR,
    ///    * Platforms,
    ///    * CranePosY   
    ///    * CraneNegY
    ///    * CableTrusses  
    #[cfg(any(
        cfd2021,
        all(feature = "cfd2021", not(cfd2025), not(feature = "cfd2025"))
    ))]
    pub fn mount(mut self, loads: Option<Vec<WindLoads>>) -> Self {
        self.windloads = loads.unwrap_or(vec![
            WindLoads::TopEnd,
            WindLoads::M2Baffle,
            WindLoads::Trusses,
            WindLoads::M1Baffle,
            WindLoads::MirrorCovers,
            WindLoads::LaserGuideStars,
            WindLoads::CRings,
            WindLoads::GIR,
            WindLoads::Platforms,
        ]);
        self
    }
    #[cfg(any(
        cfd2025,
        all(feature = "cfd2025", not(cfd2021), not(feature = "cfd2021"))
    ))]
    pub fn mount(mut self, loads: Option<Vec<WindLoads>>) -> Self {
        self.windloads = loads.unwrap_or(vec![
            WindLoads::TopEnd,
            WindLoads::M2Baffle,
            WindLoads::Trusses,
            WindLoads::PrimeFocusArms,
            WindLoads::M1Baffle,
            WindLoads::MirrorCovers,
            WindLoads::LaserGuideStars,
            WindLoads::CRings,
            WindLoads::CRingTrusses,
            WindLoads::GIR,
            WindLoads::Platforms,
            WindLoads::CranePosY,
            WindLoads::CraneNegY,
            WindLoads::CableTrusses,
        ]);
        self
    }
    /// Requests M1 assembly (segments and cells) loads
    #[cfg(any(
        cfd2021,
        all(feature = "cfd2021", not(cfd2025), not(feature = "cfd2025"))
    ))]
    pub fn m1_assembly(mut self) -> Self {
        let m1_nodes: Vec<_> = WindLoads::M1Segments
            .keys()
            .into_iter()
            .zip((1..=7).map(|i| CS::M1S(i)))
            .map(|(x, y)| (x, y))
            .collect();
        self.m1_nodes = Some(m1_nodes);
        self
    }
    #[cfg(any(
        cfd2025,
        all(feature = "cfd2025", not(cfd2021), not(feature = "cfd2021"))
    ))]
    pub fn m1_assembly(mut self) -> Self {
        self.windloads.push(WindLoads::M1Cells);
        let m1_nodes: Vec<_> = WindLoads::M1Segments
            .keys()
            .into_iter()
            .zip((1..=7).map(|i| CS::M1S(i)))
            .map(|(x, y)| (x, y))
            .collect();
        self.m1_nodes = Some(m1_nodes);
        self
    }
    /// Requests M1 segments loads
    pub fn m1_segments(mut self) -> Self {
        let m1_nodes: Vec<_> = WindLoads::M1Segments
            .keys()
            .into_iter()
            .zip((1..=7).map(|i| CS::M1S(i)))
            .map(|(x, y)| (x, y))
            .collect();
        self.m1_nodes = Some(m1_nodes);
        self
    }
    /// Requests M2 assembly (segments and cells) loads
    pub fn m2_assembly(mut self) -> Self {
        let m2_nodes: Vec<_> = WindLoads::M2Segments
            .keys()
            .into_iter()
            .zip((1..=7).map(|i| CS::M2S(i)))
            .map(|(x, y)| (x, y))
            .collect();
        self.m2_nodes = Some(m2_nodes);
        self
    }
}
