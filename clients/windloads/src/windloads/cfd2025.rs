use serde::{Deserialize, Serialize};

/// List of  all the CFD wind loads
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum WindLoads {
    TopEnd,
    M2Segments,
    M2Baffle,
    Trusses,
    PrimeFocusArms,
    M1Baffle,
    MirrorCovers,
    LaserGuideStars,
    CRings,
    CRingTrusses,
    GIR,
    Platforms,
    M1Cells,
    M1Segments,
    CranePosY,
    CraneNegY,
    CableTrusses,
}
impl WindLoads {
    /// Returns the names of the CFD monitors
    pub fn keys(&self) -> Vec<String> {
        use WindLoads::*;
        match self {
            MirrorCovers => (1..=6)
                .map(|i| format!("M1cov{}", i))
                .chain((1..=6).map(|i| format!("M1covin{}", i)))
                .collect(),
            M1Cells => (1..=7)
                .map(|i| format!("M1c_{i}"))
                .chain(Some("M1p_+X".to_string()))
                .chain((2..7).map(|i| format!("M1p_{i}")))
                .collect(),
            M1Segments => (1..=7).map(|i| format!("M1s_{i}")).collect(),
            // M2Cells => (1..=7).map(|i| format!("M2seg{i}")).collect(),
            M2Segments => (1..=7).map(|i| format!("M2seg{i}")).collect(),
            TopEnd => vec![String::from("Topend")],
            M2Baffle => vec![String::from("M2baffle")],
            Trusses => (1..=3)
                .map(|i| format!("Tup{i}"))
                .chain((1..=3).map(|i| format!("Tbot{i}")))
                .collect(),
            PrimeFocusArms => (2..=3).map(|i| format!("arm{i}")).collect(),
            M1Baffle => vec![String::from("M1Baffle")],
            //LPA => vec![String::from("M1level")],
            LaserGuideStars => (1..=3).map(|i| format!("LGSS{i}")).collect(),
            CRings => ["Cring+X", "Cring-X"]
                .into_iter()
                .map(|x| x.into())
                .collect(),
            CRingTrusses => ["Cring_str+X", "Cring_str+Y", "Cring_str-X", "Cring_str-Y"]
                .into_iter()
                .map(|x| x.into())
                .collect(),
            GIR => vec!["GIR".into(), "GIR_GCLEF".into()],
            Platforms => [
                "plat+Xlo", "plat+Xup", "plat+Ylo", "plat+Yup", "plat-Xlo", "plat-Xup", "plat-Ylo",
                "plat-Yup",
            ]
            .into_iter()
            .map(|x| x.to_string())
            .collect(),
            CranePosY => vec!["crane+Y".to_string()],
            CraneNegY => vec!["crane-Y".to_string()],
            CableTrusses => [
                "cabletruss1_bot",
                "cabletruss1_up",
                "cabletruss2_bot",
                "cabletruss2_up",
                "cabletruss3_bot",
                "cabletruss3_up",
            ]
            .into_iter()
            .map(|x| x.to_string())
            .collect(),
        }
    }
    /// Returns a pattern to match against the FEM CFD_202110_6F input
    pub fn fem(&self) -> Vec<String> {
        use WindLoads::*;
        match self {
            MirrorCovers => vec![String::from("mirror cover")],
            M1Cells => vec![
                "the entire M1 cell c".to_string(),
                "LPA servicing and M1 in-situ wash platform".to_string(),
            ],
            M1Segments => (1..=7).map(|i| format!("M1-S{i} unit")).collect(),
            // M2Cells => (1..=7).map(|i| format!("M2 cell {i}.")).collect(),
            M2Segments => (1..=7).map(|i| format!("M2-S{i} unit")).collect(),
            TopEnd => vec![String::from("Top-End")],
            M2Baffle => vec![String::from("M2 baffle unit")],
            Trusses => ["Upper truss", "Lower truss"]
                .into_iter()
                .map(|x| x.into())
                .collect(),
            PrimeFocusArms => vec!["Focus Assembly Arm".to_string()],
            M1Baffle => vec![String::from("Baffle protruding")],
            //LPA => vec![String::from("LPA")],
            LaserGuideStars => (1..=3).map(|i| format!("Laser Guide Star {i}")).collect(),
            CRings => [
                "C-Ring under M1 segments 2 and 3",
                "C-Ring under M1 segments 5 and 6",
            ]
            .into_iter()
            .map(|x| x.into())
            .collect(),
            CRingTrusses => [
                "Truss on the outside of C-Ring below M1 cells 2 and 3",
                "Truss between C-Rings below M1 cell 1",
                "Truss on the outside of C-Ring below M1 cells 5 and 6",
                "Truss between C-Rings below M1 cell 4",
            ]
            .into_iter()
            .map(|x| x.into())
            .collect(),
            GIR => vec!["GIR".into()],
            Platforms => vec!["Instrument, OSS mid-level and auxiliary platforms".into()],
            CranePosY => vec!["Crane Assembly on the +Y".to_string()],
            CraneNegY => vec!["Crane Assembly on the -Y".to_string()],
            CableTrusses => vec!["Cables on the".to_string()],
        }
    }
}
