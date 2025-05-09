use serde::{Deserialize, Serialize};

/// List of  all the CFD wind loads
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum WindLoads {
    TopEnd,
    M2Segments,
    M2Baffle,
    Trusses,
    M1Baffle,
    MirrorCovers,
    LaserGuideStars,
    CRings,
    GIR,
    //LPA,
    Platforms,
    M1Segments,
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
            M1Segments => (1..=7).map(|i| format!("M1_{i}")).collect(),
            M2Segments => (1..=7).map(|i| format!("M2seg{i}")).collect(),
            TopEnd => vec![String::from("Topend")],
            M2Baffle => vec![String::from("M2Baffle")],
            Trusses => (1..=3)
                .map(|i| format!("Tup{i}"))
                .chain((1..=3).map(|i| format!("Tbot{i}")))
                .chain((1..=3).map(|i| format!("arm{i}")))
                .collect(),
            M1Baffle => vec![String::from("M1Baffle")],
            //LPA => vec![String::from("M1level")],
            LaserGuideStars => (1..=3).map(|i| format!("LGSS{i}")).collect(),
            CRings => [
                "CringL",
                "CringR",
                "Cring_strL",
                "Cring_strR",
                "Cring_strF",
                "Cring_strB",
            ]
            .into_iter()
            .map(|x| x.into())
            .collect(),
            GIR => vec!["GIR".into()],
            Platforms => vec!["platform".into()],
        }
    }
    /// Returns a pattern to match against the FEM CFD_202110_6F input
    pub fn fem(&self) -> Vec<String> {
        use WindLoads::*;
        match self {
            MirrorCovers => vec![String::from("mirror cover")],
            M1Segments => (1..=7).map(|i| format!("M1-S{i} unit")).collect(),
            M2Segments => (1..=7).map(|i| format!("M2 cell {i}.")).collect(),
            TopEnd => vec![String::from("Top-End")],
            M2Baffle => vec![String::from("M2 baffle unit")],
            Trusses => ["Upper truss", "Lower truss", "Focus Assembly Arm"]
                .into_iter()
                .map(|x| x.into())
                .collect(),
            M1Baffle => vec![String::from("Baffle protruding")],
            //LPA => vec![String::from("LPA")],
            LaserGuideStars => vec![String::from("Laser Guide Star")],
            CRings => [
                "C-Ring under M1 segments 5 and 6",
                "C-Ring under M1 segments 2 and 3",
                "C-Ring below M1 cells 5 and 6",
                "C-Ring below M1 cells 2 and 3",
                "between C-Rings below M1 cell 4",
                "between C-Rings below M1 cell 1",
            ]
            .into_iter()
            .map(|x| x.into())
            .collect(),
            GIR => vec!["GIR".into()],
            Platforms => vec!["Instrument, OSS mid-level, and Auxiliary Platforms".into()],
        }
    }
}
