use std::{
    fmt::Display,
    ops::{Deref, DerefMut},
};

use serde::{Deserialize, Serialize};

use super::MirrorMode;

#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct MixedMirrorMode(Vec<MirrorMode>);
impl Deref for MixedMirrorMode {
    type Target = [MirrorMode];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for MixedMirrorMode {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Display for MixedMirrorMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:}",
            self.iter()
                .map(|x| x.to_string())
                .collect::<Vec<_>>()
                .join(",")
        )
    }
}

impl From<Vec<MirrorMode>> for MixedMirrorMode {
    fn from(value: Vec<MirrorMode>) -> Self {
        Self(value)
    }
}
