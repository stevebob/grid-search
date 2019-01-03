#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SearchConfig {
    pub allow_solid_start: bool,
}

impl Default for SearchConfig {
    fn default() -> Self {
        Self {
            allow_solid_start: true,
        }
    }
}
