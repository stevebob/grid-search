#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
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
