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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BfsConfig {
    pub allow_solid_start: bool,
    pub max_depth: usize,
}

impl Default for BfsConfig {
    fn default() -> Self {
        Self {
            allow_solid_start: true,
            max_depth: ::std::usize::MAX,
        }
    }
}
