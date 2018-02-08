#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SearchMetadata {
    pub num_nodes_visited: usize,
}
