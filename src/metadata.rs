#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SearchMetadata<C> {
    pub num_nodes_visited: usize,
    pub cost: C,
    pub length: usize,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DijkstraMapMetadata {
    pub num_nodes_visited: usize,
}
