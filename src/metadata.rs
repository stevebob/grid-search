#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SearchMetadata<C> {
    pub num_nodes_visited: usize,
    pub cost: C,
    pub length: usize,
}

#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DistanceMapMetadata {
    pub num_nodes_visited: usize,
}
