use std::ops::Add;
use num::traits::Zero;
use direction::*;
use grid::*;
use error::*;
use metadata::*;
use config::*;
use search::*;

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub fn dijkstra<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        self.search_general(grid, start, goal, directions, |_, _| Zero::zero(), config, path)
    }
}
