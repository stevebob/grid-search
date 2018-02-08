use std::ops::Add;
use num_traits::Zero;
use direction::*;
use grid_2d::*;
use grid::*;
use error::*;
use metadata::*;
use search::*;

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub fn search<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        self.search_general(grid, start, goal, directions, |_, _| Zero::zero(), path)
    }
}
