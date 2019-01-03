use config::*;
use direction::*;
use error::*;
use grid::*;
use metadata::*;
use num_traits::{NumCast, Zero};
use search::*;
use std::ops::{Add, Mul};

fn manhatten_distance(a: Coord, b: Coord) -> i32 {
    (a.x - b.x).abs() + (a.y - b.y).abs()
}

fn diagonal_distance<Cost>(a: Coord, b: Coord, weights: &HeuristicDirectionWeights<Cost>) -> Cost
where
    Cost: Copy + Add<Cost, Output = Cost> + Mul<Cost, Output = Cost> + PartialOrd<Cost> + NumCast,
{
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    let (cardinal, ordinal) = if dx < dy {
        (dy - dx, dx)
    } else {
        (dx - dy, dy)
    };

    let cardinal: Cost = NumCast::from(cardinal).expect("Failed to cast to Cost");
    let ordinal: Cost = NumCast::from(ordinal).expect("Failed to cast to Cost");

    let cardinal: Cost = cardinal * weights.cardinal;
    let ordinal: Cost = ordinal * weights.ordinal;

    cardinal + ordinal
}

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + NumCast + Zero> SearchContext<Cost> {
    pub fn astar_cardinal_manhatten_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: CostGrid<Cost = Cost>,
    {
        let heuristic_fn =
            |a, b| NumCast::from(manhatten_distance(a, b)).expect("Failed to cast to Cost");

        self.search_general(
            grid,
            start,
            goal,
            DirectionsCardinal,
            heuristic_fn,
            config,
            path,
        )
    }
}

#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy)]
pub struct HeuristicDirectionWeights<Cost> {
    pub cardinal: Cost,
    pub ordinal: Cost,
}

impl<Cost> HeuristicDirectionWeights<Cost> {
    pub fn new(cardinal: Cost, ordinal: Cost) -> Self {
        Self { cardinal, ordinal }
    }
}

impl<Cost> SearchContext<Cost>
where
    Cost: Copy
        + Add<Cost, Output = Cost>
        + Mul<Cost, Output = Cost>
        + PartialOrd<Cost>
        + NumCast
        + Zero,
{
    pub fn astar_diagonal_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        weights: HeuristicDirectionWeights<Cost>,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: CostGrid<Cost = Cost>,
    {
        let heuristic_fn = |a, b| diagonal_distance(a, b, &weights);
        self.search_general(grid, start, goal, Directions, heuristic_fn, config, path)
    }
}
