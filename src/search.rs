use std::collections::BinaryHeap;
use std::ops::{Add, Mul};
use std::cmp::Ordering;
use num_traits::{Zero, NumCast};
use direction::*;
use grid_2d::*;
use grid::*;
use error::*;
use metadata::*;
use path::{self, PathNode};

#[derive(Debug, Clone, Copy)]
struct SearchNode<Cost: Add<Cost> + PartialOrd<Cost>> {
    seen: u64,
    visited: u64,
    coord: Coord,
    from_parent: Option<Direction>,
    cost: Cost,
}

impl<Cost: Add<Cost> + PartialOrd<Cost> + Zero> From<Coord> for SearchNode<Cost> {
    fn from(coord: Coord) -> Self {
        Self {
            seen: 0,
            visited: 0,
            coord,
            from_parent: None,
            cost: Zero::zero(),
        }
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PathNode for SearchNode<Cost> {
    fn from_parent(&self) -> Option<Direction> {
        self.from_parent
    }
    fn coord(&self) -> Coord {
        self.coord
    }
}

#[derive(Debug, Clone)]
struct PriorityEntry<Cost: Add<Cost> + PartialOrd<Cost>> {
    node_index: usize,
    cost: Cost,
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PriorityEntry<Cost> {
    fn new(node_index: usize, cost: Cost) -> Self {
        Self { node_index, cost }
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PartialEq for PriorityEntry<Cost> {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PartialOrd for PriorityEntry<Cost> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.cost.partial_cmp(&self.cost)
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> Eq for PriorityEntry<Cost> {}

impl<Cost: Add<Cost> + PartialOrd<Cost>> Ord for PriorityEntry<Cost> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.partial_cmp(&self.cost).unwrap_or(
            Ordering::Equal,
        )
    }
}

#[derive(Debug, Clone)]
pub struct SearchContext<Cost: Add<Cost> + PartialOrd<Cost>> {
    seq: u64,
    priority_queue: BinaryHeap<PriorityEntry<Cost>>,
    node_grid: Grid<SearchNode<Cost>>,
}

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            seq: 0,
            node_grid: Grid::new_from_coord(width, height),
            priority_queue: BinaryHeap::new(),
        }
    }

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
        self.search_general(
            grid,
            start,
            goal,
            directions,
            |_, _| Zero::zero(),
            path,
        )
    }

    fn init<G>(
        &mut self,
        start: Coord,
        goal: Coord,
        grid: &G,
        path: &mut Vec<Direction>,
    ) -> Option<Result<SearchMetadata, Error>>
    where
        G: CostGrid<Cost = Cost>,
    {
        if let Some(solid) = grid.is_solid(start) {

            let index = self.node_grid.coord_to_index(start).expect(
                "SearchContext too small for grid",
            );

            if solid {
                return Some(Err(Error::StartSolid));
            };

            if start == goal {
                path.clear();
                return Some(Ok(Default::default()));
            }

            self.seq += 1;
            self.priority_queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            node.cost = Zero::zero();

            let entry = PriorityEntry::new(index, Zero::zero());
            self.priority_queue.push(entry);
        } else {
            return Some(Err(Error::StartOutsideGrid));
        };

        None
    }

    fn search_general<G, V, D, H>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        heuristic_fn: H,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        H: Fn(Coord, Coord) -> Cost,
    {
        if let Some(result) = self.init(start, goal, grid, path) {
            return result;
        }

        let goal_index = self.node_grid.coord_to_index(goal).expect(
            "SearchContext too small for grid",
        );

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {

            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                path::make_path(&self.node_grid, goal_index, path);
                return Ok(SearchMetadata { num_nodes_visited });
            }

            let (current_coord, current_cost) = {
                let node = &mut self.node_grid[current_entry.node_index];
                if node.visited == self.seq {
                    continue;
                }
                node.visited = self.seq;
                (node.coord, node.cost)
            };

            for d in directions {
                let direction = d.into();
                let offset: Coord = direction.into();
                let neighbour_coord = current_coord + offset;

                let neighbour_cost = if let Some(CostCell::Cost(cost)) = grid.cost(neighbour_coord, direction) {
                    cost
                } else {
                    continue;
                };

                let index = self.node_grid.coord_to_index(neighbour_coord).expect(
                    "SearchContext too small for grid",
                );

                let node = &mut self.node_grid[index];

                Self::update_node_and_priority_queue(
                    index,
                    current_cost,
                    neighbour_cost,
                    neighbour_coord,
                    direction,
                    self.seq,
                    &heuristic_fn,
                    goal,
                    node,
                    &mut self.priority_queue,
                );
            }
        }

        Err(Error::NoPath)
    }

    fn update_node_and_priority_queue<H>(
        index: usize,
        current_cost: Cost,
        neighbour_cost: Cost,
        neighbour_coord: Coord,
        direction: Direction,
        seq: u64,
        heuristic_fn: H,
        goal: Coord,
        neighbour_node: &mut SearchNode<Cost>,
        priority_queue: &mut BinaryHeap<PriorityEntry<Cost>>)
    where
        H: Fn(Coord, Coord) -> Cost,
    {
        let cost = current_cost + neighbour_cost;

        if neighbour_node.seen != seq || neighbour_node.cost > cost {
            neighbour_node.from_parent = Some(direction);
            neighbour_node.seen = seq;
            neighbour_node.cost = cost;

            let heuristic = cost + heuristic_fn(neighbour_coord, goal);
            let entry = PriorityEntry::new(index, heuristic);
            priority_queue.push(entry);
        }
    }
}

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + NumCast + Zero> SearchContext<Cost> {
    fn manhatten_distance(a: Coord, b: Coord) -> i32 {
        (a.x - b.x).abs() + (a.y - b.y).abs()
    }

    pub fn search_cardinal_manhatten_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
    {

        let heuristic_fn =
            |a, b| NumCast::from(Self::manhatten_distance(a, b)).expect("Failed to cast to Cost");

        self.search_general(
            grid,
            start,
            goal,
            DirectionsCardinal,
            heuristic_fn,
            path,
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HeuristicDirectionWeights<Cost: Add<Cost> + PartialOrd<Cost>> {
    pub cardinal: Cost,
    pub ordinal: Cost,
}

impl<Cost: Add<Cost> + PartialOrd<Cost> + Zero> HeuristicDirectionWeights<Cost> {
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
    fn diagonal_distance(a: Coord, b: Coord, weights: &HeuristicDirectionWeights<Cost>) -> Cost {
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

    pub fn search_diagonal_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        weights: HeuristicDirectionWeights<Cost>,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
    {
        let heuristic_fn = |a, b| Self::diagonal_distance(a, b, &weights);
        self.search_general(
            grid,
            start,
            goal,
            Directions,
            heuristic_fn,
            path,
        )
    }
}

impl SearchContext<f64> {
    fn octile_distance(a: Coord, b: Coord) -> f64 {
        let dx = (a.x - b.x).abs();
        let dy = (a.y - b.y).abs();
        let (cardinal, ordinal) = if dx > dy { (dx, dx) } else { (dy, dx) };

        const SQRT_2_MIN_1: f64 = ::std::f64::consts::SQRT_2 - 1.0;
        cardinal as f64 + ordinal as f64 * SQRT_2_MIN_1
    }

    fn jump<G>(
        grid: &G,
        coord: Coord,
        direction: Direction,
        goal: Coord,
    ) -> Option<(Coord, f64)>
    where
        G: CostGrid<Cost = f64>,
    {
        unimplemented!()
    }

    pub fn search_jump_point<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = f64>,
    {
        if let Some(result) = self.init(start, goal, grid, path) {
            return result;
        }

        let goal_index = self.node_grid.coord_to_index(goal).expect(
            "SearchContext too small for grid",
        );

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {

            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                path::make_path(&self.node_grid, goal_index, path);
                return Ok(SearchMetadata { num_nodes_visited });
            }

            let (current_coord, current_cost) = {
                let node = &mut self.node_grid[current_entry.node_index];
                if node.visited == self.seq {
                    continue;
                }
                node.visited = self.seq;
                (node.coord, node.cost)
            };

            // TODO
        }

        Err(Error::NoPath)
    }
}
