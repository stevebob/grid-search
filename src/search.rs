use std::collections::BinaryHeap;
use std::ops::Add;
use std::cmp::Ordering;
use num_traits::Zero;
use direction::*;
use grid_2d::*;
use grid::*;
use error::*;
use metadata::*;
use path::{self, PathNode};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub(crate) struct SearchNode<Cost> {
    pub(crate) seen: u64,
    pub(crate) visited: u64,
    pub(crate) coord: Coord,
    pub(crate) from_parent: Option<Direction>,
    pub(crate) cost: Cost,
}

impl<Cost: Zero> From<Coord> for SearchNode<Cost> {
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

impl<Cost> PathNode for SearchNode<Cost> {
    fn from_parent(&self) -> Option<Direction> {
        self.from_parent
    }
    fn coord(&self) -> Coord {
        self.coord
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct PriorityEntry<Cost: PartialOrd<Cost>> {
    pub(crate) node_index: usize,
    pub(crate) cost: Cost,
}

impl<Cost: PartialOrd<Cost>> PriorityEntry<Cost> {
    fn new(node_index: usize, cost: Cost) -> Self {
        Self { node_index, cost }
    }
}

impl<Cost: PartialOrd<Cost>> PartialEq for PriorityEntry<Cost> {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl<Cost: PartialOrd<Cost>> PartialOrd for PriorityEntry<Cost> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.cost.partial_cmp(&self.cost)
    }
}

impl<Cost: PartialOrd<Cost>> Eq for PriorityEntry<Cost> {}

impl<Cost: PartialOrd<Cost>> Ord for PriorityEntry<Cost> {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SearchContext<Cost: PartialOrd<Cost>> {
    pub(crate) seq: u64,
    pub(crate) priority_queue: BinaryHeap<PriorityEntry<Cost>>,
    pub(crate) node_grid: Grid<SearchNode<Cost>>,
}

impl<Cost: PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            seq: 0,
            node_grid: Grid::new_from_coord(width, height),
            priority_queue: BinaryHeap::new(),
        }
    }
}

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub(crate) fn init<G>(
        &mut self,
        start: Coord,
        goal: Coord,
        grid: &G,
        path: &mut Vec<Direction>,
    ) -> Result<PriorityEntry<Cost>, Result<SearchMetadata, Error>>
    where
        G: SolidGrid,
    {
        if let Some(solid) = grid.is_solid(start) {
            let index = self.node_grid
                .coord_to_index(start)
                .expect("SearchContext too small for grid");

            if solid {
                return Err(Err(Error::StartSolid));
            };

            if start == goal {
                path.clear();
                return Err(Ok(Default::default()));
            }

            self.seq += 1;
            self.priority_queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            node.cost = Zero::zero();

            Ok(PriorityEntry::new(index, Zero::zero()))
        } else {
            Err(Err(Error::StartOutsideGrid))
        }
    }

    pub(crate) fn search_general<G, V, D, H>(
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
        let initial_entry = match self.init(start, goal, grid, path) {
            Ok(initial_entry) => initial_entry,
            Err(result) => return result,
        };

        self.priority_queue.push(initial_entry);

        let goal_index = self.node_grid
            .coord_to_index(goal)
            .expect("SearchContext too small for grid");

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {
            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                path::make_path_all_adjacent(&self.node_grid, goal_index, path);
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
                let neighbour_coord = current_coord + direction.coord();

                let neighbour_cost =
                    if let Some(CostCell::Cost(cost)) = grid.cost(neighbour_coord, direction) {
                        cost
                    } else {
                        continue;
                    };

                self.see_successor(
                    current_cost + neighbour_cost,
                    neighbour_coord,
                    direction,
                    &heuristic_fn,
                    goal,
                );
            }
        }

        Err(Error::NoPath)
    }

    pub(crate) fn see_successor<H>(
        &mut self,
        cost: Cost,
        successor_coord: Coord,
        direction: Direction,
        heuristic_fn: H,
        goal: Coord,
    ) where
        H: Fn(Coord, Coord) -> Cost,
    {
        let index = self.node_grid
            .coord_to_index(successor_coord)
            .expect("SearchContext too small for grid");

        let node = &mut self.node_grid[index];

        if node.seen != self.seq || node.cost > cost {
            node.from_parent = Some(direction);
            node.seen = self.seq;
            node.cost = cost;

            let heuristic = cost + heuristic_fn(successor_coord, goal);
            let entry = PriorityEntry::new(index, heuristic);
            self.priority_queue.push(entry);
        }
    }
}
