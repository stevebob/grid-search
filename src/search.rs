use std::collections::BinaryHeap;
use std::ops::{Add, Sub};
use std::cmp::Ordering;
use num::traits::{One, Zero};
use direction::*;
use grid_2d::*;
use best::BestMapNonEmpty;
use grid::*;
use error::*;
use metadata::*;
use config::*;
use path::{self, PathNode};
use distance_map::*;

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
    pub(crate) fn new(node_index: usize, cost: Cost) -> Self {
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
    pub fn new(size: Size) -> Self {
        Self {
            seq: 0,
            node_grid: Grid::new_from_coord(size),
            priority_queue: BinaryHeap::new(),
        }
    }
    pub fn width(&self) -> u32 {
        self.node_grid.width()
    }
    pub fn height(&self) -> u32 {
        self.node_grid.height()
    }
    pub fn size(&self) -> Size {
        self.node_grid.size()
    }
}

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub(crate) fn init<G, F>(
        &mut self,
        start: Coord,
        predicate: F,
        grid: &G,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<PriorityEntry<Cost>, Result<SearchMetadata<Cost>, Error>>
    where
        G: SolidGrid,
        F: Fn(Coord) -> bool,
    {
        if let Some(solid) = grid.is_solid(start) {
            let index = if let Some(index) = self.node_grid.coord_to_index(start) {
                index
            } else {
                return Err(Err(Error::VisitOutsideContext));
            };

            if solid && !config.allow_solid_start {
                return Err(Err(Error::StartSolid));
            };

            if predicate(start) {
                path.clear();
                return Err(Ok(SearchMetadata {
                    num_nodes_visited: 0,
                    cost: Zero::zero(),
                    length: 0,
                }));
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
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        H: Fn(Coord, Coord) -> Cost,
    {
        let initial_entry = match self.init(start, |c| c == goal, grid, config, path) {
            Ok(initial_entry) => initial_entry,
            Err(result) => return result,
        };

        self.priority_queue.push(initial_entry);

        let goal_index = self.node_grid
            .coord_to_index(goal)
            .ok_or(Error::VisitOutsideContext)?;

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {
            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                let node = &self.node_grid[goal_index];

                path::make_path_all_adjacent(&self.node_grid, goal_index, path);
                return Ok(SearchMetadata {
                    num_nodes_visited,
                    cost: node.cost,
                    length: path.len(),
                });
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
                )?;
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
    ) -> Result<(), Error>
    where
        H: Fn(Coord, Coord) -> Cost,
    {
        let index = self.node_grid
            .coord_to_index(successor_coord)
            .ok_or(Error::VisitOutsideContext)?;

        let node = &mut self.node_grid[index];

        if node.seen != self.seq || node.cost > cost {
            node.from_parent = Some(direction);
            node.seen = self.seq;
            node.cost = cost;

            let heuristic = cost + heuristic_fn(successor_coord, goal);
            let entry = PriorityEntry::new(index, heuristic);
            self.priority_queue.push(entry);
        }

        Ok(())
    }
}

impl<Cost> SearchContext<Cost>
where
    Cost: Copy + Add + PartialOrd + Zero + One,
{
    pub fn populate_distance_map<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        directions: D,
        config: SearchConfig,
        distance_map: &mut DistanceMap<Cost>,
    ) -> Result<DistanceMapMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        if let Some(solid) = grid.is_solid(start) {
            if solid && !config.allow_solid_start {
                return Err(Error::StartSolid);
            };

            let index = distance_map
                .grid
                .coord_to_index(start)
                .ok_or(Error::VisitOutsideDistanceMap)?;

            self.priority_queue.clear();
            self.priority_queue
                .push(PriorityEntry::new(index, Zero::zero()));

            distance_map.seq += 1;
            distance_map.origin = start;
            let cell = &mut distance_map.grid[index];
            cell.seen = distance_map.seq;
            cell.cost = Zero::zero();
        } else {
            return Err(Error::StartOutsideGrid);
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {
            num_nodes_visited += 1;

            let (current_coord, current_cost) = {
                let cell = &mut distance_map.grid[current_entry.node_index];
                if cell.visited == distance_map.seq {
                    continue;
                }
                cell.visited = distance_map.seq;
                (cell.coord, cell.cost)
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

                let cost = current_cost + neighbour_cost;

                let index = distance_map
                    .grid
                    .coord_to_index(neighbour_coord)
                    .ok_or(Error::VisitOutsideDistanceMap)?;

                let cell = &mut distance_map.grid[index];

                if cell.seen != distance_map.seq || cell.cost > cost {
                    cell.direction = direction.opposite();
                    cell.seen = distance_map.seq;
                    cell.cost = cost;

                    let entry = PriorityEntry::new(index, cost);
                    self.priority_queue.push(entry);
                }
            }
        }

        Ok(DistanceMapMetadata { num_nodes_visited })
    }
}

impl<Cost> SearchContext<Cost>
where
    Cost: Copy + Add + PartialOrd + Zero + One + Sub<Output = Cost>,
{
    pub fn best_search_uniform_distance_map<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        config: SearchConfig,
        max_depth: Cost,
        distance_map: &UniformDistanceMap<Cost, D>,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        let mut initial_entry =
            match self.init(start, |_| max_depth == Zero::zero(), grid, config, path) {
                Ok(initial_entry) => initial_entry,
                Err(result) => return result,
            };

        initial_entry.cost = distance_map
            .cost(start)
            .ok_or(Error::InconsistentDistanceMap)?;

        let mut best_map = BestMapNonEmpty::new(initial_entry.cost, initial_entry.node_index);
        self.priority_queue.push(initial_entry);

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {
            num_nodes_visited += 1;

            let (current_coord, current_depth) = {
                let node = &self.node_grid[current_entry.node_index];
                (node.coord, node.cost)
            };

            if current_depth >= max_depth {
                continue;
            }

            let remaining_depth = max_depth - current_depth;
            if *best_map.key() + remaining_depth <= current_entry.cost {
                continue;
            }

            let next_depth = current_depth + One::one();

            for v in distance_map.directions {
                let direction = v.into();
                let offset: Coord = direction.coord();
                let neighbour_coord = current_coord + offset;

                if let Some(false) = grid.is_solid(neighbour_coord) {
                } else {
                    continue;
                }

                let cost = distance_map
                    .cost(neighbour_coord)
                    .ok_or(Error::InconsistentDistanceMap)?;

                let index = self.node_grid
                    .coord_to_index(neighbour_coord)
                    .ok_or(Error::VisitOutsideContext)?;

                {
                    let node = &mut self.node_grid[index];
                    if node.seen != self.seq {
                        node.seen = self.seq;
                        node.from_parent = Some(direction);
                        node.cost = next_depth;
                        self.priority_queue.push(PriorityEntry::new(index, cost));
                    }
                }

                best_map.insert_lt(cost, index);
            }
        }

        let (cost, index) = best_map.into_key_and_value();
        path::make_path_all_adjacent(&self.node_grid, index, path);
        let length = path.len();
        Ok(SearchMetadata {
            num_nodes_visited,
            length,
            cost,
        })
    }
}
