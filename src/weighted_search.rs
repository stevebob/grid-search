use std::collections::BinaryHeap;
use std::cmp::Ordering;
use direction::Direction;
use grid_2d::*;
use grid::*;
use error::*;
use metadata::*;
use path::{self, PathNode};

#[derive(Debug, Clone, Copy)]
pub struct WeightedSearchNode {
    seen: u64,
    visited: u64,
    coord: Coord,
    from_parent: Option<Direction>,
    cost: u32,
}

impl PathNode for WeightedSearchNode {
    fn from_parent(&self) -> Option<Direction> {
        self.from_parent
    }
    fn coord(&self) -> Coord {
        self.coord
    }
}

impl From<Coord> for WeightedSearchNode {
    fn from(coord: Coord) -> Self {
        Self {
            seen: 0,
            visited: 0,
            coord,
            from_parent: None,
            cost: 0,
        }
    }
}

#[derive(Debug, Clone)]
struct PriorityEntry {
    node_index: usize,
    cost: u32,
}

impl PriorityEntry {
    fn new(node_index: usize, cost: u32) -> Self {
        PriorityEntry { node_index, cost }
    }
}

impl PartialEq for PriorityEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl PartialOrd for PriorityEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.cost.partial_cmp(&self.cost)
    }
}

impl Eq for PriorityEntry {}

impl Ord for PriorityEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

#[derive(Debug, Clone)]
pub struct WeightedSearchContext {
    seq: u64,
    priority_queue: BinaryHeap<PriorityEntry>,
    node_grid: Grid<WeightedSearchNode>,
}

impl WeightedSearchContext {
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
        G: CostGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {

        if let Some(index) = self.node_grid.coord_to_index(start) {

            if grid.is_solid(start) {
                return Err(Error::StartSolid);
            };

            if start == goal {
                path.clear();
                return Ok(Default::default());
            }

            self.seq += 1;
            self.priority_queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            node.cost = 0;
            let entry = PriorityEntry::new(index, 0);
            self.priority_queue.push(entry);
        } else {
            return Err(Error::StartOutsideGrid);
        };

        let goal_index = if let Some(goal_index) = self.node_grid.coord_to_index(goal) {
            goal_index
        } else {
            return Err(Error::GoalOutsideGrid);
        };

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {

            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                path::make_path(&self.node_grid, goal_index, path);
                return Ok(SearchMetadata {
                    num_nodes_visited,
                });
            }

            let current_coord = {
                let node = &mut self.node_grid[current_entry.node_index];
                if node.visited == self.seq {
                    continue;
                }
                node.visited = self.seq;
                node.coord
            };

            for v in directions {
                let direction = v.into();
                let offset: Coord = direction.into();
                let neighbour_coord = current_coord + offset;

                if let Some(index) = self.node_grid.coord_to_index(neighbour_coord) {

                    let neighbour_cost =
                        if let Some(cost) = grid.cost(neighbour_coord, direction) {
                            cost
                        } else {
                            continue;
                        };

                    let node = &mut self.node_grid[index];

                    let cost = current_entry.cost + neighbour_cost;

                    if node.seen != self.seq || node.cost > cost {
                        node.from_parent = Some(direction);
                        node.seen = self.seq;
                        node.cost = cost;
                        let entry = PriorityEntry::new(index, cost);
                        self.priority_queue.push(entry);
                    }

                }
            }
        }

        Err(Error::NoPath)
    }
}
