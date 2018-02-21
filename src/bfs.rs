use std::collections::VecDeque;
use std::ops::Add;
use best::BestMap;
use num::traits::{One, Zero};
use direction::Direction;
use grid::SolidGrid;
use grid_2d::*;
use error::*;
use metadata::*;
use config::*;
use path::{self, PathNode};
use distance_map::*;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct BfsNode {
    seen: u64,
    coord: Coord,
    from_parent: Option<Direction>,
}

impl PathNode for BfsNode {
    fn from_parent(&self) -> Option<Direction> {
        self.from_parent
    }
    fn coord(&self) -> Coord {
        self.coord
    }
}

impl From<Coord> for BfsNode {
    fn from(coord: Coord) -> Self {
        Self {
            seen: 0,
            coord,
            from_parent: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Entry {
    index: usize,
    depth: usize,
}

impl Entry {
    fn new(index: usize, depth: usize) -> Self {
        Self { index, depth }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BfsContext {
    seq: u64,
    queue: VecDeque<Entry>,
    node_grid: Grid<BfsNode>,
}

impl BfsContext {
    pub fn new(size: Size) -> Self {
        Self {
            seq: 0,
            node_grid: Grid::new_from_coord(size),
            queue: VecDeque::new(),
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

    pub fn bfs_best<G, V, D, S, F>(
        &mut self,
        grid: &G,
        start: Coord,
        score: F,
        directions: D,
        config: SearchConfig,
        max_depth: usize,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<usize>, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        S: PartialOrd,
        F: Fn(Coord) -> Option<S>,
    {
        let mut best_map = BestMap::new();

        if let Some(solid) = grid.is_solid(start) {
            if solid && !config.allow_solid_start {
                return Err(Error::StartSolid);
            }

            let index = self.node_grid
                .coord_to_index(start)
                .ok_or(Error::VisitOutsideContext)?;

            if let Some(initial_score) = score(start) {
                best_map.insert_gt(initial_score, index);
            }

            self.seq += 1;
            self.queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            self.queue.push_back(Entry::new(index, 0));
        } else {
            return Err(Error::StartOutsideGrid);
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.queue.pop_front() {
            num_nodes_visited += 1;

            if current_entry.depth >= max_depth {
                continue;
            }

            let current_coord = self.node_grid[current_entry.index].coord;

            let next_depth = current_entry.depth + 1;

            for v in directions {
                let direction = v.into();
                let offset: Coord = direction.into();
                let neighbour_coord = current_coord + offset;

                if let Some(false) = grid.is_solid(neighbour_coord) {
                } else {
                    continue;
                }

                let index = self.node_grid
                    .coord_to_index(neighbour_coord)
                    .ok_or(Error::VisitOutsideContext)?;

                {
                    let node = &mut self.node_grid[index];
                    if node.seen != self.seq {
                        node.seen = self.seq;
                        node.from_parent = Some(direction);
                        self.queue.push_back(Entry::new(index, next_depth));
                    }
                }

                if let Some(score) = score(neighbour_coord) {
                    best_map.insert_gt(score, index);
                }
            }
        }

        if let Some(index) = best_map.into_value() {
            path::make_path_all_adjacent(&self.node_grid, index, path);
            let length = path.len();
            Ok(SearchMetadata {
                num_nodes_visited,
                length,
                cost: length,
            })
        } else {
            Err(Error::NoPath)
        }
    }

    pub fn bfs_predicate<G, V, D, F>(
        &mut self,
        grid: &G,
        start: Coord,
        predicate: F,
        directions: D,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<usize>, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        F: Fn(Coord) -> bool,
    {
        if let Some(solid) = grid.is_solid(start) {
            if solid && !config.allow_solid_start {
                return Err(Error::StartSolid);
            }

            let index = self.node_grid
                .coord_to_index(start)
                .ok_or(Error::VisitOutsideContext)?;

            if predicate(start) {
                path.clear();
                return Ok(SearchMetadata {
                    num_nodes_visited: 0,
                    cost: Zero::zero(),
                    length: 0,
                });
            }

            self.seq += 1;
            self.queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            self.queue.push_back(Entry::new(index, 0));
        } else {
            return Err(Error::StartOutsideGrid);
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.queue.pop_front() {
            num_nodes_visited += 1;

            let current_coord = self.node_grid[current_entry.index].coord;

            let next_depth = current_entry.depth + 1;

            for v in directions {
                let direction = v.into();
                let offset: Coord = direction.into();
                let neighbour_coord = current_coord + offset;

                if let Some(false) = grid.is_solid(neighbour_coord) {
                } else {
                    continue;
                }

                let index = self.node_grid
                    .coord_to_index(neighbour_coord)
                    .ok_or(Error::VisitOutsideContext)?;

                {
                    let node = &mut self.node_grid[index];
                    if node.seen != self.seq {
                        node.seen = self.seq;
                        node.from_parent = Some(direction);
                        self.queue.push_back(Entry::new(index, next_depth));
                    }
                }

                if predicate(neighbour_coord) {
                    path::make_path_all_adjacent(&self.node_grid, index, path);
                    let length = path.len();
                    return Ok(SearchMetadata {
                        num_nodes_visited,
                        length,
                        cost: length,
                    });
                }
            }
        }

        Err(Error::NoPath)
    }

    pub fn bfs<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<usize>, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        self.bfs_predicate(grid, start, |c| c == goal, directions, config, path)
    }

    pub fn populate_distance_map<G, V, D, C>(
        &mut self,
        grid: &G,
        start: Coord,
        directions: D,
        config: SearchConfig,
        distance_map: &mut DistanceMap<C>,
    ) -> Result<DistanceMapMetadata, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        C: Copy + Zero + One + Add<C>,
    {
        if let Some(solid) = grid.is_solid(start) {
            if solid && !config.allow_solid_start {
                return Err(Error::StartSolid);
            }

            let index = distance_map
                .grid
                .coord_to_index(start)
                .ok_or(Error::VisitOutsideDistanceMap)?;

            self.queue.clear();
            self.queue.push_back(Entry::new(index, 0));

            distance_map.seq += 1;
            distance_map.origin = start;
            let cell = &mut distance_map.grid[index];
            cell.seen = distance_map.seq;
            cell.cost = Zero::zero();
        } else {
            return Err(Error::StartOutsideGrid);
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.queue.pop_front() {
            num_nodes_visited += 1;

            let next_depth = current_entry.depth + 1;

            let (current_coord, current_cost) = {
                let cell = &distance_map.grid[current_entry.index];
                (cell.coord, cell.cost)
            };

            for v in directions {
                let direction = v.into();
                let offset: Coord = direction.into();
                let neighbour_coord = current_coord + offset;

                if let Some(false) = grid.is_solid(neighbour_coord) {
                } else {
                    continue;
                }

                let index = distance_map
                    .grid
                    .coord_to_index(neighbour_coord)
                    .ok_or(Error::VisitOutsideDistanceMap)?;

                let cell = &mut distance_map.grid[index];
                if cell.seen != distance_map.seq {
                    cell.seen = distance_map.seq;
                    cell.direction = direction.opposite();
                    cell.cost = current_cost + One::one();
                    self.queue.push_back(Entry::new(index, next_depth));
                }
            }
        }

        Ok(DistanceMapMetadata { num_nodes_visited })
    }

    pub fn populate_uniform_distance_map<G, V, D, C>(
        &mut self,
        grid: &G,
        start: Coord,
        config: SearchConfig,
        distance_map: &mut UniformDistanceMap<C, D>,
    ) -> Result<DistanceMapMetadata, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        C: Copy + Zero + One + Add<C>,
    {
        self.populate_distance_map(
            grid,
            start,
            distance_map.directions,
            config,
            &mut distance_map.distance_map,
        )
    }
}
