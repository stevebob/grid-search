use std::collections::VecDeque;
use std::ops::Add;
use num_traits::{One, Zero};
use direction::Direction;
use grid::SolidGrid;
use grid_2d::*;
use error::*;
use metadata::*;
use path::{self, PathNode};
use dijkstra_map::*;

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
pub struct BfsContext {
    seq: u64,
    queue: VecDeque<usize>,
    node_grid: Grid<BfsNode>,
}

impl BfsContext {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            seq: 0,
            node_grid: Grid::new_from_coord(width, height),
            queue: VecDeque::new(),
        }
    }

    pub fn width(&self) -> u32 {
        self.node_grid.width()
    }

    pub fn height(&self) -> u32 {
        self.node_grid.height()
    }

    pub fn bfs<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        if let Some(solid) = grid.is_solid(start) {
            if solid {
                return Err(Error::StartSolid);
            }

            let index = self.node_grid
                .coord_to_index(start)
                .expect("BfsContext too small for grid");

            if start == goal {
                path.clear();
                return Ok(Default::default());
            }

            self.seq += 1;
            self.queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            self.queue.push_back(index);
        } else {
            return Err(Error::StartOutsideGrid);
        }

        let mut num_nodes_visited = 0;

        while let Some(current_index) = self.queue.pop_front() {
            num_nodes_visited += 1;

            let current_coord = self.node_grid[current_index].coord;
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
                    .expect("BfsContext too small for grid");

                {
                    let node = &mut self.node_grid[index];
                    if node.seen != self.seq {
                        node.seen = self.seq;
                        node.from_parent = Some(direction);
                        self.queue.push_back(index);
                    }
                }

                if neighbour_coord == goal {
                    path::make_path_all_adjacent(&self.node_grid, index, path);
                    return Ok(SearchMetadata { num_nodes_visited });
                }
            }
        }

        Err(Error::NoPath)
    }

    pub fn populate_dijkstra_map<G, V, D, C>(
        &mut self,
        grid: &G,
        start: Coord,
        directions: D,
        dijkstra_map: &mut DijkstraMap<C>,
    ) -> Result<SearchMetadata, Error>
    where
        G: SolidGrid,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        C: Copy + Zero + One + Add<C>,
    {
        if let Some(solid) = grid.is_solid(start) {
            if solid {
                return Err(Error::StartSolid);
            }

            let index = dijkstra_map
                .grid
                .coord_to_index(start)
                .expect("BfsContext too small for grid");

            self.queue.clear();
            self.queue.push_back(index);

            dijkstra_map.seq += 1;
            dijkstra_map.origin = start;
            let cell = &mut dijkstra_map.grid[index];
            cell.seen = dijkstra_map.seq;
            cell.cost = Zero::zero();
        } else {
            return Err(Error::StartOutsideGrid);
        }

        let mut num_nodes_visited = 0;

        while let Some(current_index) = self.queue.pop_front() {
            num_nodes_visited += 1;

            let (current_coord, current_cost) = {
                let cell = &dijkstra_map.grid[current_index];
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

                let index = dijkstra_map
                    .grid
                    .coord_to_index(neighbour_coord)
                    .expect("BfsContext too small for grid");

                let cell = &mut dijkstra_map.grid[index];
                if cell.seen != dijkstra_map.seq {
                    cell.seen = dijkstra_map.seq;
                    cell.direction = direction.opposite();
                    cell.cost = current_cost + One::one();
                    self.queue.push_back(index);
                }
            }
        }

        Ok(SearchMetadata { num_nodes_visited })
    }
}
