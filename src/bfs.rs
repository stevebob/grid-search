use std::collections::VecDeque;
use direction::Direction;
use grid::SolidGrid;
use grid_2d::*;
use error::*;
use path::{self, PathNode};

#[derive(Debug, Clone, Copy)]
pub struct BfsNode {
    pub seen: u64,
    pub coord: Coord,
    pub from_parent: Option<Direction>,
}

impl PathNode for BfsNode {
    fn from_parent(&self) -> Option<Direction> { self.from_parent }
    fn coord(&self) -> Coord { self.coord }
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

#[derive(Debug, Clone)]
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

    pub fn search<G, V, D>(&mut self,
                           grid: &G,
                           start: Coord,
                           goal: Coord,
                           directions: D,
                           path: &mut Vec<Direction>) -> Result<(), Error>
        where G: SolidGrid,
              V: Into<Direction>,
              D: Copy + IntoIterator<Item=V>,
    {

        if let Some(index) = self.node_grid.coord_to_index(start) {

            if grid.is_solid(start) {
                return Err(Error::StartSolid);
            }

            if start == goal {
                path.clear();
                return Ok(());
            }

            self.seq += 1;
            self.queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            self.queue.push_back(index);
        } else {
            return Err(Error::StartOutsideGrid);
        };

        while let Some(current_index) = self.queue.pop_front() {
            let current_coord = self.node_grid[current_index].coord;
            for v in directions {
                let direction = v.into();
                let offset: Coord = direction.into();
                let neighbour_coord = current_coord + offset;

                if let Some(index) = self.node_grid.coord_to_index(neighbour_coord) {
                    if grid.is_solid(neighbour_coord) {
                        continue;
                    }

                    {
                        let node = &mut self.node_grid[index];
                        if node.seen != self.seq {
                            node.seen = self.seq;
                            node.from_parent = Some(direction);
                            self.queue.push_back(index);
                        }
                    }

                    if neighbour_coord == goal {
                        path::make_path(&self.node_grid, index, path);
                        return Ok(());
                    }
                }
            }
        }

        Err(Error::NoPath)
    }
}
