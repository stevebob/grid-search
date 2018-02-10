use num::traits::Zero;
use grid_2d::*;
use direction::*;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DijkstraMapCell<Cost> {
    pub(crate) seen: u64,
    pub(crate) visited: u64,
    pub(crate) cost: Cost,
    pub(crate) direction: Direction,
    pub(crate) coord: Coord,
}

impl<Cost: Zero> DijkstraMapCell<Cost> {
    fn new(coord: Coord) -> Self {
        Self {
            seen: 0,
            visited: 0,
            cost: Zero::zero(),
            direction: Direction::North,
            coord,
        }
    }
}

impl<Cost> DijkstraMapCell<Cost>
where
    Cost: Copy,
{
    pub fn cost(&self) -> Cost {
        self.cost
    }
    pub fn direction(&self) -> Direction {
        self.direction
    }
}

#[derive(Debug, Clone)]
pub enum DijkstraMapEntry<'a, Cost: 'a> {
    Origin,
    Unvisited,
    Outside,
    Cell(&'a DijkstraMapCell<Cost>),
}

impl<'a, Cost> DijkstraMapEntry<'a, Cost> {
    pub fn cell(self) -> Option<&'a DijkstraMapCell<Cost>> {
        match self {
            DijkstraMapEntry::Cell(c) => Some(c),
            _ => None,
        }
    }
    pub fn is_origin(self) -> bool {
        match self {
            DijkstraMapEntry::Origin => true,
            _ => false,
        }
    }
    pub fn is_unvisited(self) -> bool {
        match self {
            DijkstraMapEntry::Unvisited => true,
            _ => false,
        }
    }
    pub fn is_outside(self) -> bool {
        match self {
            DijkstraMapEntry::Outside => true,
            _ => false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DijkstraMap<Cost> {
    pub(crate) seq: u64,
    pub(crate) grid: Grid<DijkstraMapCell<Cost>>,
    pub(crate) origin: Coord,
}

impl<Cost> DijkstraMap<Cost>
where
    Cost: Zero + Copy,
{
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            seq: 1,
            grid: Grid::new_from_fn(width, height, DijkstraMapCell::new),
            origin: Coord::new(0, 0),
        }
    }

    pub fn get(&self, coord: Coord) -> DijkstraMapEntry<Cost> {
        if let Some(cell) = self.grid.get(coord) {
            if cell.seen == self.seq {
                if coord == self.origin {
                    DijkstraMapEntry::Origin
                } else {
                    DijkstraMapEntry::Cell(cell)
                }
            } else {
                DijkstraMapEntry::Unvisited
            }
        } else {
            DijkstraMapEntry::Outside
        }
    }
}
