use num::traits::Zero;
use grid_2d::*;
use direction::*;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DistanceMapCell<Cost> {
    pub(crate) seen: u64,
    pub(crate) visited: u64,
    pub(crate) cost: Cost,
    pub(crate) direction: Direction,
    pub(crate) coord: Coord,
}

impl<Cost: Zero> DistanceMapCell<Cost> {
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

impl<Cost> DistanceMapCell<Cost>
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
pub enum DistanceMapEntry<'a, Cost: 'a> {
    Origin,
    Unvisited,
    Outside,
    Cell(&'a DistanceMapCell<Cost>),
}

impl<'a, Cost> DistanceMapEntry<'a, Cost> {
    pub fn cell(self) -> Option<&'a DistanceMapCell<Cost>> {
        match self {
            DistanceMapEntry::Cell(c) => Some(c),
            _ => None,
        }
    }
    pub fn is_origin(self) -> bool {
        match self {
            DistanceMapEntry::Origin => true,
            _ => false,
        }
    }
    pub fn is_unvisited(self) -> bool {
        match self {
            DistanceMapEntry::Unvisited => true,
            _ => false,
        }
    }
    pub fn is_outside(self) -> bool {
        match self {
            DistanceMapEntry::Outside => true,
            _ => false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceMap<Cost> {
    pub(crate) seq: u64,
    pub(crate) grid: Grid<DistanceMapCell<Cost>>,
    pub(crate) origin: Coord,
}

impl<Cost> DistanceMap<Cost>
where
    Cost: Zero + Copy,
{
    pub fn new(size: Size) -> Self {
        Self {
            seq: 1,
            grid: Grid::new_from_fn(size, DistanceMapCell::new),
            origin: Coord::new(0, 0),
        }
    }

    pub fn width(&self) -> u32 {
        self.grid.width()
    }

    pub fn height(&self) -> u32 {
        self.grid.height()
    }

    pub fn size(&self) -> Size {
        self.grid.size()
    }

    pub fn get(&self, coord: Coord) -> DistanceMapEntry<Cost> {
        if let Some(cell) = self.grid.get(coord) {
            if cell.seen == self.seq {
                if coord == self.origin {
                    DistanceMapEntry::Origin
                } else {
                    DistanceMapEntry::Cell(cell)
                }
            } else {
                DistanceMapEntry::Unvisited
            }
        } else {
            DistanceMapEntry::Outside
        }
    }

    pub fn cost(&self, coord: Coord) -> Option<Cost> {
        match self.get(coord) {
            DistanceMapEntry::Cell(cell) => Some(cell.cost),
            DistanceMapEntry::Origin => Some(Zero::zero()),
            DistanceMapEntry::Unvisited => None,
            DistanceMapEntry::Outside => None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UniformDistanceMap<Cost, Directions> {
    pub(crate) distance_map: DistanceMap<Cost>,
    pub(crate) directions: Directions,
}

impl<Cost, IntoDirection, Directions> UniformDistanceMap<Cost, Directions>
where
    Cost: Zero + Copy,
    IntoDirection: Into<Direction>,
    Directions: Copy + IntoIterator<Item = IntoDirection>,
{
    pub fn new(size: Size, directions: Directions) -> Self {
        let distance_map = DistanceMap::new(size);
        Self {
            distance_map,
            directions,
        }
    }

    pub fn width(&self) -> u32 {
        self.distance_map.width()
    }

    pub fn height(&self) -> u32 {
        self.distance_map.height()
    }

    pub fn size(&self) -> Size {
        self.distance_map.size()
    }

    pub fn get(&self, coord: Coord) -> DistanceMapEntry<Cost> {
        self.distance_map.get(coord)
    }

    pub fn cost(&self, coord: Coord) -> Option<Cost> {
        self.distance_map.cost(coord)
    }
}
