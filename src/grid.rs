use grid_2d::Coord;
use direction::Direction;

pub trait SolidGrid {
    fn is_solid(&self, coord: Coord) -> bool;
}

pub trait CostGrid: SolidGrid {
    fn cost(&self, coord: Coord, direction: Direction) -> Option<u32>;
}
