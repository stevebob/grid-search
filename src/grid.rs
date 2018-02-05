use std::ops::Add;
use grid_2d::Coord;
use direction::Direction;

pub trait SolidGrid {
    fn is_solid(&self, coord: Coord) -> bool;
}

pub trait CostGrid: SolidGrid {
    type Cost: Add<Self::Cost> + PartialOrd<Self::Cost>;
    fn cost(&self, coord: Coord, direction: Direction) -> Option<Self::Cost>;
}
