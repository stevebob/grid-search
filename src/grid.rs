use grid_2d::Coord;
use direction::Direction;

pub trait SolidGrid {
    fn is_solid(&self, coord: Coord) -> Option<bool>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CostCell<Cost> {
    Solid,
    Cost(Cost),
}

impl<Cost> CostCell<Cost> {
    pub fn is_solid(&self) -> bool {
        if let &CostCell::Solid = self {
            true
        } else {
            false
        }
    }
}

pub trait CostGrid: SolidGrid {
    type Cost;
    fn cost(&self, coord: Coord, direction: Direction) -> Option<CostCell<Self::Cost>>;
}
