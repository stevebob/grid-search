use grid_2d::Coord;

pub trait SolidGrid {
    fn is_solid(&self, coord: Coord) -> bool;
}
