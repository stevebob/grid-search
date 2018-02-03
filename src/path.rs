use std::slice;
use direction::Direction;
use grid_2d::Coord;

pub struct PathWalk<'a> {
    current_coord: Coord,
    directions: slice::Iter<'a, Direction>,
}

impl<'a> PathWalk<'a> {
    pub fn new(start: Coord, path: &'a Vec<Direction>) -> Self {
        Self {
            current_coord: start,
            directions: path.iter(),
        }
    }
}

impl<'a> Iterator for PathWalk<'a> {
    type Item = Coord;
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(&direction) = self.directions.next() {
            let offset: Coord = direction.into();
            self.current_coord = self.current_coord + offset;
            Some(self.current_coord)
        } else {
            None
        }
    }
}
