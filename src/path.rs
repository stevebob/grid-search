use std::slice;
use direction::Direction;

pub struct PathWalk<'a> {
    current_coord: (i32, i32),
    directions: slice::Iter<'a, Direction>,
}

impl<'a> PathWalk<'a> {
    pub fn new(start: (i32, i32), path: &'a Vec<Direction>) -> Self {
        Self {
            current_coord: start,
            directions: path.iter(),
        }
    }
}

impl<'a> Iterator for PathWalk<'a> {
    type Item = (i32, i32);
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(&direction) = self.directions.next() {
            let (dx, dy) = direction.into();
            let (current_x, current_y) = self.current_coord;
            let next_coord = (current_x + dx, current_y + dy);
            self.current_coord = next_coord;
            Some(self.current_coord)
        } else {
            None
        }
    }
}
