use std::slice;
use direction::Direction;
use grid_2d::*;

pub(crate) trait PathNode {
    fn from_parent(&self) -> Option<Direction>;
    fn coord(&self) -> Coord;
}

pub(crate) fn make_path<N: PathNode>(
    node_grid: &Grid<N>,
    goal_index: usize,
    path: &mut Vec<Direction>,
) {
    path.clear();
    let mut index = goal_index;
    while let Some(from_parent) = node_grid[index].from_parent() {
        path.push(from_parent);
        let to_parent = from_parent.opposite();
        let offset: Coord = to_parent.into();
        index = node_grid
            .coord_to_index(node_grid[index].coord() + offset)
            .expect("Invalid search state");
    }
    path.reverse();
}


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
    type Item = (Coord, Direction);
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(&direction) = self.directions.next() {
            let offset: Coord = direction.into();
            self.current_coord = self.current_coord + offset;
            Some((self.current_coord, direction))
        } else {
            None
        }
    }
}
