use std::collections::VecDeque;
use direction::Direction;
use grid::SolidGrid;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    StartOutsideGrid,
    StartSolid,
    NoPath,
}

#[derive(Debug, Clone, Copy)]
struct BfsNode {
    seen: u64,
    coord: (i32, i32),
    from_parent: Option<Direction>,
}

#[derive(Debug, Clone)]
struct BfsNodeGrid {
    width: u32,
    height: u32,
    nodes: Vec<BfsNode>,
}

impl BfsNodeGrid {
    fn new(width: u32, height: u32) -> Self {
        let size = (width * height) as usize;
        let mut nodes = Vec::with_capacity(size);

        for y in 0..height {
            for x in 0..width {
                let node = BfsNode {
                    seen: 0,
                    coord: (x as i32, y as i32),
                    from_parent: None,
                };
                nodes.push(node);
            }
        }

        Self {
            width,
            height,
            nodes,
        }
    }

    fn coord_to_index(&self, (x, y): (i32, i32)) -> Option<usize> {
        if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
            return None;
        }

        Some((self.width * y as u32 + x as u32) as usize)
    }
}

#[derive(Debug, Clone)]
pub struct BfsContext {
    seq: u64,
    queue: VecDeque<usize>,
    node_grid: BfsNodeGrid,
}

impl BfsContext {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            seq: 0,
            node_grid: BfsNodeGrid::new(width, height),
            queue: VecDeque::new(),
        }
    }

    fn make_path(path: &mut Vec<Direction>, node_grid: &BfsNodeGrid, goal_index: usize) {
        path.clear();
        let mut index = goal_index;
        while let Some(from_parent) = node_grid.nodes[index].from_parent {
            path.push(from_parent);
            let to_parent = from_parent.opposite();
            let (dx, dy) = to_parent.into();
            let (current_x, current_y) = node_grid.nodes[index].coord;
            index = node_grid.coord_to_index((current_x + dx, current_y + dy))
                .expect("Invalid search state");
        }
        path.reverse();
    }

    pub fn search<G, V, D>(&mut self,
                           grid: &G,
                           start: (i32, i32),
                           goal: (i32, i32),
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

            let node = &mut self.node_grid.nodes[index];
            node.from_parent = None;
            node.seen = self.seq;
            self.queue.push_back(index);
        } else {
            return Err(Error::StartOutsideGrid);
        };

        while let Some(current_index) = self.queue.pop_front() {
            let (current_x, current_y) = self.node_grid.nodes[current_index].coord;
            for v in directions {
                let direction = v.into();
                let (dx, dy) = direction.into();
                let neighbour_coord = (current_x + dx, current_y + dy);

                if grid.is_solid(neighbour_coord) {
                    continue;
                }

                if let Some(index) = self.node_grid.coord_to_index(neighbour_coord) {
                    {
                        let node = &mut self.node_grid.nodes[index];
                        if node.seen != self.seq {
                            node.seen = self.seq;
                            node.from_parent = Some(direction);
                            self.queue.push_back(index);
                        }
                    }

                    if neighbour_coord == goal {
                        Self::make_path(path, &self.node_grid, index);
                        return Ok(());
                    }
                }
            }
        }

        Err(Error::NoPath)
    }
}

#[cfg(test)]
mod tests {

    use direction::*;
    use grid::SolidGrid;
    use path::PathWalk;
    use super::*;

    struct BoolGrid {
        width: u32,
        height: u32,
        cells: Vec<bool>,
    }

    impl BoolGrid {
        fn new(width: u32, height: u32) -> Self {
            let size = (width * height) as usize;
            let mut cells = Vec::with_capacity(size);
            cells.resize(size, false);
            Self {
                width,
                height,
                cells,
            }
        }

        fn get_index(&self, (x, y): (i32, i32)) -> Option<usize> {
            if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
                return None;
            }
            Some((self.width * y as u32 + x as u32) as usize)
        }

        fn get_mut(&mut self, coord: (i32, i32)) -> Option<&mut bool> {
            if let Some(index) = self.get_index(coord) {
                Some(&mut self.cells[index])
            } else {
                None
            }
        }

        fn get(&self, coord: (i32, i32)) -> Option<&bool> {
            if let Some(index) = self.get_index(coord) {
                Some(&self.cells[index])
            } else {
                None
            }
        }

        fn from_strings(strings: &Vec<&str>) -> (Self, (i32, i32), (i32, i32)) {
            let width = strings[0].len() as u32;
            let height = strings.len() as u32;
            let mut grid = Self::new(width, height);
            let mut start = None;
            let mut goal = None;
            for (i, line) in strings.into_iter().enumerate() {
                for (j, ch) in line.chars().enumerate() {
                    let coord = (j as i32, i as i32);
                    match ch {
                        '.' => (),
                        '#' => *grid.get_mut(coord).unwrap() = true,
                        's' => start = Some(coord),
                        'g' => goal = Some(coord),
                        'B' => {
                            goal = Some(coord);
                            start = Some(coord);
                        }
                        'G' => {
                            goal = Some(coord);
                            *grid.get_mut(coord).unwrap() = true;
                        }
                        'S' => {
                            start = Some(coord);
                            *grid.get_mut(coord).unwrap() = true;
                        }
                        _ => panic!(),
                    }
                }
            }

            (grid, start.unwrap(), goal.unwrap())
        }
    }

    impl SolidGrid for BoolGrid {
        fn is_solid(&self, coord: (i32, i32)) -> bool {
            self.get(coord).cloned().unwrap_or(true)
        }
    }

    fn common_test<V, D>(strings: &Vec<&str>, directions: D, length: usize)
        where V: Into<Direction>,
              D: Copy + IntoIterator<Item=V>,
    {
        let (grid, start, goal) = BoolGrid::from_strings(strings);
        let mut ctx = BfsContext::new(grid.width, grid.height);
        let mut path = Vec::new();
        ctx.search(&grid, start, goal, directions, &mut path).unwrap();

        assert_eq!(path.len(), length);

        let walk = PathWalk::new(start, &path);

        let should_be_goal = walk.inspect(|&coord| {
            assert!(!grid.is_solid(coord));
        }).last().unwrap_or(start);

        assert_eq!(should_be_goal, goal);
    }

    #[test]
    fn wall() {
        let strings = vec![
            "..........",
            "....#.....",
            "....#.....",
            "....#.....",
            ".s..#.....",
            "....#...g.",
            "....#.....",
            "..........",
            "..........",
            "..........",
        ];
        common_test(&strings, CardinalDirections, 12);
        common_test(&strings, Directions, 7);
    }

    #[test]
    fn no_path() {
        let strings = vec![
            "....#.....",
            "....#.....",
            "....#.....",
            "....#.....",
            ".s..#.....",
            "....#...g.",
            "....######",
            "..........",
            "..........",
            "..........",
        ];

        let (grid, start, goal) = BoolGrid::from_strings(&strings);
        let mut ctx = BfsContext::new(grid.width, grid.height);
        let mut path = Vec::new();
        let result = ctx.search(&grid, start, goal, Directions, &mut path);

        assert_eq!(result, Err(Error::NoPath));
    }

    #[test]
    fn start_is_goal() {
        let strings = vec![
            "..........",
            "....#.....",
            ".B..#.....",
            "..........",
        ];
        common_test(&strings, CardinalDirections, 0);
        common_test(&strings, Directions, 0);
    }

    #[test]
    fn goal_is_solid() {
        let strings = vec![
            "....#.....",
            "....#.....",
            "....#.....",
            "....#.....",
            ".s..#.....",
            "....#.....",
            "....###G##",
            "..........",
            "..........",
            "..........",
        ];

        let (grid, start, goal) = BoolGrid::from_strings(&strings);
        let mut ctx = BfsContext::new(grid.width, grid.height);
        let mut path = Vec::new();
        let result = ctx.search(&grid, start, goal, Directions, &mut path);

        assert_eq!(result, Err(Error::NoPath));
    }


    #[test]
    fn start_is_solid() {
        let strings = vec![
            "....#.....",
            "....#.....",
            "....#.....",
            "....#.....",
            ".S..#.....",
            "....#.....",
            "....######",
            "..........",
            ".......g..",
            "..........",
        ];

        let (grid, start, goal) = BoolGrid::from_strings(&strings);
        let mut ctx = BfsContext::new(grid.width, grid.height);
        let mut path = Vec::new();
        let result = ctx.search(&grid, start, goal, Directions, &mut path);

        assert_eq!(result, Err(Error::StartSolid));
    }

    #[test]
    fn start_outside_grid() {
        let strings = vec![
            "....#.....",
            "....#.....",
            "....#.....",
            "....#.....",
            ".s..#.....",
            "....#.....",
            "....######",
            "..........",
            ".......g..",
            "..........",
        ];

        let (grid, _, goal) = BoolGrid::from_strings(&strings);

        let start = (-1, -1);

        let mut ctx = BfsContext::new(grid.width, grid.height);
        let mut path = Vec::new();
        let result = ctx.search(&grid, start, goal, Directions, &mut path);

        assert_eq!(result, Err(Error::StartOutsideGrid));
    }
}
