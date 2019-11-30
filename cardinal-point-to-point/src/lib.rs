use direction::CardinalDirection;
use grid_2d::{Coord, Grid, Size};
use std::cmp::Ordering;
use std::collections::{vec_deque, BinaryHeap, VecDeque};

const DIRECTIONS: [Direction; 4] = [
    Direction(Coord::new(0, 1)),
    Direction(Coord::new(1, 0)),
    Direction(Coord::new(0, -1)),
    Direction(Coord::new(-1, 0)),
];

#[derive(Clone, Copy, Debug)]
struct Direction(Coord);

#[derive(Clone, Debug)]
struct Step {
    to_coord: Coord,
    in_direction: Direction,
}

impl Step {
    fn forward(&self) -> Self {
        let in_direction = self.in_direction;
        Self {
            to_coord: self.to_coord + in_direction.0,
            in_direction,
        }
    }
    fn left(&self) -> Self {
        let in_direction = Direction(self.in_direction.0.left90());
        Self {
            to_coord: self.to_coord + in_direction.0,
            in_direction,
        }
    }
    fn right(&self) -> Self {
        let in_direction = Direction(self.in_direction.0.right90());
        Self {
            to_coord: self.to_coord + in_direction.0,
            in_direction,
        }
    }
}

#[derive(Debug)]
struct Node {
    cost: u32,
    cost_plus_heuristic: u32,
    step: Step,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.cost_plus_heuristic.eq(&other.cost_plus_heuristic)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match other
            .cost_plus_heuristic
            .partial_cmp(&self.cost_plus_heuristic)
        {
            Some(Ordering::Equal) => self.cost.partial_cmp(&other.cost),
            other => other,
        }
    }
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.cost_plus_heuristic.cmp(&self.cost_plus_heuristic) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            other => other,
        }
    }
}

struct SeenCell {
    count: u64,
    in_direction: Option<Direction>,
}

pub trait PointToPointSearch {
    fn can_enter(&self, coord: Coord) -> bool;
}

struct Stop;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PathNode {
    pub to_coord: Coord,
    pub in_direction: CardinalDirection,
}

pub struct PathIter<'a> {
    iter: vec_deque::Iter<'a, Step>,
}

impl<'a> Iterator for PathIter<'a> {
    type Item = PathNode;
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|step| PathNode {
            to_coord: step.to_coord,
            in_direction: CardinalDirection::from_unit_coord(step.in_direction.0),
        })
    }
}

#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[derive(Default, Clone, Debug)]
pub struct Path {
    steps: VecDeque<Step>,
}

impl Path {
    pub fn iter(&self) -> PathIter {
        PathIter {
            iter: self.steps.iter(),
        }
    }
    pub fn len(&self) -> usize {
        self.steps.len()
    }
}

pub struct Context {
    count: u64,
    seen_set: Grid<SeenCell>,
    priority_queue: BinaryHeap<Node>,
}

impl Context {
    pub fn new(size: Size) -> Self {
        Self {
            count: 1,
            seen_set: Grid::new_fn(size, |_| SeenCell {
                count: 0,
                in_direction: None,
            }),
            priority_queue: BinaryHeap::new(),
        }
    }

    fn build_path_to(&self, end: Coord, path: &mut Path) {
        let mut cell = self.seen_set.get(end).expect("path end out of bounds");
        debug_assert_eq!(
            cell.count, self.count,
            "path end not visited in latest search"
        );
        let mut coord = end;
        path.steps.clear();
        while let Some(in_direction) = cell.in_direction {
            let step = Step {
                to_coord: coord,
                in_direction,
            };
            path.steps.push_back(step);
            coord = coord - in_direction.0;
            cell = self.seen_set.get_checked(coord);
            debug_assert_eq!(
                cell.count, self.count,
                "path includes cell not visited in latest search"
            );
        }
    }

    fn first_step_towards(&self, end: Coord) -> Option<Step> {
        let mut cell = self.seen_set.get(end).expect("path end out of bounds");
        debug_assert_eq!(
            cell.count, self.count,
            "path end not visited in latest search"
        );
        let mut coord = end;
        let mut ret = None;
        while let Some(in_direction) = cell.in_direction {
            let step = Step {
                to_coord: coord,
                in_direction,
            };
            coord = coord - in_direction.0;
            cell = self.seen_set.get_checked(coord);
            debug_assert_eq!(
                cell.count, self.count,
                "path includes cell not visited in latest search"
            );
            ret = Some(step);
        }
        ret
    }

    fn consider<P: PointToPointSearch>(
        &mut self,
        point_to_point_search: &P,
        step: Step,
        cost: u32,
        goal: Coord,
    ) -> Option<Stop> {
        if let Some(cell) = self.seen_set.get_mut(step.to_coord) {
            if cell.count != self.count {
                cell.count = self.count;
                if point_to_point_search.can_enter(step.to_coord) {
                    cell.in_direction = Some(step.in_direction);
                    if step.to_coord == goal {
                        return Some(Stop);
                    }
                    let heuristic = step.to_coord.manhattan_distance(goal);
                    let node = Node {
                        cost,
                        cost_plus_heuristic: cost + heuristic,
                        step,
                    };
                    self.priority_queue.push(node);
                }
            }
        }
        None
    }

    fn point_to_point_search_core<P: PointToPointSearch>(
        &mut self,
        point_to_point_search: &P,
        start: Coord,
        goal: Coord,
    ) {
        self.count += 1;
        self.priority_queue.clear();
        let start_cell = self.seen_set.get_checked_mut(start);
        start_cell.count = self.count;
        start_cell.in_direction = None;
        if start == goal {
            return;
        }
        for &in_direction in &DIRECTIONS {
            let to_coord = start + in_direction.0;
            let step = Step {
                to_coord,
                in_direction,
            };
            if let Some(Stop) = self.consider(point_to_point_search, step, 1, goal) {
                return;
            }
        }
        while let Some(Node { cost, step, .. }) = self.priority_queue.pop() {
            let next_cost = cost + 1;
            if let Some(Stop) =
                self.consider(point_to_point_search, step.forward(), next_cost, goal)
            {
                return;
            }
            if let Some(Stop) = self.consider(point_to_point_search, step.left(), next_cost, goal) {
                return;
            }
            if let Some(Stop) = self.consider(point_to_point_search, step.right(), next_cost, goal)
            {
                return;
            }
        }
    }

    pub fn point_to_point_search_path<P: PointToPointSearch>(
        &mut self,
        point_to_point_search: P,
        start: Coord,
        goal: Coord,
        path: &mut Path,
    ) {
        self.point_to_point_search_core(&point_to_point_search, start, goal);
        self.build_path_to(goal, path);
    }

    pub fn point_to_point_search_first<P: PointToPointSearch>(
        &mut self,
        point_to_point_search: P,
        start: Coord,
        goal: Coord,
    ) -> Option<CardinalDirection> {
        self.point_to_point_search_core(&point_to_point_search, start, goal);
        self.first_step_towards(goal)
            .map(|step| CardinalDirection::from_unit_coord(step.in_direction.0))
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[derive(Clone)]
    enum Cell {
        Solid,
        Traversable,
    }

    struct Test {
        grid: Grid<Cell>,
        start: Coord,
        goal: Coord,
    }

    fn str_slice_to_test(str_slice: &[&str]) -> Test {
        let width = str_slice[0].len() as u32;
        let height = str_slice.len() as u32;
        let size = Size::new(width, height);
        let mut grid = Grid::new_clone(size, Cell::Solid);
        let mut start = None;
        let mut goal = None;
        for (y, line) in str_slice.iter().enumerate() {
            for (x, ch) in line.chars().enumerate() {
                let coord = Coord::new(x as i32, y as i32);
                let cell = match ch {
                    '.' => Cell::Traversable,
                    '@' => {
                        start = Some(coord);
                        Cell::Traversable
                    }
                    '*' => {
                        goal = Some(coord);
                        Cell::Traversable
                    }
                    '#' => Cell::Solid,
                    _ => panic!(),
                };
                *grid.get_checked_mut(coord) = cell;
            }
        }
        Test {
            grid,
            start: start.unwrap(),
            goal: goal.unwrap_or(start.unwrap()),
        }
    }

    struct Search<'a> {
        grid: &'a Grid<Cell>,
    }

    impl<'a> PointToPointSearch for Search<'a> {
        fn can_enter(&self, coord: Coord) -> bool {
            if let Some(cell) = self.grid.get(coord) {
                match cell {
                    Cell::Solid => false,
                    Cell::Traversable => true,
                }
            } else {
                false
            }
        }
    }

    const GRID_A: &[&str] = &[
        "..........",
        ".......*..",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        ".@........",
        "..........",
    ];

    #[test]
    fn grid_a() {
        let Test { grid, start, goal } = str_slice_to_test(GRID_A);
        let mut ctx = Context::new(grid.size());
        let mut path = Path::default();
        ctx.point_to_point_search_path(Search { grid: &grid }, start, goal, &mut path);
        assert_eq!(path.len(), 13);
    }

    const GRID_B: &[&str] = &[
        "..........",
        ".......#..",
        ".......#..",
        "....*..#..",
        "########..",
        "..........",
        "..........",
        "..........",
        ".@........",
        "..........",
    ];

    #[test]
    fn grid_b() {
        let Test { grid, start, goal } = str_slice_to_test(GRID_B);
        let mut ctx = Context::new(grid.size());
        let mut path = Path::default();
        ctx.point_to_point_search_path(Search { grid: &grid }, start, goal, &mut path);
        assert_eq!(path.len(), 22);
    }

    const GRID_C: &[&str] = &[
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        ".@*.......",
        "..........",
    ];

    #[test]
    fn grid_c() {
        let Test { grid, start, goal } = str_slice_to_test(GRID_C);
        let mut ctx = Context::new(grid.size());
        let mut path = Path::default();
        ctx.point_to_point_search_path(Search { grid: &grid }, start, goal, &mut path);
        assert_eq!(path.len(), 1);
    }

    const GRID_D: &[&str] = &[
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        "..........",
        ".@........",
        "..........",
    ];

    #[test]
    fn grid_d() {
        let Test { grid, start, goal } = str_slice_to_test(GRID_D);
        let mut ctx = Context::new(grid.size());
        let mut path = Path::default();
        ctx.point_to_point_search_path(Search { grid: &grid }, start, goal, &mut path);
        assert_eq!(path.len(), 0);
    }
}
