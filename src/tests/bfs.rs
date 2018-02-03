use direction::*;
use grid_2d::*;
use grid::SolidGrid;
use path::PathWalk;
use bfs::*;
use error::*;

fn grid_from_strings(strings: &Vec<&str>) -> (Grid<bool>, Coord, Coord) {
    let width = strings[0].len() as u32;
    let height = strings.len() as u32;
    let mut grid = Grid::new_copy(width, height, false);
    let mut start = None;
    let mut goal = None;
    for (i, line) in strings.into_iter().enumerate() {
        for (j, ch) in line.chars().enumerate() {
            let coord = Coord::new(j as i32, i as i32);
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

impl SolidGrid for Grid<bool> {
    fn is_solid(&self, coord: Coord) -> bool {
        self.get(coord).cloned().unwrap()
    }
}

fn common_test<V, D>(strings: &Vec<&str>, directions: D, length: usize)
    where V: Into<Direction>,
          D: Copy + IntoIterator<Item=V>,
{
    let (grid, start, goal) = grid_from_strings(strings);
    let mut ctx = BfsContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    ctx.search(&grid, start, goal, directions, &mut path).unwrap();

    assert_eq!(path.len(), length);

    let walk = PathWalk::new(start, &path);

    let (should_be_goal, _) = walk.inspect(|&(coord, _)| {
        assert!(!grid.is_solid(coord));
    }).last().unwrap_or((start, Direction::North));

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

    let (grid, start, goal) = grid_from_strings(&strings);
    let mut ctx = BfsContext::new(grid.width(), grid.height());
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

    let (grid, start, goal) = grid_from_strings(&strings);
    let mut ctx = BfsContext::new(grid.width(), grid.height());
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

    let (grid, start, goal) = grid_from_strings(&strings);
    let mut ctx = BfsContext::new(grid.width(), grid.height());
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

    let (grid, _, goal) = grid_from_strings(&strings);

    let start = Coord::new(-1, -1);

    let mut ctx = BfsContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    let result = ctx.search(&grid, start, goal, Directions, &mut path);

    assert_eq!(result, Err(Error::StartOutsideGrid));
}
