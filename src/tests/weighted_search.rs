use direction::*;
use grid_2d::*;
use grid::*;
use path::PathWalk;
use weighted_search::*;
use error::*;
use metadata::*;

const DEFAULT_ORDINAL_MULTIPLIER: u32 = 2;

struct TestGrid {
    grid: Grid<Option<u32>>,
    ordinal_multiplier: u32,
}

impl TestGrid {
    fn width(&self) -> u32 {
        self.grid.width()
    }
    fn height(&self) -> u32 {
        self.grid.height()
    }
}

impl SolidGrid for TestGrid {
    fn is_solid(&self, coord: Coord) -> bool {
        self.grid.get(coord).cloned().unwrap().is_none()
    }
}

impl CostGrid for TestGrid {
    fn cost(&self, coord: Coord, direction: Direction) -> Option<u32> {
        let cost = self.grid.get(coord).cloned().unwrap()?;
        if direction.is_ordinal() {
            Some(cost * self.ordinal_multiplier)
        } else {
            Some(cost)
        }
    }
}

fn grid_from_strings(strings: &Vec<&str>, ordinal_multiplier: u32) -> (TestGrid, Coord, Coord) {
    let width = strings[0].len() as u32;
    let height = strings.len() as u32;
    let mut grid = Grid::new_copy(width, height, Some(1));
    let mut start = None;
    let mut goal = None;
    for (i, line) in strings.into_iter().enumerate() {
        for (j, ch) in line.chars().enumerate() {
            let coord = Coord::new(j as i32, i as i32);
            match ch {
                '.' => (),
                ',' => *grid.get_mut(coord).unwrap() = Some(10),
                '#' => *grid.get_mut(coord).unwrap() = None,
                's' => start = Some(coord),
                'g' => goal = Some(coord),
                'B' => {
                    goal = Some(coord);
                    start = Some(coord);
                }
                'G' => {
                    goal = Some(coord);
                    *grid.get_mut(coord).unwrap() = None;
                }
                'S' => {
                    start = Some(coord);
                    *grid.get_mut(coord).unwrap() = None;
                }
                _ => panic!(),
            }
        }
    }

    let grid = TestGrid {
        grid,
        ordinal_multiplier,
    };

    (grid, start.unwrap(), goal.unwrap())
}

fn common_test(
    strings: &Vec<&str>,
    ordinal_multiplier: u32,
    cardinal_only: bool,
    length: usize,
    cost: u32,
) {
    let (grid, start, goal) = grid_from_strings(strings, ordinal_multiplier);
    let mut ctx = WeightedSearchContext::new(grid.width(), grid.height());

    let check_result = |path: &Vec<_>, metadata: SearchMetadata| {

        assert_eq!(path.len(), length);

        let walk = PathWalk::new(start, &path);

        let (should_be_goal, total_cost) =
            walk.fold((start, 0), |(_, total_cost), (coord, direction)| {
                if let Some(cost) = grid.cost(coord, direction) {
                    (coord, total_cost + cost)
                } else {
                    panic!("Path goes through wall");
                }
            });

        assert_eq!(should_be_goal, goal);
        assert_eq!(total_cost, cost);

        metadata.num_nodes_visited
    };

    let mut path = Vec::new();

    let (with_heuristic, without_heuristic) = if cardinal_only {

        let metadata = ctx.search(&grid, start, goal, DirectionsCardinal, &mut path)
            .unwrap();
        let without_heuristic = check_result(&path, metadata);

        let metadata =
            ctx.search_cardinal_manhatten_distance_heuristic(&grid, start, goal, &mut path)
                .unwrap();
        let with_heuristic = check_result(&path, metadata);

        (with_heuristic, without_heuristic)

    } else {

        let metadata = ctx.search(&grid, start, goal, Directions, &mut path)
            .unwrap();
        let without_heuristic = check_result(&path, metadata);

        let weights = HeuristicDirectionWeights::new(1, ordinal_multiplier);

        let metadata =
            ctx.search_diagonal_distance_heuristic(&grid, start, goal, weights, &mut path)
                .unwrap();
        let with_heuristic = check_result(&path, metadata);

        (with_heuristic, without_heuristic)
    };

    println!(
        "Nodes visited: {}, Nodes visited with heuristic, {}",
        without_heuristic,
        with_heuristic
    );
}

#[test]
fn uniform_wall() {
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
    common_test(&strings, 1, true, 12, 12);
    common_test(&strings, 1, false, 7, 7);
}

#[test]
fn non_uniform_wall() {
    let strings = vec![
        "....,.....",
        "....#.....",
        "....#.....",
        "....#.....",
        ".s..#.....",
        "....#...g.",
        "....#,....",
        "....,,....",
        "....,,....",
        "....,,....",
    ];
    common_test(&strings, 1, true, 16, 25);
    common_test(&strings, 1, false, 9, 18);
}

#[test]
fn cheap_route() {
    let strings = vec![
        "..........",
        "..........",
        "..........",
        ",,,.,,,,..",
        ".s,...,,..",
        "..,..,,...",
        "..,..,g...",
        ".....,,...",
        "....,,,...",
        "....,,,...",
    ];
    common_test(&strings, 1, true, 9, 18);
    common_test(&strings, 1, false, 15, 15);
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

    let (grid, start, goal) = grid_from_strings(&strings, DEFAULT_ORDINAL_MULTIPLIER);
    let mut ctx = WeightedSearchContext::new(grid.width(), grid.height());
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
        "..........",
    ];
    common_test(&strings, DEFAULT_ORDINAL_MULTIPLIER, true, 0, 0);
    common_test(&strings, DEFAULT_ORDINAL_MULTIPLIER, false, 0, 0);
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

    let (grid, start, goal) = grid_from_strings(&strings, DEFAULT_ORDINAL_MULTIPLIER);
    let mut ctx = WeightedSearchContext::new(grid.width(), grid.height());
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

    let (grid, start, goal) = grid_from_strings(&strings, DEFAULT_ORDINAL_MULTIPLIER);
    let mut ctx = WeightedSearchContext::new(grid.width(), grid.height());
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

    let (grid, _, goal) = grid_from_strings(&strings, DEFAULT_ORDINAL_MULTIPLIER);

    let start = Coord::new(-1, -1);

    let mut ctx = WeightedSearchContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    let result = ctx.search(&grid, start, goal, Directions, &mut path);

    assert_eq!(result, Err(Error::StartOutsideGrid));
}

#[test]
fn simple_optimality() {
    let strings = vec![
        "....g.....",
        "...s......",
        "..........",
        "..........",
        "..........",
    ];

    common_test(&strings, 20, false, 2, 2);
    common_test(&strings, 1, false, 1, 1);
}
