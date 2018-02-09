use direction::*;
use grid_2d::*;
use grid::*;
use path::PathWalk;
use search::*;
use dijkstra_map::*;
use astar::*;
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
    fn is_solid(&self, coord: Coord) -> Option<bool> {
        self.grid.get(coord).map(Option::is_none)
    }
}

impl CostGrid for TestGrid {
    type Cost = u32;
    fn cost(&self, coord: Coord, direction: Direction) -> Option<CostCell<Self::Cost>> {
        if let Some(cost) = self.grid.get(coord).cloned()? {
            if direction.is_ordinal() {
                Some(CostCell::Cost(cost * self.ordinal_multiplier))
            } else {
                Some(CostCell::Cost(cost))
            }
        } else {
            None
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

#[derive(Clone, Copy, PartialEq, Eq)]
enum WhichDirections {
    Cardinal,
    All,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum GridWeights {
    Weighted,
    Uniform,
}

use self::WhichDirections::*;
use self::GridWeights::*;

fn common_test(
    strings: &Vec<&str>,
    ordinal_multiplier: u32,
    which_directions: WhichDirections,
    grid_weights: GridWeights,
    length: usize,
    cost: u32,
) {
    let (grid, start, goal) = grid_from_strings(strings, ordinal_multiplier);
    let mut ctx = SearchContext::new(grid.width(), grid.height());

    let check_result = |path: &Vec<_>, metadata: SearchMetadata| {
        assert_eq!(path.len(), length);

        let walk = PathWalk::new(start, &path);

        let (should_be_goal, total_cost) =
            walk.fold((start, 0), |(_, total_cost), (coord, direction)| {
                if let CostCell::Cost(cost) = grid.cost(coord, direction).unwrap() {
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

    let (with_heuristic, without_heuristic, jps) = if which_directions == Cardinal {
        let metadata = ctx.dijkstra(&grid, start, goal, DirectionsCardinal, &mut path)
            .unwrap();
        let without_heuristic = check_result(&path, metadata);

        let metadata =
            ctx.astar_cardinal_manhatten_distance_heuristic(&grid, start, goal, &mut path)
                .unwrap();
        let with_heuristic = check_result(&path, metadata);

        let jps = if grid_weights == Uniform {
            let metadata = ctx.jump_point_search_cardinal_manhatten_distance_heuristic(
                &grid,
                start,
                goal,
                &mut path,
            ).unwrap();
            Some(check_result(&path, metadata))
        } else {
            None
        };

        (with_heuristic, without_heuristic, jps)
    } else {
        let metadata = ctx.dijkstra(&grid, start, goal, Directions, &mut path)
            .unwrap();
        let without_heuristic = check_result(&path, metadata);

        let weights = HeuristicDirectionWeights::new(1, ordinal_multiplier);

        let metadata =
            ctx.astar_diagonal_distance_heuristic(&grid, start, goal, weights, &mut path)
                .unwrap();
        let with_heuristic = check_result(&path, metadata);

        let jps = if grid_weights == Uniform {
            let mut jps_ctx = SearchContext::new(grid.width(), grid.height());
            let metadata = jps_ctx
                .jump_point_search_octile_distance_heuristic(&grid, start, goal, &mut path)
                .unwrap();
            Some(check_result(&path, metadata))
        } else {
            None
        };

        (with_heuristic, without_heuristic, jps)
    };

    println!(
        "Nodes visited: {}, With heuristic: {}, With JPS: {:?}",
        without_heuristic, with_heuristic, jps
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
    common_test(&strings, 1, Cardinal, Uniform, 12, 12);
    common_test(&strings, 1, All, Uniform, 7, 7);
}

#[test]
fn uniform_corridors() {
    let strings = vec![
        "#####.####",
        "#........#",
        "###.#.##.#",
        "###.#.##.#",
        "#s#.#..###",
        "#.#.#.##g#",
        "#.#.#.##.#",
        "#...#....#",
        "##.####.##",
        ".####.####",
    ];
    common_test(&strings, 1, Cardinal, Uniform, 24, 24);
    common_test(&strings, 1, All, Uniform, 18, 18);
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
    common_test(&strings, 1, Cardinal, Weighted, 16, 25);
    common_test(&strings, 1, All, Weighted, 9, 18);
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
    common_test(&strings, 1, Cardinal, Weighted, 9, 18);
    common_test(&strings, 1, All, Weighted, 15, 15);
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
    let mut ctx = SearchContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    let result = ctx.dijkstra(&grid, start, goal, Directions, &mut path);

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
    common_test(
        &strings,
        DEFAULT_ORDINAL_MULTIPLIER,
        Cardinal,
        Uniform,
        0,
        0,
    );
    common_test(&strings, DEFAULT_ORDINAL_MULTIPLIER, All, Uniform, 0, 0);
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
    let mut ctx = SearchContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    let result = ctx.dijkstra(&grid, start, goal, Directions, &mut path);

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
    let mut ctx = SearchContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    let result = ctx.dijkstra(&grid, start, goal, Directions, &mut path);

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

    let mut ctx = SearchContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    let result = ctx.dijkstra(&grid, start, goal, Directions, &mut path);

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

    common_test(&strings, 20, All, Weighted, 2, 2);
    common_test(&strings, 1, All, Uniform, 1, 1);
}

#[test]
fn jps() {
    let strings = vec![
        "....#.....",
        "....#.....",
        ".s..#.....",
        "....#.....",
        "....#.....",
        "....#...g.",
        "....#.....",
        "....#.....",
        "....#.....",
        "..........",
    ];

    let (grid, start, goal) = grid_from_strings(&strings, DEFAULT_ORDINAL_MULTIPLIER);

    let mut ctx = SearchContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    ctx.jump_point_search_octile_distance_heuristic(&grid, start, goal, &mut path)
        .unwrap();
    assert_eq!(path.len(), 11);
}

#[test]
fn cardinal_jps() {
    let strings = vec![
        "....#.....",
        "....#.....",
        ".s..#.....",
        "....#.....",
        "....#.....",
        "....#...g.",
        "....#.....",
        "....#.....",
        "....#.....",
        "..........",
    ];

    let (grid, start, goal) = grid_from_strings(&strings, DEFAULT_ORDINAL_MULTIPLIER);

    let mut ctx: SearchContext<u32> = SearchContext::new(grid.width(), grid.height());
    let mut path = Vec::new();
    ctx.jump_point_search_cardinal_manhatten_distance_heuristic(&grid, start, goal, &mut path)
        .unwrap();
    assert_eq!(path.len(), 18);
}

#[test]
fn dijkstra_map() {
    let strings = vec![
        "..#.",
        "..##",
        ".#.B",
        ".#..",
    ];

    let (grid, start, _) = grid_from_strings(&strings, DEFAULT_ORDINAL_MULTIPLIER);

    let mut ctx: SearchContext<u32> = SearchContext::new(grid.width(), grid.height());
    let mut dijkstra_map: DijkstraMap<u32> = DijkstraMap::new(ctx.width(), ctx.height());

    let result = ctx.populate_dijkstra_map(&grid, start, Directions, &mut dijkstra_map)
        .unwrap();

    assert_eq!(result.num_nodes_visited, 10);

    assert!(dijkstra_map.get(Coord::new(10, 10)).is_outside());
    assert!(dijkstra_map.get(Coord::new(3, 2)).is_origin());
    assert!(dijkstra_map.get(Coord::new(3, 0)).is_unvisited());
    assert!(dijkstra_map.get(Coord::new(1, 2)).is_unvisited());
    assert_eq!(dijkstra_map.get(Coord::new(0, 3)).cell().unwrap().cost(), 6);
    assert_eq!(dijkstra_map.get(Coord::new(0, 3)).cell().unwrap().direction(), Direction::North);
    assert_eq!(dijkstra_map.get(Coord::new(1, 1)).cell().unwrap().cost(), 3);
    assert_eq!(dijkstra_map.get(Coord::new(1, 1)).cell().unwrap().direction(), Direction::SouthEast);
}

#[test]
fn dijkstra_map_weights() {
    let strings = vec![
        "...",
        ".B.",
        "...",
    ];

    let (grid, start, _) = grid_from_strings(&strings, 20);

    let mut ctx: SearchContext<u32> = SearchContext::new(grid.width(), grid.height());
    let mut dijkstra_map: DijkstraMap<u32> = DijkstraMap::new(ctx.width(), ctx.height());

    ctx.populate_dijkstra_map(&grid, start, Directions, &mut dijkstra_map)
        .unwrap();

    assert_eq!(dijkstra_map.get(Coord::new(0, 0)).cell().unwrap().cost(), 2);
}
