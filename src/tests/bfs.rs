use direction::*;
use grid_2d::*;
use grid::SolidGrid;
use path::PathWalk;
use bfs::*;
use dijkstra_map::*;
use error::*;
use config::*;

fn grid_from_strings(strings: &Vec<&str>) -> (Grid<bool>, Coord, Coord) {
    let width = strings[0].len() as u32;
    let height = strings.len() as u32;
    let size = Size::new(width, height);
    let mut grid = Grid::new_clone(size, false);
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
    fn is_solid(&self, coord: Coord) -> Option<bool> {
        self.get(coord).cloned()
    }
}

fn common_test<V, D>(strings: &Vec<&str>, directions: D, length: usize)
where
    V: Into<Direction>,
    D: Copy + IntoIterator<Item = V>,
{
    let (grid, start, goal) = grid_from_strings(strings);
    let mut ctx = BfsContext::new(grid.size());
    let mut path = Vec::new();
    let metadata = ctx.bfs(
        &grid,
        start,
        goal,
        directions,
        Default::default(),
        &mut path,
    ).unwrap();

    println!("{:?}", metadata);

    assert_eq!(path.len(), length);

    let walk = PathWalk::new(start, &path);

    let (should_be_goal, _) = walk.inspect(|&(coord, _)| {
        assert_eq!(grid.is_solid(coord), Some(false));
    }).last()
        .unwrap_or((start, Direction::North));

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
    let mut ctx = BfsContext::new(grid.size());
    let mut path = Vec::new();
    let result = ctx.bfs(
        &grid,
        start,
        goal,
        Directions,
        Default::default(),
        &mut path,
    );

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
    let mut ctx = BfsContext::new(grid.size());
    let mut path = Vec::new();
    let result = ctx.bfs(
        &grid,
        start,
        goal,
        Directions,
        Default::default(),
        &mut path,
    );

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
    let mut ctx = BfsContext::new(grid.size());
    let mut path = Vec::new();
    let result = ctx.bfs(
        &grid,
        start,
        goal,
        Directions,
        BfsConfig {
            max_depth: ::std::usize::MAX,
            allow_solid_start: false,
        },
        &mut path,
    );

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

    let mut ctx = BfsContext::new(grid.size());
    let mut path = Vec::new();
    let result = ctx.bfs(
        &grid,
        start,
        goal,
        Directions,
        Default::default(),
        &mut path,
    );

    assert_eq!(result, Err(Error::StartOutsideGrid));
}

#[test]
fn dijkstra_map() {
    let strings = vec!["..#.", "..##", ".#.B", ".#.."];

    let (grid, start, _) = grid_from_strings(&strings);

    let mut ctx = BfsContext::new(grid.size());
    let mut dijkstra_map: DijkstraMap<u32> = DijkstraMap::new(ctx.size());

    let result = ctx.populate_dijkstra_map(
        &grid,
        start,
        Directions,
        Default::default(),
        &mut dijkstra_map,
    ).unwrap();

    assert_eq!(result.num_nodes_visited, 10);

    assert!(dijkstra_map.get(Coord::new(10, 10)).is_outside());
    assert!(dijkstra_map.get(Coord::new(3, 2)).is_origin());
    assert!(dijkstra_map.get(Coord::new(3, 0)).is_unvisited());
    assert!(dijkstra_map.get(Coord::new(1, 2)).is_unvisited());
    assert_eq!(dijkstra_map.get(Coord::new(0, 3)).cell().unwrap().cost(), 4);
    assert_eq!(
        dijkstra_map
            .get(Coord::new(0, 3))
            .cell()
            .unwrap()
            .direction(),
        Direction::North
    );
    assert_eq!(dijkstra_map.get(Coord::new(1, 1)).cell().unwrap().cost(), 2);
    assert_eq!(
        dijkstra_map
            .get(Coord::new(1, 1))
            .cell()
            .unwrap()
            .direction(),
        Direction::SouthEast
    );
}

#[test]
fn dijkstra_map_cardinal() {
    let strings = vec!["..#.", "...#", ".#.B", ".#.."];

    let (grid, start, _) = grid_from_strings(&strings);

    let mut ctx = BfsContext::new(grid.size());
    let mut dijkstra_map: DijkstraMap<u32> = DijkstraMap::new(ctx.size());

    let result = ctx.populate_dijkstra_map(
        &grid,
        start,
        CardinalDirections,
        Default::default(),
        &mut dijkstra_map,
    ).unwrap();

    assert_eq!(result.num_nodes_visited, 11);

    assert!(dijkstra_map.get(Coord::new(10, 10)).is_outside());
    assert!(dijkstra_map.get(Coord::new(3, 2)).is_origin());
    assert!(dijkstra_map.get(Coord::new(3, 0)).is_unvisited());
    assert!(dijkstra_map.get(Coord::new(1, 2)).is_unvisited());
    assert_eq!(dijkstra_map.get(Coord::new(0, 3)).cell().unwrap().cost(), 6);
    assert_eq!(
        dijkstra_map
            .get(Coord::new(0, 3))
            .cell()
            .unwrap()
            .direction(),
        Direction::North
    );
    assert_eq!(dijkstra_map.get(Coord::new(1, 1)).cell().unwrap().cost(), 3);
    assert_eq!(
        dijkstra_map
            .get(Coord::new(1, 1))
            .cell()
            .unwrap()
            .direction(),
        Direction::East
    );
}

#[test]
fn bfs_best() {
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

    let (grid, start, _) = grid_from_strings(&strings);

    let mut ctx = BfsContext::new(grid.size());
    let mut path = Vec::new();

    let score = |coord: Coord| Some(coord.x + coord.y);

    let metadata = ctx.bfs_best(
        &grid,
        start,
        score,
        Directions,
        Default::default(),
        &mut path,
    ).unwrap();

    assert_eq!(metadata.length, 8);

    let walk = PathWalk::new(start, &path);

    let (should_be_bottom_right, _) = walk.inspect(|&(coord, _)| {
        assert_eq!(grid.is_solid(coord), Some(false));
    }).last()
        .unwrap_or((start, Direction::North));

    assert_eq!(should_be_bottom_right, Coord::new(9, 9));
}
