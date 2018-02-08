use std::collections::BinaryHeap;
use std::ops::{Add, Mul};
use std::cmp::Ordering;
use num_traits::{Zero, NumCast};
use direction::*;
use grid_2d::*;
use grid::*;
use error::*;
use metadata::*;
use path::{self, PathNode};

#[derive(Debug, Clone, Copy)]
struct SearchNode<Cost: Add<Cost> + PartialOrd<Cost>> {
    seen: u64,
    visited: u64,
    coord: Coord,
    from_parent: Option<Direction>,
    cost: Cost,
}

impl<Cost: Add<Cost> + PartialOrd<Cost> + Zero> From<Coord> for SearchNode<Cost> {
    fn from(coord: Coord) -> Self {
        Self {
            seen: 0,
            visited: 0,
            coord,
            from_parent: None,
            cost: Zero::zero(),
        }
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PathNode for SearchNode<Cost> {
    fn from_parent(&self) -> Option<Direction> {
        self.from_parent
    }
    fn coord(&self) -> Coord {
        self.coord
    }
}

#[derive(Debug, Clone)]
struct PriorityEntry<Cost: Add<Cost> + PartialOrd<Cost>> {
    node_index: usize,
    cost: Cost,
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PriorityEntry<Cost> {
    fn new(node_index: usize, cost: Cost) -> Self {
        Self { node_index, cost }
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PartialEq for PriorityEntry<Cost> {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> PartialOrd for PriorityEntry<Cost> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.cost.partial_cmp(&self.cost)
    }
}

impl<Cost: Add<Cost> + PartialOrd<Cost>> Eq for PriorityEntry<Cost> {}

impl<Cost: Add<Cost> + PartialOrd<Cost>> Ord for PriorityEntry<Cost> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.partial_cmp(&self.cost).unwrap_or(
            Ordering::Equal,
        )
    }
}

#[derive(Debug, Clone)]
pub struct SearchContext<Cost: Add<Cost> + PartialOrd<Cost>> {
    seq: u64,
    priority_queue: BinaryHeap<PriorityEntry<Cost>>,
    node_grid: Grid<SearchNode<Cost>>,
}


impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            seq: 0,
            node_grid: Grid::new_from_coord(width, height),
            priority_queue: BinaryHeap::new(),
        }
    }

    pub fn search<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        self.search_general(grid, start, goal, directions, |_, _| Zero::zero(), path)
    }

    fn init<G>(
        &mut self,
        start: Coord,
        goal: Coord,
        grid: &G,
        path: &mut Vec<Direction>,
    ) -> Result<PriorityEntry<Cost>, Result<SearchMetadata, Error>>
    where
        G: SolidGrid,
    {
        if let Some(solid) = grid.is_solid(start) {

            let index = self.node_grid.coord_to_index(start).expect(
                "SearchContext too small for grid",
            );

            if solid {
                return Err(Err(Error::StartSolid));
            };

            if start == goal {
                path.clear();
                return Err(Ok(Default::default()));
            }

            self.seq += 1;
            self.priority_queue.clear();

            let node = &mut self.node_grid[index];
            node.from_parent = None;
            node.seen = self.seq;
            node.cost = Zero::zero();

            Ok(PriorityEntry::new(index, Zero::zero()))
        } else {
            Err(Err(Error::StartOutsideGrid))
        }
    }

    fn search_general<G, V, D, H>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        heuristic_fn: H,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        H: Fn(Coord, Coord) -> Cost,
    {
        let initial_entry = match self.init(start, goal, grid, path) {
            Ok(initial_entry) => initial_entry,
            Err(result) => return result,
        };

        self.priority_queue.push(initial_entry);

        let goal_index = self.node_grid.coord_to_index(goal).expect(
            "SearchContext too small for grid",
        );

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {

            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                path::make_path(&self.node_grid, goal_index, path);
                return Ok(SearchMetadata { num_nodes_visited });
            }

            let (current_coord, current_cost) = {
                let node = &mut self.node_grid[current_entry.node_index];
                if node.visited == self.seq {
                    continue;
                }
                node.visited = self.seq;
                (node.coord, node.cost)
            };

            for d in directions {
                let direction = d.into();
                let neighbour_coord = current_coord + direction.coord();

                let neighbour_cost =
                    if let Some(CostCell::Cost(cost)) = grid.cost(neighbour_coord, direction) {
                        cost
                    } else {
                        continue;
                    };

                self.see_successor(
                    current_cost + neighbour_cost,
                    neighbour_coord,
                    direction,
                    &heuristic_fn,
                    goal,
                );
            }
        }

        Err(Error::NoPath)
    }

    fn see_successor<H>(
        &mut self,
        cost: Cost,
        successor_coord: Coord,
        direction: Direction,
        heuristic_fn: H,
        goal: Coord,
    ) where
        H: Fn(Coord, Coord) -> Cost,
    {
        let index = self.node_grid.coord_to_index(successor_coord).expect(
            "SearchContext too small for grid",
        );

        let node = &mut self.node_grid[index];

        if node.seen != self.seq || node.cost > cost {
            node.from_parent = Some(direction);
            node.seen = self.seq;
            node.cost = cost;

            let heuristic = cost + heuristic_fn(successor_coord, goal);
            let entry = PriorityEntry::new(index, heuristic);
            self.priority_queue.push(entry);
        }
    }
}

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + NumCast + Zero> SearchContext<Cost> {
    fn manhatten_distance(a: Coord, b: Coord) -> i32 {
        (a.x - b.x).abs() + (a.y - b.y).abs()
    }

    pub fn search_cardinal_manhatten_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
    {

        let heuristic_fn =
            |a, b| NumCast::from(Self::manhatten_distance(a, b)).expect("Failed to cast to Cost");

        self.search_general(grid, start, goal, DirectionsCardinal, heuristic_fn, path)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HeuristicDirectionWeights<Cost: Add<Cost> + PartialOrd<Cost>> {
    pub cardinal: Cost,
    pub ordinal: Cost,
}

impl<Cost: Add<Cost> + PartialOrd<Cost> + Zero> HeuristicDirectionWeights<Cost> {
    pub fn new(cardinal: Cost, ordinal: Cost) -> Self {
        Self { cardinal, ordinal }
    }
}

impl<Cost> SearchContext<Cost>
where
    Cost: Copy
        + Add<Cost, Output = Cost>
        + Mul<Cost, Output = Cost>
        + PartialOrd<Cost>
        + NumCast
        + Zero,
{
    fn diagonal_distance(a: Coord, b: Coord, weights: &HeuristicDirectionWeights<Cost>) -> Cost {
        let dx = (a.x - b.x).abs();
        let dy = (a.y - b.y).abs();
        let (cardinal, ordinal) = if dx < dy {
            (dy - dx, dx)
        } else {
            (dx - dy, dy)
        };

        let cardinal: Cost = NumCast::from(cardinal).expect("Failed to cast to Cost");
        let ordinal: Cost = NumCast::from(ordinal).expect("Failed to cast to Cost");

        let cardinal: Cost = cardinal * weights.cardinal;
        let ordinal: Cost = ordinal * weights.ordinal;

        cardinal + ordinal
    }

    pub fn search_diagonal_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        weights: HeuristicDirectionWeights<Cost>,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: CostGrid<Cost = Cost>,
    {
        let heuristic_fn = |a, b| Self::diagonal_distance(a, b, &weights);
        self.search_general(grid, start, goal, Directions, heuristic_fn, path)
    }
}

fn jump_point_make_path<Cost: Add<Cost> + PartialOrd<Cost>>(
    node_grid: &Grid<SearchNode<Cost>>,
    goal_coord: Coord,
    seq: u64,
    path: &mut Vec<Direction>
) {
    path.clear();

    let mut node = node_grid.get(goal_coord).expect("Invalid search state");

    loop {
        let from_parent = if let Some(from_parent) = node.from_parent() {
            from_parent
        } else {
            break;
        };

        path.push(from_parent);

        let step = from_parent.opposite().coord();
        let mut coord = node.coord;
        loop {
            coord += step;
            let next_node = node_grid.get(coord).expect("Invalid search state");
            if next_node.seen == seq {
                node = next_node;
                break;
            }

            path.push(from_parent);
        }
    }

    path.reverse();
}

impl SearchContext<f64> {
    fn octile_distance(a: Coord, b: Coord) -> f64 {
        let dx = (a.x - b.x).abs();
        let dy = (a.y - b.y).abs();
        let (cardinal, ordinal) = if dx > dy { (dx, dx) } else { (dy, dx) };

        const SQRT_2_MIN_1: f64 = ::std::f64::consts::SQRT_2 - 1.0;
        cardinal as f64 + ordinal as f64 * SQRT_2_MIN_1
    }

    fn has_forced_neighbour_cardinal<G>(
        grid: &G,
        coord: Coord,
        direction: CardinalDirection,
    ) -> bool
    where
        G: SolidGrid,
    {
        if grid.is_solid_or_outside(coord + direction.left90().coord()) {
            return true;
        }

        if grid.is_solid_or_outside(coord + direction.right90().coord()) {
            return true;
        }

        false
    }

    fn has_forced_neighbour_ordinal<G>(grid: &G, coord: Coord, direction: OrdinalDirection) -> bool
    where
        G: SolidGrid,
    {
        let (left, right) = direction.opposite().to_cardinals();

        if grid.is_solid_or_outside(coord + left.coord()) {
            return true;
        }

        if grid.is_solid_or_outside(coord + right.coord()) {
            return true;
        }

        false
    }

    fn jump_cardinal<G>(
        grid: &G,
        coord: Coord,
        direction: CardinalDirection,
        goal: Coord,
    ) -> Option<(Coord, f64)>
    where
        G: SolidGrid,
    {
        let neighbour_coord = coord + direction.coord();

        if grid.is_solid_or_outside(neighbour_coord) {
            return None;
        }

        const COST: f64 = 1.0;

        if neighbour_coord == goal {
            return Some((neighbour_coord, COST));
        }

        if Self::has_forced_neighbour_cardinal(grid, neighbour_coord, direction) {
            return Some((neighbour_coord, COST));
        }

        Self::jump_cardinal(grid, neighbour_coord, direction, goal)
            .map(|(coord, cost)| (coord, cost + COST))
    }

    fn jump_ordinal<G>(
        grid: &G,
        coord: Coord,
        direction: OrdinalDirection,
        goal: Coord,
    ) -> Option<(Coord, f64)>
    where
        G: SolidGrid,
    {
        let neighbour_coord = coord + direction.coord();

        if grid.is_solid_or_outside(neighbour_coord) {
            return None;
        }

        const COST: f64 = ::std::f64::consts::SQRT_2;

        if neighbour_coord == goal {
            return Some((neighbour_coord, COST));
        }

        if Self::has_forced_neighbour_ordinal(grid, neighbour_coord, direction) {
            return Some((neighbour_coord, COST));
        }

        let (card0, card1) = direction.to_cardinals();

        if Self::jump_cardinal(grid, neighbour_coord, card0, goal).is_some() ||
            Self::jump_cardinal(grid, neighbour_coord, card1, goal).is_some()
        {
            return Some((neighbour_coord, COST));
        }

        Self::jump_ordinal(grid, neighbour_coord, direction, goal)
            .map(|(coord, cost)| (coord, cost + COST))
    }

    fn expand_common(
        &mut self,
        successor_coord: Coord,
        cost: f64,
        direction: Direction,
        goal: Coord,
    ) {
        self.see_successor(
            cost,
            successor_coord,
            direction,
            Self::octile_distance,
            goal,
        );
    }

    fn expand_cardinal<G>(
        &mut self,
        grid: &G,
        current_coord: Coord,
        current_cost: f64,
        direction: CardinalDirection,
        goal: Coord,
    ) where
        G: SolidGrid,
    {
        if let Some((successor_coord, successor_cost)) =
            Self::jump_cardinal(grid, current_coord, direction, goal)
        {
            self.expand_common(
                successor_coord,
                current_cost + successor_cost,
                direction.direction(),
                goal,
            );
        }
    }

    fn expand_ordinal<G>(
        &mut self,
        grid: &G,
        current_coord: Coord,
        current_cost: f64,
        direction: OrdinalDirection,
        goal: Coord,
    ) where
        G: SolidGrid,
    {
        if let Some((successor_coord, successor_cost)) =
            Self::jump_ordinal(grid, current_coord, direction, goal)
        {
            self.expand_common(
                successor_coord,
                current_cost + successor_cost,
                direction.direction(),
                goal,
            );
        }
    }

    fn expand_general<G>(
        &mut self,
        grid: &G,
        current_coord: Coord,
        current_cost: f64,
        direction: Direction,
        goal: Coord,
    ) where
        G: SolidGrid,
    {
        match direction.typ() {
            DirectionType::Cardinal(direction) => {
                self.expand_cardinal(
                    grid,
                    current_coord,
                    current_cost,
                    direction,
                    goal,
                )
            }
            DirectionType::Ordinal(direction) => {
                self.expand_ordinal(
                    grid,
                    current_coord,
                    current_cost,
                    direction,
                    goal,
                )
            }
        }
    }

    pub fn search_jump_point<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata, Error>
    where
        G: SolidGrid,
    {
        let initial_entry = match self.init(start, goal, grid, path) {
            Ok(initial_entry) => initial_entry,
            Err(result) => return result,
        };

        let goal_index = self.node_grid.coord_to_index(goal).expect(
            "SearchContext too small for grid",
        );

        for direction in Directions {
            self.expand_general(
                grid,
                start,
                initial_entry.cost,
                direction,
                goal,
            );
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {

            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                jump_point_make_path(&self.node_grid, goal, self.seq, path);
                return Ok(SearchMetadata { num_nodes_visited });
            }

            let (current_coord, current_cost, direction) = {
                let node = &mut self.node_grid[current_entry.node_index];
                if node.visited == self.seq {
                    continue;
                }
                node.visited = self.seq;
                let direction = node.from_parent.expect("Open set node without direction");
                (node.coord, node.cost, direction)
            };

            match direction.typ() {
                DirectionType::Cardinal(direction) => {
                    self.expand_cardinal(
                        grid,
                        current_coord,
                        current_cost,
                        direction,
                        goal,
                    );
                    let left = direction.left90();
                    if grid.is_solid_or_outside(current_coord + left.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.left45(),
                            goal,
                        );
                    }
                    let right = direction.right90();
                    if grid.is_solid_or_outside(current_coord + right.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.right45(),
                            goal,
                        );
                    }
                }
                DirectionType::Ordinal(direction) => {
                    self.expand_ordinal(
                        grid,
                        current_coord,
                        current_cost,
                        direction,
                        goal,
                    );
                    let (left, right) = direction.to_cardinals();
                    self.expand_cardinal(
                        grid,
                        current_coord,
                        current_cost,
                        left,
                        goal,
                    );
                    self.expand_cardinal(
                        grid,
                        current_coord,
                        current_cost,
                        right,
                        goal,
                    );

                    let (check_right, check_left) = direction.opposite().to_cardinals();

                    if grid.is_solid_or_outside(current_coord + check_left.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.left90(),
                            goal,
                        );
                    }
                    if grid.is_solid_or_outside(current_coord + check_right.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.right90(),
                            goal,
                        );
                    }
                }
            }
        }

        Err(Error::NoPath)
    }
}
