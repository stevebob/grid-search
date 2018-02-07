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

                let index = self.node_grid.coord_to_index(neighbour_coord).expect(
                    "SearchContext too small for grid",
                );

                let node = &mut self.node_grid[index];

                Self::update_node_and_priority_queue(
                    index,
                    current_cost + neighbour_cost,
                    neighbour_coord,
                    direction,
                    self.seq,
                    &heuristic_fn,
                    goal,
                    node,
                    &mut self.priority_queue,
                );
            }
        }

        Err(Error::NoPath)
    }

    fn update_node_and_priority_queue<H>(
        index: usize,
        cost: Cost,
        neighbour_coord: Coord,
        direction: Direction,
        seq: u64,
        heuristic_fn: H,
        goal: Coord,
        neighbour_node: &mut SearchNode<Cost>,
        priority_queue: &mut BinaryHeap<PriorityEntry<Cost>>,
    ) where
        H: Fn(Coord, Coord) -> Cost,
    {
        if neighbour_node.seen != seq || neighbour_node.cost > cost {
            neighbour_node.from_parent = Some(direction);
            neighbour_node.seen = seq;
            neighbour_node.cost = cost;

            let heuristic = cost + heuristic_fn(neighbour_coord, goal);
            let entry = PriorityEntry::new(index, heuristic);
            priority_queue.push(entry);
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
        successor_coord: Coord,
        cost: f64,
        direction: Direction,
        goal: Coord,
        seq: u64,
        node_grid: &mut Grid<SearchNode<f64>>,
        priority_queue: &mut BinaryHeap<PriorityEntry<f64>>,
    ) {
        let index = node_grid.coord_to_index(successor_coord).expect(
            "SearchContext too small for grid",
        );

        let node = &mut node_grid[index];

        Self::update_node_and_priority_queue(
            index,
            cost,
            successor_coord,
            direction,
            seq,
            Self::octile_distance,
            goal,
            node,
            priority_queue,
        );
    }

    fn expand_cardinal<G>(
        grid: &G,
        current_coord: Coord,
        current_cost: f64,
        direction: CardinalDirection,
        goal: Coord,
        seq: u64,
        node_grid: &mut Grid<SearchNode<f64>>,
        priority_queue: &mut BinaryHeap<PriorityEntry<f64>>,
    ) where
        G: SolidGrid,
    {
        if let Some((successor_coord, successor_cost)) =
            Self::jump_cardinal(grid, current_coord, direction, goal)
        {
            Self::expand_common(
                successor_coord,
                current_cost + successor_cost,
                direction.direction(),
                goal,
                seq,
                node_grid,
                priority_queue,
            );
        }
    }

    fn expand_ordinal<G>(
        grid: &G,
        current_coord: Coord,
        current_cost: f64,
        direction: OrdinalDirection,
        goal: Coord,
        seq: u64,
        node_grid: &mut Grid<SearchNode<f64>>,
        priority_queue: &mut BinaryHeap<PriorityEntry<f64>>,
    ) where
        G: SolidGrid,
    {
        if let Some((successor_coord, successor_cost)) =
            Self::jump_ordinal(grid, current_coord, direction, goal)
        {
            Self::expand_common(
                successor_coord,
                current_cost + successor_cost,
                direction.direction(),
                goal,
                seq,
                node_grid,
                priority_queue,
            );
        }
    }

    fn expand_general<G>(
        grid: &G,
        current_coord: Coord,
        current_cost: f64,
        direction: Direction,
        goal: Coord,
        seq: u64,
        node_grid: &mut Grid<SearchNode<f64>>,
        priority_queue: &mut BinaryHeap<PriorityEntry<f64>>,
    ) where
        G: SolidGrid,
    {
        match direction.typ() {
            DirectionType::Cardinal(direction) => {
                Self::expand_cardinal(
                    grid,
                    current_coord,
                    current_cost,
                    direction,
                    goal,
                    seq,
                    node_grid,
                    priority_queue,
                )
            }
            DirectionType::Ordinal(direction) => {
                Self::expand_ordinal(
                    grid,
                    current_coord,
                    current_cost,
                    direction,
                    goal,
                    seq,
                    node_grid,
                    priority_queue,
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
            Self::expand_general(
                grid,
                start,
                initial_entry.cost,
                direction,
                goal,
                self.seq,
                &mut self.node_grid,
                &mut self.priority_queue,
            );
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {

            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                path::make_path(&self.node_grid, goal_index, path);
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
                    Self::expand_cardinal(
                        grid,
                        current_coord,
                        current_cost,
                        direction,
                        goal,
                        self.seq,
                        &mut self.node_grid,
                        &mut self.priority_queue,
                    );
                    let left = direction.left90();
                    if grid.is_solid_or_outside(current_coord + left.coord()) {
                        Self::expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            OrdinalDirection::from_cardinals(direction, left).unwrap(),
                            goal,
                            self.seq,
                            &mut self.node_grid,
                            &mut self.priority_queue,
                        );
                    }
                    let right = direction.right90();
                    if grid.is_solid_or_outside(current_coord + right.coord()) {
                        Self::expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            OrdinalDirection::from_cardinals(direction, right).unwrap(),
                            goal,
                            self.seq,
                            &mut self.node_grid,
                            &mut self.priority_queue,
                        );
                    }
                }
                DirectionType::Ordinal(direction) => {
                    Self::expand_ordinal(
                        grid,
                        current_coord,
                        current_cost,
                        direction,
                        goal,
                        self.seq,
                        &mut self.node_grid,
                        &mut self.priority_queue,
                    );
                    let (left, right) = direction.to_cardinals();
                    Self::expand_cardinal(
                        grid,
                        current_coord,
                        current_cost,
                        left,
                        goal,
                        self.seq,
                        &mut self.node_grid,
                        &mut self.priority_queue,
                    );
                    Self::expand_cardinal(
                        grid,
                        current_coord,
                        current_cost,
                        right,
                        goal,
                        self.seq,
                        &mut self.node_grid,
                        &mut self.priority_queue,
                    );

                    let (check_right, check_left) = direction.opposite().to_cardinals();

                    if grid.is_solid_or_outside(current_coord + check_left.coord()) {
                        Self::expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.left90(),
                            goal,
                            self.seq,
                            &mut self.node_grid,
                            &mut self.priority_queue,
                        );
                    }
                    if grid.is_solid_or_outside(current_coord + check_right.coord()) {
                        Self::expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.right90(),
                            goal,
                            self.seq,
                            &mut self.node_grid,
                            &mut self.priority_queue,
                        );
                    }
                }
            }
        }

        Err(Error::NoPath)
    }
}
