use direction::*;
use grid_2d::*;
use metadata::*;
use search::*;
use error::*;
use grid::*;
use path;

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

    if has_forced_neighbour_cardinal(grid, neighbour_coord, direction) {
        return Some((neighbour_coord, COST));
    }

    jump_cardinal(grid, neighbour_coord, direction, goal)
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

    if has_forced_neighbour_ordinal(grid, neighbour_coord, direction) {
        return Some((neighbour_coord, COST));
    }

    let (card0, card1) = direction.to_cardinals();

    if jump_cardinal(grid, neighbour_coord, card0, goal).is_some() ||
        jump_cardinal(grid, neighbour_coord, card1, goal).is_some()
        {
            return Some((neighbour_coord, COST));
        }

    jump_ordinal(grid, neighbour_coord, direction, goal)
        .map(|(coord, cost)| (coord, cost + COST))
}

impl SearchContext<f64> {
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
            octile_distance,
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
            jump_cardinal(grid, current_coord, direction, goal)
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
            jump_ordinal(grid, current_coord, direction, goal)
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

    pub fn jump_point_search_octile_distance_heuristic<G>(
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
                path::make_path_jump_points(&self.node_grid, goal, self.seq, path);
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
