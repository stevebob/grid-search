use std::ops::*;
use num::traits::*;
use direction::*;
use metadata::*;
use config::*;
use search::*;
use error::*;
use grid::*;
use path;

fn octile_distance<C>(a: Coord, b: Coord) -> C
where
    C: Add<C, Output = C> + Mul<C, Output = C> + FloatConst + NumCast,
{
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    let (cardinal, ordinal) = if dx > dy {
        (dx - dy, dy)
    } else {
        (dy - dx, dx)
    };

    let sqrt_2: C = FloatConst::SQRT_2();
    let cardinal: C = NumCast::from(cardinal).expect("Failed to cast to Cost");
    let ordinal: C = NumCast::from(ordinal).expect("Failed to cast to Cost");

    cardinal + ordinal * sqrt_2
}

fn has_forced_neighbour_cardinal<G>(grid: &G, coord: Coord, direction: CardinalDirection) -> bool
where
    G: SolidGrid,
{
    if grid.is_solid_or_outside(coord + direction.left90().coord()) {
        return !grid.is_solid_or_outside(coord + direction.left45().coord());
    }

    if grid.is_solid_or_outside(coord + direction.right90().coord()) {
        return !grid.is_solid_or_outside(coord + direction.right45().coord());
    }

    false
}

fn has_forced_neighbour_ordinal<G>(grid: &G, coord: Coord, direction: OrdinalDirection) -> bool
where
    G: SolidGrid,
{
    if grid.is_solid_or_outside(coord + direction.left135().coord()) {
        return !grid.is_solid_or_outside(coord + direction.left90().coord());
    }

    if grid.is_solid_or_outside(coord + direction.right135().coord()) {
        return !grid.is_solid_or_outside(coord + direction.right90().coord());
    }

    false
}

fn jump_cardinal<G, C>(
    grid: &G,
    coord: Coord,
    direction: CardinalDirection,
    goal: Coord,
) -> Option<(Coord, C)>
where
    G: SolidGrid,
    C: Add<C, Output = C> + One,
{
    let neighbour_coord = coord + direction.coord();

    if grid.is_solid_or_outside(neighbour_coord) {
        return None;
    }

    if neighbour_coord == goal {
        return Some((neighbour_coord, One::one()));
    }

    if has_forced_neighbour_cardinal(grid, neighbour_coord, direction) {
        return Some((neighbour_coord, One::one()));
    }

    jump_cardinal(grid, neighbour_coord, direction, goal)
        .map(|(coord, cost): (_, C)| (coord, cost + One::one()))
}

fn jump_ordinal<G, C>(
    grid: &G,
    coord: Coord,
    direction: OrdinalDirection,
    goal: Coord,
) -> Option<(Coord, C)>
where
    G: SolidGrid,
    C: Add<C, Output = C> + One + FloatConst,
{
    let neighbour_coord = coord + direction.coord();

    if grid.is_solid_or_outside(neighbour_coord) {
        return None;
    }

    if neighbour_coord == goal {
        return Some((neighbour_coord, FloatConst::SQRT_2()));
    }

    if has_forced_neighbour_ordinal(grid, neighbour_coord, direction) {
        return Some((neighbour_coord, FloatConst::SQRT_2()));
    }

    let (card0, card1) = direction.to_cardinals();

    if jump_cardinal::<_, C>(grid, neighbour_coord, card0, goal).is_some()
        || jump_cardinal::<_, C>(grid, neighbour_coord, card1, goal).is_some()
    {
        return Some((neighbour_coord, FloatConst::SQRT_2()));
    }

    jump_ordinal(grid, neighbour_coord, direction, goal)
        .map(|(coord, cost): (_, C)| (coord, cost + FloatConst::SQRT_2()))
}

impl<C> SearchContext<C>
where
    C: Copy
        + Zero
        + One
        + Add<C, Output = C>
        + Mul<C, Output = C>
        + FloatConst
        + NumCast
        + PartialOrd<C>,
{
    fn expand_cardinal<G>(
        &mut self,
        grid: &G,
        current_coord: Coord,
        current_cost: C,
        direction: CardinalDirection,
        goal: Coord,
    ) -> Result<(), Error>
    where
        G: SolidGrid,
    {
        if let Some((successor_coord, successor_cost)) =
            jump_cardinal::<_, C>(grid, current_coord, direction, goal)
        {
            self.see_successor(
                current_cost + successor_cost,
                successor_coord,
                direction.direction(),
                octile_distance,
                goal,
            )?;
        }

        Ok(())
    }

    fn expand_ordinal<G>(
        &mut self,
        grid: &G,
        current_coord: Coord,
        current_cost: C,
        direction: OrdinalDirection,
        goal: Coord,
    ) -> Result<(), Error>
    where
        G: SolidGrid,
    {
        if let Some((successor_coord, successor_cost)) =
            jump_ordinal::<_, C>(grid, current_coord, direction, goal)
        {
            self.see_successor(
                current_cost + successor_cost,
                successor_coord,
                direction.direction(),
                octile_distance,
                goal,
            )?;
        }

        Ok(())
    }

    fn expand_general<G>(
        &mut self,
        grid: &G,
        current_coord: Coord,
        current_cost: C,
        direction: Direction,
        goal: Coord,
    ) -> Result<(), Error>
    where
        G: SolidGrid,
    {
        match direction.typ() {
            DirectionType::Cardinal(direction) => {
                self.expand_cardinal(grid, current_coord, current_cost, direction, goal)
            }
            DirectionType::Ordinal(direction) => {
                self.expand_ordinal(grid, current_coord, current_cost, direction, goal)
            }
        }
    }

    pub fn jump_point_search_octile_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<C>, Error>
    where
        G: SolidGrid,
    {
        let initial_entry = match self.init(start, |c| c == goal, grid, config, path) {
            Ok(initial_entry) => initial_entry,
            Err(result) => return result,
        };

        let goal_index = self.node_grid
            .coord_to_index(goal)
            .ok_or(Error::VisitOutsideContext)?;

        for direction in Directions {
            self.expand_general(grid, start, initial_entry.cost, direction, goal)?;
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {
            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                let node = &self.node_grid[goal_index];
                path::make_path_jump_points(&self.node_grid, goal, self.seq, path);
                return Ok(SearchMetadata {
                    num_nodes_visited,
                    cost: node.cost,
                    length: path.len(),
                });
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
                    self.expand_cardinal(grid, current_coord, current_cost, direction, goal)?;
                    let left = direction.left90();
                    if grid.is_solid_or_outside(current_coord + left.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.left45(),
                            goal,
                        )?;
                    }
                    let right = direction.right90();
                    if grid.is_solid_or_outside(current_coord + right.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.right45(),
                            goal,
                        )?;
                    }
                }
                DirectionType::Ordinal(direction) => {
                    self.expand_ordinal(grid, current_coord, current_cost, direction, goal)?;
                    let (left, right) = direction.to_cardinals();
                    self.expand_cardinal(grid, current_coord, current_cost, left, goal)?;
                    self.expand_cardinal(grid, current_coord, current_cost, right, goal)?;

                    let (check_right, check_left) = direction.opposite().to_cardinals();

                    if grid.is_solid_or_outside(current_coord + check_left.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.left90(),
                            goal,
                        )?;
                    }
                    if grid.is_solid_or_outside(current_coord + check_right.coord()) {
                        self.expand_ordinal(
                            grid,
                            current_coord,
                            current_cost,
                            direction.right90(),
                            goal,
                        )?;
                    }
                }
            }
        }

        Err(Error::NoPath)
    }
}
