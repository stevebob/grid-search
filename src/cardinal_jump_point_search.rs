use config::*;
use direction::*;
use error::*;
use grid::*;
use metadata::*;
use num_traits::{NumCast, One, Zero};
use path;
use search::*;
use std::ops::Add;

fn manhatten_distance<Cost>(a: Coord, b: Coord) -> Cost
where
    Cost: NumCast,
{
    let as_i32 = (a.x - b.x).abs() + (a.y - b.y).abs();
    NumCast::from(as_i32).expect("Failed to cast to Cost")
}

fn has_forced_neighbour<G>(grid: &G, coord: Coord, direction: CardinalDirection) -> bool
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

fn jump_to_jump_point<G>(grid: &G, coord: Coord, direction: CardinalDirection, goal: Coord) -> bool
where
    G: SolidGrid,
{
    let neighbour_coord = coord + direction.coord();

    if grid.is_solid_or_outside(neighbour_coord) {
        return false;
    }

    if neighbour_coord == goal {
        return true;
    }

    if has_forced_neighbour(grid, neighbour_coord, direction) {
        return true;
    }

    jump_to_jump_point(grid, neighbour_coord, direction, goal)
}

fn jump<G, Cost>(
    grid: &G,
    coord: Coord,
    direction: CardinalDirection,
    goal: Coord,
) -> Option<(Coord, Cost)>
where
    G: SolidGrid,
    Cost: Add<Cost, Output = Cost> + One,
{
    let neighbour_coord = coord + direction.coord();

    if grid.is_solid_or_outside(neighbour_coord) {
        return None;
    }

    if neighbour_coord == goal {
        return Some((neighbour_coord, One::one()));
    }

    if has_forced_neighbour(grid, neighbour_coord, direction) {
        return Some((neighbour_coord, One::one()));
    }

    if jump_to_jump_point(grid, neighbour_coord, direction.left90(), goal)
        || jump_to_jump_point(grid, neighbour_coord, direction.right90(), goal)
    {
        return Some((neighbour_coord, One::one()));
    }

    jump(grid, neighbour_coord, direction, goal)
        .map(|(coord, cost): (Coord, Cost)| (coord, cost + One::one()))
}

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + NumCast + Zero + One> SearchContext<Cost> {
    fn expand<G>(
        &mut self,
        grid: &G,
        current_coord: Coord,
        current_cost: Cost,
        direction: CardinalDirection,
        goal: Coord,
    ) -> Result<(), Error>
    where
        G: SolidGrid,
    {
        if let Some((successor_coord, successor_cost)) = jump(grid, current_coord, direction, goal)
        {
            self.see_successor(
                current_cost + successor_cost,
                successor_coord,
                direction.direction(),
                manhatten_distance,
                goal,
            )?;
        }

        Ok(())
    }

    pub fn jump_point_search_cardinal_manhatten_distance_heuristic<G>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: SolidGrid,
    {
        let initial_entry = match self.init(start, |c| c == goal, grid, config, path) {
            Ok(initial_entry) => initial_entry,
            Err(result) => return result,
        };

        let goal_index = self
            .node_grid
            .index_of_coord(goal)
            .ok_or(Error::VisitOutsideContext)?;

        for direction in CardinalDirections {
            self.expand(grid, start, initial_entry.cost, direction, goal)?;
        }

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {
            num_nodes_visited += 1;

            if current_entry.node_index == goal_index {
                let node = &self.node_grid[goal_index];
                path::make_path_jump_points(&self.node_grid, goal, self.seq, path);
                return Ok(SearchMetadata {
                    num_nodes_visited,
                    length: path.len(),
                    cost: node.cost,
                });
            }

            let (current_coord, current_cost, direction) = {
                let node = &mut self.node_grid[current_entry.node_index];
                if node.visited == self.seq {
                    continue;
                }
                node.visited = self.seq;
                let direction = node
                    .from_parent
                    .expect("Open set node without direction")
                    .cardinal()
                    .expect("Expected cardinal directions only");
                (node.coord, node.cost, direction)
            };

            self.expand(grid, current_coord, current_cost, direction, goal)?;
            self.expand(grid, current_coord, current_cost, direction.left90(), goal)?;
            self.expand(grid, current_coord, current_cost, direction.right90(), goal)?;
        }

        Err(Error::NoPath)
    }
}
