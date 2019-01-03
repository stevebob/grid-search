use config::*;
use direction::*;
use error::*;
use grid::*;
use metadata::*;
use num_traits::Zero;
use path;
use search::*;
use std::ops::Add;

impl<Cost: Copy + Add<Cost> + PartialOrd<Cost> + Zero> SearchContext<Cost> {
    pub fn dijkstra<G, V, D>(
        &mut self,
        grid: &G,
        start: Coord,
        goal: Coord,
        directions: D,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
    {
        self.search_general(
            grid,
            start,
            goal,
            directions,
            |_, _| Zero::zero(),
            config,
            path,
        )
    }

    pub fn dijkstra_predicate<G, V, D, F>(
        &mut self,
        grid: &G,
        start: Coord,
        predicate: F,
        directions: D,
        config: SearchConfig,
        path: &mut Vec<Direction>,
    ) -> Result<SearchMetadata<Cost>, Error>
    where
        G: CostGrid<Cost = Cost>,
        V: Into<Direction>,
        D: Copy + IntoIterator<Item = V>,
        F: Fn(Coord) -> bool,
    {
        let initial_entry = match self.init(start, &predicate, grid, config, path) {
            Ok(initial_entry) => initial_entry,
            Err(result) => return result,
        };

        self.priority_queue.push(initial_entry);

        let mut num_nodes_visited = 0;

        while let Some(current_entry) = self.priority_queue.pop() {
            num_nodes_visited += 1;

            let (current_coord, current_cost) = {
                let node = &mut self.node_grid[current_entry.node_index];

                if node.visited == self.seq {
                    continue;
                }
                node.visited = self.seq;
                (node.coord, node.cost)
            };

            if predicate(current_coord) {
                path::make_path_all_adjacent(&self.node_grid, current_entry.node_index, path);
                return Ok(SearchMetadata {
                    num_nodes_visited,
                    cost: current_cost,
                    length: path.len(),
                });
            }

            for d in directions {
                let direction = d.into();
                let neighbour_coord = current_coord + direction.coord();

                let neighbour_cost =
                    if let Some(CostCell::Cost(cost)) = grid.cost(neighbour_coord, direction) {
                        cost
                    } else {
                        continue;
                    };

                let index = self
                    .node_grid
                    .index_of_coord(neighbour_coord)
                    .ok_or(Error::VisitOutsideContext)?;

                let cost = current_cost + neighbour_cost;
                let node = &mut self.node_grid[index];

                if node.seen != self.seq || node.cost > cost {
                    node.from_parent = Some(direction);
                    node.seen = self.seq;
                    node.cost = cost;

                    let entry = PriorityEntry::new(index, cost);
                    self.priority_queue.push(entry);
                }
            }
        }

        Err(Error::NoPath)
    }
}
