//! Dijkstra's algorithm implementation for pathfinding in a grid or graph.
use bevy::{
    math::UVec3,
    platform::collections::{HashMap, HashSet},
};

use indexmap::map::Entry::{Occupied, Vacant};
use ndarray::ArrayView3;
use std::collections::BinaryHeap;

use crate::{graph::Graph, in_bounds_3d, nav::NavCell, nav_mask::NavMaskView, path::Path, FxIndexMap, SmallestCostHolder};

/// Dijkstra's algorithm for pathfinding in a grid.
///
/// ## Arguments
/// * `neighborhood` - The `Neighborhood` to use for finding neighbors.
/// * `grid` - The `Grid` to search.
/// * `start` - The starting position.
/// * `goals` - The goal positions.
/// * `only_closest_goal` - If true, only the closest goal will be returned.
/// * `size_hint` - A hint for the size of the priority queue.
/// * `blocking` - A map of blocking entities.
///
/// ## Returns
/// A `HashMap` of `UVec3` goal positions with their respective `Path`s.
pub(crate) fn dijkstra_grid<M: NavMaskView>(
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goals: &[UVec3],
    only_closest_goal: bool,
    size_hint: usize,
    mask: &M,
) -> HashMap<UVec3, Path> {
    let mut to_visit = BinaryHeap::with_capacity(size_hint / 2);
    to_visit.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });

    let mut visited: FxIndexMap<UVec3, (usize, u32)> = FxIndexMap::default();
    visited.insert(start, (usize::MAX, 0));

    let mut remaining_goals: HashSet<UVec3> = goals.iter().copied().collect();
    let mut goal_costs = HashMap::with_capacity(goals.len());

    let shape = grid.shape();
    let min = UVec3::new(0, 0, 0);
    let max = UVec3::new(shape[0] as u32, shape[1] as u32, shape[2] as u32);

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
        let neighbors = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();

            if remaining_goals.remove(current_pos) {
                goal_costs.insert(*current_pos, current_cost);
                if only_closest_goal || remaining_goals.is_empty() {
                    break;
                }
            }

            let cell = &grid[[
                current_pos.x as usize,
                current_pos.y as usize,
                current_pos.z as usize,
            ]];

            cell.neighbor_iter(*current_pos)
        };

        for neighbor in neighbors {
            if !in_bounds_3d(neighbor, min, max) {
                continue;
            }

            let neighbor_cell = &grid[[
                neighbor.x as usize,
                neighbor.y as usize,
                neighbor.z as usize,
            ]];

            if neighbor_cell.is_impassable() {
                continue;
            }

            if let Some(nav_cell) = mask.nav(grid, neighbor) {
                if nav_cell.is_impassable() {
                    continue;
                }
            }

            let new_cost = cost + neighbor_cell.cost;
            let n;

            match visited.entry(neighbor) {
                Vacant(e) => {
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_visit.push(SmallestCostHolder {
                estimated_cost: new_cost,
                cost: new_cost,
                index: n,
            });
        }
    }

    let mut goal_data = HashMap::with_capacity_and_hasher(goal_costs.len(), Default::default());

    for (&goal, &cost) in goal_costs.iter() {
        let steps = {
            let mut steps = vec![goal];
            let mut current = visited.get(&goal).unwrap().0;

            while current != usize::MAX {
                let (pos, _) = visited.get_index(current).unwrap();
                steps.push(*pos);
                current = visited.get(pos).unwrap().0;
            }
            steps.reverse();
            steps
        };

        goal_data.insert(goal, Path::new(steps, cost));
    }

    goal_data
}

/// Dijkstra's algorithm for pathfinding in a graph.
///
/// ## Arguments
/// * `graph` - The `Graph` to search.
/// * `start` - The starting position.
/// * `goals` - The goal positions.
/// * `only_closest_goal` - If true, only the closest goal will be returned.
/// * `size_hint` - A hint for the size of the priority queue.
///
/// ## Returns
/// A `HashMap` of `UVec3` goal positions with their respective `Path`s.
#[allow(dead_code)]
pub fn dijkstra_graph(
    graph: &Graph,
    start: UVec3,
    goals: &[UVec3],
    only_closest_goal: bool,
    size_hint: usize,
) -> HashMap<UVec3, Path> {
    let mut to_visit = BinaryHeap::with_capacity(size_hint / 2);
    to_visit.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });

    let mut visited: FxIndexMap<UVec3, (usize, u32)> = FxIndexMap::default();
    visited.insert(start, (usize::MAX, 0));

    let mut remaining_goals: HashSet<UVec3> = goals.iter().copied().collect();
    let mut goal_costs = HashMap::with_capacity(goals.len());

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
        let neighbors = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();

            if remaining_goals.remove(current_pos) {
                goal_costs.insert(*current_pos, current_cost);
                if only_closest_goal || remaining_goals.is_empty() {
                    break;
                }
            }

            let node = graph.node_at(*current_pos).unwrap();
            node.edges()
        };

        for &neighbor in neighbors.iter() {
            let new_cost = cost + 1;
            let n;

            match visited.entry(neighbor) {
                Vacant(e) => {
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_visit.push(SmallestCostHolder {
                estimated_cost: new_cost,
                cost: new_cost,
                index: n,
            });
        }
    }

    let mut goal_data = HashMap::with_capacity_and_hasher(goal_costs.len(), Default::default());

    for (&goal, &cost) in goal_costs.iter() {
        let steps = {
            let mut steps = vec![goal];
            let mut current = visited.get(&goal).unwrap().0;

            while current != usize::MAX {
                let (pos, _) = visited.get_index(current).unwrap();
                steps.push(*pos);
                current = visited.get(pos).unwrap().0;
            }
            steps.reverse();
            steps
        };

        goal_data.insert(goal, Path::new(steps, cost));
    }

    goal_data
}

#[cfg(test)]
mod tests {
    use crate::{
        chunk::Chunk, grid::{Grid, GridSettingsBuilder}, nav_mask::CompositeNavMask, neighbor::OrdinalNeighborhood3d, node::Node
    };

    use super::*;

    #[test]
    fn test_dijkstra_grid() {
        let grid_settings = GridSettingsBuilder::new_3d(8, 8, 8).chunk_size(4).build();
        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);
        grid.build();

        let start = UVec3::new(0, 0, 0);
        let goals = [
            UVec3::new(7, 7, 7),
            UVec3::new(0, 7, 7),
            UVec3::new(7, 0, 7),
            UVec3::new(7, 7, 0),
        ];

        let paths = dijkstra_grid(
            &grid.view(),
            start,
            &goals,
            false,
            8 * 8 * 8,
            &CompositeNavMask::new(),
        );

        assert_eq!(paths.len(), 4);
        assert_eq!(paths[&UVec3::new(7, 7, 7)].len(), 8);
    }

    #[test]
    fn test_dijkstra_graph() {
        let mut graph = Graph::new();

        let _ = graph.add_node(Node::new(
            UVec3::new(0, 0, 0),
            Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        ));
        let _ = graph.add_node(Node::new(
            UVec3::new(1, 1, 1),
            Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        ));
        let _ = graph.add_node(Node::new(
            UVec3::new(2, 2, 2),
            Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        ));

        graph.connect_node(
            UVec3::new(0, 0, 0),
            UVec3::new(1, 1, 1),
            Path::new(vec![UVec3::new(0, 0, 0), UVec3::new(1, 1, 1)], 1),
        );
        graph.connect_node(
            UVec3::new(1, 1, 1),
            UVec3::new(0, 0, 0),
            Path::new(vec![UVec3::new(1, 1, 1), UVec3::new(0, 0, 0)], 1),
        );
        graph.connect_node(
            UVec3::new(1, 1, 1),
            UVec3::new(2, 2, 2),
            Path::new(vec![UVec3::new(1, 1, 1), UVec3::new(2, 2, 2)], 1),
        );
        graph.connect_node(
            UVec3::new(2, 2, 2),
            UVec3::new(1, 1, 1),
            Path::new(vec![UVec3::new(2, 2, 2), UVec3::new(1, 1, 1)], 1),
        );

        let paths = dijkstra_graph(
            &graph,
            UVec3::new(0, 0, 0),
            &[UVec3::new(1, 1, 1), UVec3::new(2, 2, 2)],
            false,
            3,
        );

        assert_eq!(paths.len(), 2);
        assert_eq!(paths[&UVec3::new(1, 1, 1)].len(), 2);
    }
}
