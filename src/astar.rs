//! A* algorithms used by the crate.
use bevy::{ecs::entity::Entity, log, math::UVec3, platform::collections::HashMap};
use indexmap::map::Entry::{Occupied, Vacant};
use ndarray::ArrayView3;
use std::collections::BinaryHeap;

use crate::{
    graph::Graph, in_bounds_3d, nav::NavCell, nav_mask::NavMaskData, neighbor::Neighborhood,
    path::Path, size_hint_graph, size_hint_grid, FxIndexMap, SmallestCostHolder,
};

/// A* search algorithm for a [`crate::grid::Grid`] of [`crate::nav::NavCell`]s.
///
/// # Arguments
/// * `neighborhood` - Reference to the [`Neighborhood`] to use.
/// * `grid` - A reference to a 3D array representing the grid, as an [`ndarray::ArrayView3`] of [`NavCell`].
/// * `start` - The start position as [`bevy::math::UVec3`].
/// * `goal` - The goal position as [`bevy::math::UVec3`].
/// * `size_hint` - A hint for the size of the binary heap.
/// * `partial` - If `true`, the algorithm will return the closest node if the goal is not reachable.
/// * `blocking` - Pass [`crate::plugin::BlockingMap`] or a new `HashMap<UVec3, Entity>` to indicate which positions are blocked by entities.
///
/// # Returns
/// * [`Option<Path>`] - An optional path object. If a path is found, it returns `Some(Path)`, otherwise it returns `None`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn astar_grid<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    partial: bool,
    blocking: &HashMap<UVec3, Entity>,
    mask: &NavMaskData,
) -> Option<Path> {
    let size_hint = size_hint_grid(neighborhood, grid.shape(), start, goal);

    let masked = mask.layers.len() > 0;

    let mut to_visit = BinaryHeap::with_capacity(size_hint);
    to_visit.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });

    let mut visited: FxIndexMap<UVec3, (usize, u32)> = FxIndexMap::default();
    visited.insert(start, (usize::MAX, 0));

    let mut closest_node = start;
    let mut closest_distance = neighborhood.heuristic(start, goal);

    let shape = grid.shape();
    let min = UVec3::new(0, 0, 0);
    let max = UVec3::new(shape[0] as u32, shape[1] as u32, shape[2] as u32);

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
        let neighbors = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();
            let current_distance = neighborhood.heuristic(*current_pos, goal);

            // Update the closest node if this node is closer
            if current_distance < closest_distance {
                closest_node = *current_pos;
                closest_distance = current_distance;
            }

            if *current_pos == goal {
                let mut current = index;
                let mut steps = vec![];

                while current != usize::MAX {
                    let (pos, _) = visited.get_index(current).unwrap();
                    steps.push(*pos);
                    current = visited.get(pos).unwrap().0;
                }

                steps.reverse();
                return Some(Path::new(steps, current_cost));
            }

            if cost > current_cost {
                continue;
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

            if blocking.contains_key(&neighbor) {
                continue; // Skip blocked positions
            }

            let neighbor_cell = &grid[[
                neighbor.x as usize,
                neighbor.y as usize,
                neighbor.z as usize,
            ]];

            let (cost_value, is_impassable) = if masked {
                if let Some(masked_cell) = mask.get(neighbor_cell.clone(), neighbor) {
                    (masked_cell.cost, masked_cell.is_impassable())
                } else {
                    (neighbor_cell.cost, neighbor_cell.is_impassable())
                }
            } else {
                (neighbor_cell.cost, neighbor_cell.is_impassable())
            };

            if is_impassable {
                continue;
            }

            let new_cost = cost + cost_value;

            let h;
            let n;
            match visited.entry(neighbor) {
                Vacant(e) => {
                    h = neighborhood.heuristic(neighbor, goal);
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = neighborhood.heuristic(neighbor, goal);
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_visit.push(SmallestCostHolder {
                estimated_cost: h,
                cost: new_cost,
                index: n,
            });
        }
    }

    if partial {
        // If the goal is not reached, return the path to the closest node, but if the closest node is the start return None

        if closest_node == start {
            return None;
        }

        // If the goal is not reached, return the path to the closest node
        let mut current = visited.get_index_of(&closest_node).unwrap();
        let mut steps = vec![];

        while current != usize::MAX {
            let (pos, _) = visited.get_index(current).unwrap();
            steps.push(*pos);
            current = visited.get(pos).unwrap().0;
        }

        if steps.is_empty() {
            log::error!("Steps is empty, so there's actually no path?");
            return None;
        }

        steps.reverse();
        Some(Path::new(steps, visited[&closest_node].1))
    } else {
        None
    }
}

/// A* search algorithm for a graph of nodes with connected edges.
/// This function is primarily to be used for the crate, but can be used directly if desired.
///
/// # Arguments
/// * `neighborhood` - Reference to the [`Neighborhood`] to use.
/// * `graph` - A reference to the [`Graph`] object.
/// * `start` - The starting position as a [`bevy::math::UVec3`].
/// * `goal` - The goal position as a [`bevy::math::UVec3`].
/// * `size_hint` - A hint for the size of the binary heap.
///
/// # Returns
/// * [`Option<Path>`] - An optional path object. If a path is found, it returns `Some(Path)`, otherwise it returns `None`.
#[allow(dead_code)]
pub(crate) fn astar_graph<N: Neighborhood>(
    neighborhood: &N,
    graph: &Graph,
    start: UVec3,
    goal: UVec3,
) -> Option<Path> {
    let size_hint = size_hint_graph(neighborhood, graph, start, goal);

    let mut to_visit = BinaryHeap::with_capacity(size_hint);
    to_visit.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });

    let mut visited: FxIndexMap<UVec3, (usize, u32)> = FxIndexMap::default();
    visited.insert(start, (usize::MAX, 0));

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
        let (neighbors, current_pos) = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();
            if *current_pos == goal {
                let mut current = index;
                let mut steps = vec![];

                while current != usize::MAX {
                    let (pos, _) = visited.get_index(current).unwrap();
                    steps.push(*pos);
                    current = visited.get(pos).unwrap().0;
                }

                steps.reverse();
                return Some(Path::new(steps, current_cost));
            }

            if cost > current_cost {
                continue;
            }

            let node = graph.node_at(*current_pos).unwrap();
            let neighbors = node.edges();

            (neighbors, *current_pos)
        };

        for neighbor in neighbors.iter() {
            let neighbor_node = graph.node_at(*neighbor).unwrap();
            if neighbor_node.edges.is_empty() {
                continue;
            }

            let new_cost = cost + graph.edge_cost(current_pos, *neighbor).unwrap();

            let h;
            let n;
            match visited.entry(neighbor_node.pos) {
                Vacant(e) => {
                    h = neighborhood.heuristic(neighbor_node.pos, goal); // This might be a mistake?
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = neighborhood.heuristic(neighbor_node.pos, goal); // This might be a mistake?
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_visit.push(SmallestCostHolder {
                estimated_cost: h,
                cost: new_cost,
                index: n,
            });
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chunk::Chunk;
    use crate::grid::{Grid, GridSettingsBuilder};
    use crate::nav::Nav;
    use crate::nav_mask::NavMaskData;
    use crate::neighbor::OrdinalNeighborhood3d;
    use crate::node::Node;

    #[test]
    fn test_astar_grid() {
        let grid_settings = GridSettingsBuilder::new_3d(3, 3, 3).chunk_size(3).build();
        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);

        grid.build();

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(2, 2, 2);

        let path = astar_grid(
            &OrdinalNeighborhood3d {
                filters: Vec::new(),
            },
            &grid.view(),
            start,
            goal,
            false,
            &HashMap::new(),
            &NavMaskData::new(),
        )
        .unwrap();

        assert_eq!(path.cost(), 2);
        assert_eq!(path.len(), 3);
        // Ensure first position is the start position
        assert_eq!(path.path()[0], start);
        // Ensure last position is the goal position
        assert_eq!(path.path()[2], goal);
    }

    #[test]
    fn test_astar_grid_with_wall() {
        let grid_settings = GridSettingsBuilder::new_3d(3, 3, 3).chunk_size(3).build();

        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);

        grid.set_nav(UVec3::new(1, 1, 1), Nav::Impassable);

        grid.build();

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(2, 2, 2);

        let path = astar_grid(
            &OrdinalNeighborhood3d {
                filters: Vec::new(),
            },
            &grid.view(),
            start,
            goal,
            false,
            &HashMap::new(),
            &NavMaskData::new(),
        )
        .unwrap();

        assert_eq!(path.cost(), 3);
        assert_eq!(path.len(), 4);
        // Ensure first position is the start position
        assert_eq!(path.path()[0], start);
        // Ensure last position is the goal position
        assert_eq!(path.path()[3], goal);
        assert!(!path.is_position_in_path(UVec3::new(1, 1, 1)));
    }

    #[test]
    fn test_astar_grid_with_ramp() {
        let grid_settings = GridSettingsBuilder::new_3d(3, 3, 3)
            .chunk_size(3)
            .default_impassable()
            .build();
        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);

        // Fill the bottoom left hand layer with passable cells
        for x in 0..1 {
            for y in 0..3 {
                grid.set_nav(UVec3::new(x, y, 0), Nav::Passable(1));
            }
        }

        for x in 2..3 {
            for y in 0..3 {
                grid.set_nav(UVec3::new(x, y, 2), Nav::Passable(1));
            }
        }

        // Add a single ramp to transition from the bottom layer to the top layer
        grid.set_nav(
            UVec3::new(1, 1, 0),
            Nav::Portal(crate::nav::Portal {
                target: UVec3::new(1, 1, 2),
                cost: 1,
                one_way: false,
            }),
        );
        // Make sure the ramp destination is passable
        grid.set_nav(UVec3::new(1, 1, 2), Nav::Passable(1));

        grid.build();

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(2, 2, 2);

        let path = astar_grid(
            &OrdinalNeighborhood3d {
                filters: Vec::new(),
            },
            &grid.view(),
            start,
            goal,
            false,
            &HashMap::new(),
            &NavMaskData::new(),
        )
        .unwrap();

        assert!(!path.is_empty());
        // Ensure the path is using the ramp
        assert!(
            path.path.contains(&UVec3::new(1, 1, 0)),
            "Ramp origin missing"
        );
        assert!(
            path.path.contains(&UVec3::new(1, 1, 2)),
            "Ramp target missing"
        );
    }

    #[test]
    fn test_astar_grid_8x8() {
        let grid_settings = crate::grid::GridSettingsBuilder::new_3d(8, 8, 8)
            .chunk_size(4)
            .build();
        let mut grid = crate::grid::Grid::<OrdinalNeighborhood3d>::new(&grid_settings);
        grid.build();

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(7, 7, 7);

        let path = astar_grid(
            &OrdinalNeighborhood3d {
                filters: Vec::new(),
            },
            &grid.view(),
            start,
            goal,
            false,
            &HashMap::new(),
            &NavMaskData::new(),
        )
        .unwrap();

        assert_eq!(path.len(), 8);
        // Ensure first position is the start position
        assert_eq!(path.path()[0], start);
        // Ensure last position is the goal position
        assert_eq!(path.path()[7], goal);
    }

    #[test]
    fn test_astar_graph() {
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

        let path = astar_graph(
            &OrdinalNeighborhood3d {
                filters: Vec::new(),
            },
            &graph,
            UVec3::new(0, 0, 0),
            UVec3::new(2, 2, 2),
        )
        .unwrap();

        assert_eq!(path.cost(), 2);
        assert_eq!(path.len(), 3);
        // Ensure the first position is the start position
        assert_eq!(path.path()[0], UVec3::new(0, 0, 0));
        // Ensure the last position is the goal position
        assert_eq!(path.path()[2], UVec3::new(2, 2, 2));
    }
}
