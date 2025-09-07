use indexmap::map::Entry::{Occupied, Vacant};
use std::collections::BinaryHeap;

use bevy::{ecs::entity::Entity, log, math::UVec3, platform::collections::HashMap};

use crate::{
    are_adjacent, astar::astar_grid, grid::Grid, nav_mask::NavMaskData, neighbor::Neighborhood,
    path::Path, size_hint_graph, FxIndexMap, NavRegion, SearchLimits, SmallestCostHolder,
};

pub(crate) fn hpa<N: Neighborhood>(
    grid: &Grid<N>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &mut NavMaskData,
    limits: SearchLimits,
) -> Option<Path> {
    let bounded = limits.boundary.is_some();
    let boundary = limits.boundary.unwrap_or(NavRegion {
        min: UVec3::ZERO,
        max: UVec3::ZERO,
    });

    if bounded && !boundary.contains(start) {
        return None;
    }

    let distance_limited = limits.distance.is_some();
    let max_distance = limits.distance.unwrap_or(u32::MAX);

    let size_hint = size_hint_graph(&grid.neighborhood, grid.graph(), start, goal);

    let mut to_visit = BinaryHeap::with_capacity(size_hint);
    to_visit.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });

    let mut visited: FxIndexMap<UVec3, (usize, u32)> = FxIndexMap::default();
    visited.insert(start, (usize::MAX, 0));

    // Performance optimization if the navigation mask layer count is 0
    let masked = mask.layer_count() > 0;

    let mut closest_node = start;
    let mut closest_distance = grid.neighborhood.heuristic(start, goal);

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
        let (neighbors, current_pos) = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();

            // Update the closest node if this node is closer
            let current_distance = grid.neighborhood.heuristic(*current_pos, goal);
            if current_distance < closest_distance {
                closest_distance = current_distance;
                closest_node = *current_pos;
            }

            if *current_pos == goal {
                let mut current = index;
                let mut node_path = vec![];

                // First, collect the high-level node path
                while current != usize::MAX {
                    let (pos, _) = visited.get_index(current).unwrap();
                    node_path.push(*pos);
                    current = visited.get(pos).unwrap().0;
                }

                node_path.reverse();

                // Now rebuild the full path from cached paths
                let full_path = rebuild_full_path(grid, &node_path, mask);

                return Some(Path::new(full_path, current_cost));
            }

            if cost > current_cost {
                continue;
            }

            let node = grid.graph().node_at(*current_pos).unwrap();
            let neighbors = node.edges();

            (neighbors, *current_pos)
        };

        for neighbor in neighbors.iter() {
            if bounded && !boundary.contains(*neighbor) {
                continue;
            }

            if distance_limited && grid.neighborhood.heuristic(start, *neighbor) > max_distance {
                continue;
            }

            let neighbor_node = grid.graph().node_at(*neighbor).unwrap();
            if neighbor_node.edges().is_empty() {
                continue;
            }

            let neighbor_cell = grid.navcell(*neighbor);
            let new_cost;

            if !masked {
                // Not masked - use default graph edge cost
                if neighbor_cell.is_impassable() {
                    continue;
                }
                new_cost = cost + grid.graph().edge_cost(current_pos, *neighbor).unwrap();
            } else {
                // Check if the neighbor node is in a mask affected chunk
                let neighbor_chunk = grid.chunk_at_position(*neighbor).unwrap();

                if mask.chunk_in_mask(neighbor_chunk.index()) {
                    let (cell_cost, is_impassable) =
                        if let Some(masked_cell) = mask.get(neighbor_cell.clone(), *neighbor) {
                            (masked_cell.cost, masked_cell.is_impassable())
                        } else {
                            (neighbor_cell.cost, neighbor_cell.is_impassable()) // No clone here!
                        };

                    if is_impassable {
                        continue;
                    }

                    // Check the cache first
                    if let Some(cached_path) = mask.get_cached_path(current_pos, *neighbor) {
                        new_cost = cost + cached_path.cost();
                    } else {
                        // Calculate new path cost
                        let path_cost = if are_adjacent(
                            current_pos,
                            *neighbor,
                            grid.neighborhood().is_ordinal(),
                        ) {
                            // Adjacent case
                            let path = Path::new(vec![current_pos, *neighbor], cell_cost);
                            mask.add_cached_path(current_pos, *neighbor, path);
                            cell_cost
                        } else {
                            // Distant case - need pathfinding within chunk
                            match find_mask_path(
                                grid,
                                current_pos,
                                *neighbor,
                                blocking,
                                mask,
                                limits,
                            ) {
                                Some(path_cost) => path_cost,
                                None => continue,
                            }
                        };

                        new_cost = cost + path_cost;
                    }
                } else {
                    // Not masked - use default graph edge cost
                    if neighbor_cell.is_impassable() {
                        continue;
                    }
                    new_cost = cost + grid.graph().edge_cost(current_pos, *neighbor).unwrap();
                }
            }

            let h;
            let n;
            match visited.entry(neighbor_node.pos) {
                Vacant(e) => {
                    h = grid.neighborhood.heuristic(neighbor_node.pos, goal); // This might be a mistake?
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = grid.neighborhood.heuristic(neighbor_node.pos, goal); // This might be a mistake?
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

    if limits.partial {
        if closest_node == start {
            return None;
        }

        // If the goal is not reached, return the path to the closest node
        let mut current = visited.get_index_of(&closest_node).unwrap();
        let mut node_path = vec![];

        // First, collect the high-level node path
        while current != usize::MAX {
            let (pos, _) = visited.get_index(current).unwrap();
            node_path.push(*pos);
            current = visited.get(pos).unwrap().0;
        }

        node_path.reverse();

        // Now rebuild the full path from cached paths
        let full_path = rebuild_full_path(grid, &node_path, mask);

        let closest_cost = visited.get(&closest_node).unwrap().1;
        let mut path = Path::new(full_path, closest_cost);
        path.set_partial(true);
        return Some(path);
    }

    None
}

fn rebuild_full_path<N: Neighborhood>(
    grid: &Grid<N>,
    node_path: &[UVec3],
    mask: &NavMaskData,
) -> Vec<UVec3> {
    if node_path.len() <= 1 {
        return node_path.to_vec();
    }

    let mut full_path = vec![node_path[0]]; // Start with the first node

    for window in node_path.windows(2) {
        let from = window[0];
        let to = window[1];

        // First check if mask has a cached path
        if let Some(cached_path) = mask.get_cached_path(from, to) {
            // Add the cached path (skip the first element to avoid duplicates)
            full_path.extend(cached_path.path().iter().skip(1));
        } else if let Some(edge_path) = grid.graph().edge_path(from, to) {
            // Use the graph's cached edge path
            full_path.extend(edge_path.path().iter().skip(1));
        } else if are_adjacent(from, to, grid.neighborhood().is_ordinal()) {
            // Adjacent nodes, just add the destination
            full_path.push(to);
        } else {
            // Fallback: direct connection (shouldn't happen in a well-formed graph)
            log::warn!(
                "No cached path found between {:?} and {:?}, using direct connection",
                from,
                to
            );
            full_path.push(to);
        }
    }

    full_path
}

fn find_mask_path<N: Neighborhood>(
    grid: &Grid<N>,
    current_pos: UVec3,
    neighbor_pos: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &mut NavMaskData,
    limits: SearchLimits,
) -> Option<u32> {
    let chunk_ref = grid.chunk_at_position(neighbor_pos)?;
    let chunk = grid.chunk_view(chunk_ref);

    let chunk_current_pos = chunk_ref.global_to_chunk(&current_pos)?;
    let chunk_neighbor = chunk_ref.global_to_chunk(&neighbor_pos)?;

    let min = chunk_ref.min().as_ivec3();
    let mask_local = mask.translate_by(-min);

    let mut path = astar_grid(
        grid.neighborhood(),
        &chunk,
        chunk_current_pos,
        chunk_neighbor,
        blocking,
        &mask_local,
        limits,
    )?;

    // Convert path positions back to global
    for pos in path.as_mut_slices().iter_mut() {
        *pos = chunk_ref.chunk_to_global(pos);
    }

    let path_cost = path.cost();
    mask.add_cached_path(current_pos, neighbor_pos, path);
    Some(path_cost)
}

#[cfg(test)]
mod tests {
    use crate::{
        grid::GridSettingsBuilder,
        nav::Nav,
        pathfind::PathfindArgs,
        prelude::{NavCellMask, NavMaskLayer, NavRegion, OrdinalNeighborhood3d},
    };

    use super::*;

    #[test]
    fn test_hpa() {
        let grid_settings = GridSettingsBuilder::new_2d(16, 16).chunk_size(4).build();
        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);
        grid.set_nav(UVec3::new(1, 1, 0), Nav::Impassable);
        grid.build();

        let start = UVec3::new(2, 4, 0);
        let goal = UVec3::new(14, 12, 0);

        let path = hpa(
            &grid,
            start,
            goal,
            &HashMap::new(),
            &mut NavMaskData::new(),
            SearchLimits::default(),
        )
        .unwrap();

        assert_eq!(path.cost(), 21);
        assert_eq!(path.len(), 17);
        // Ensure first position is the start position
        assert_eq!(path.path()[0], start);
        // Ensure last position is the goal position
        assert_eq!(path.path()[16], goal);

        // Ensure that all positions in the path are adjacent
        for window in path.path().windows(2) {
            assert!(are_adjacent(
                window[0],
                window[1],
                grid.neighborhood().is_ordinal()
            ));
        }
    }

    #[test]
    fn test_hpa_with_mask() {
        let grid_settings = GridSettingsBuilder::new_2d(16, 16).chunk_size(4).build();
        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);
        grid.set_nav(UVec3::new(1, 1, 0), Nav::Impassable);
        grid.build();

        let start = UVec3::new(2, 4, 0);
        let goal = UVec3::new(14, 12, 0);

        // Create a mask that blocks the middle area

        let layer = NavMaskLayer::new();
        layer
            .insert_region_fill(
                &grid,
                NavRegion::new(UVec3::new(5, 5, 0), UVec3::new(10, 10, 0)),
                NavCellMask::ModifyCost(5000),
            )
            .ok();

        let mut mask = NavMaskData::new();
        mask.add_layer(layer);

        let path = hpa(
            &grid,
            start,
            goal,
            &HashMap::new(),
            &mut mask,
            SearchLimits::default(),
        )
        .unwrap();

        println!("Path: {:?}", path.path());

        // Assert that 8, 6, 0 is not in the path
        assert!(!path.path().contains(&UVec3::new(8, 6, 0)));

        // Ensure that all positions in the path are adjacent
        for window in path.path().windows(2) {
            assert!(are_adjacent(
                window[0],
                window[1],
                grid.neighborhood().is_ordinal()
            ));
        }
    }

    #[test]
    fn test_hpa_search_limits() {
        let grid_settings = crate::grid::GridSettingsBuilder::new_3d(8, 8, 8)
            .chunk_size(4)
            .build();
        let mut grid = crate::grid::Grid::<OrdinalNeighborhood3d>::new(&grid_settings);
        grid.build();

        // Test max_distance

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(7, 7, 7);

        let path = grid.pathfind(&mut PathfindArgs::new(start, goal).max_distance(5));

        assert!(path.is_none());

        // Test Boundary
        let path = grid.pathfind(
            &mut PathfindArgs::new(start, goal)
                .search_region(NavRegion::new(UVec3::new(0, 0, 0), UVec3::new(4, 4, 4))),
        );
        assert!(path.is_none());
    }
}
