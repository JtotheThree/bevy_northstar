//! A* algorithms used by the crate.
use bevy::{log, math::UVec3};
use indexmap::map::Entry::{Occupied, Vacant};
use ndarray::ArrayView3;
use std::collections::BinaryHeap;

use crate::{
    in_bounds_3d, nav::NavCell, nav_mask::NavMaskData, neighbor::Neighborhood, path::Path,
    raycast::has_line_of_sight, FxIndexMap, SmallestCostHolder,
};

/// θ* search algorithm for a [`crate::grid::Grid`] of [`crate::nav::NavCell`]s.
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
pub(crate) fn thetastar_grid<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    size_hint: usize,
    partial: bool,
    mask: &NavMaskData,
) -> Option<Path> {
    let mut to_visit = BinaryHeap::with_capacity(size_hint / 2);
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

            let neighbor_cell = &grid[[
                neighbor.x as usize,
                neighbor.y as usize,
                neighbor.z as usize,
            ]];

            if neighbor_cell.is_impassable() {
                continue;
            }

            if mask.get(neighbor_cell.clone(), neighbor).is_impassable() {
                continue;
            }

            let current_parent_index = visited.get_index(index).unwrap().1 .0;
            let mut chosen_parent_index = index; // default to current node

            if current_parent_index != usize::MAX {
                let parent_pos = *visited.get_index(current_parent_index).unwrap().0;

                // LOS check: parent -> neighbor
                if has_line_of_sight(grid, parent_pos, neighbor, neighborhood.is_ordinal()) {
                    chosen_parent_index = current_parent_index;
                } else if grid[[
                    neighbor.x as usize,
                    neighbor.y as usize,
                    neighbor.z as usize,
                ]]
                .is_portal()
                {
                    chosen_parent_index = current_parent_index;
                }
            }

            let new_cost = cost + neighbor_cell.cost;
            let h;
            let n;
            match visited.entry(neighbor) {
                Vacant(e) => {
                    h = neighborhood.heuristic(neighbor, goal);
                    n = e.index();
                    e.insert((chosen_parent_index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = neighborhood.heuristic(neighbor, goal);
                        n = e.index();
                        e.insert((chosen_parent_index, new_cost));
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

#[cfg(test)]
mod tests {
    use super::*;

    use crate::grid::{Grid, GridSettingsBuilder};
    use crate::nav::Nav;
    use crate::neighbor::OrdinalNeighborhood3d;
    use crate::prelude::OrdinalNeighborhood;

    #[test]
    fn test_thetastar_grid() {
        let grid_settings = GridSettingsBuilder::new_2d(9, 9).chunk_size(3).build();
        let mut grid = Grid::<OrdinalNeighborhood>::new(&grid_settings);

        grid.set_nav(UVec3::new(2, 1, 0), Nav::Impassable);

        grid.build();

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(3, 3, 0);

        let path = thetastar_grid(
            &OrdinalNeighborhood3d {
                filters: Vec::new(),
            },
            &grid.view(),
            start,
            goal,
            64,
            false,
            &NavMaskData::new(),
        )
        .unwrap();

        assert_eq!(path.cost(), 3);
        assert_eq!(path.len(), 2);
        // Ensure first position is the start position
        assert_eq!(path.path()[0], start);
        // Ensure last position is the goal position
        assert_eq!(path.path()[1], goal);
    }
}
