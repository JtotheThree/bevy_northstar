//! A* algorithms used by the crate.
use bevy::{log, math::UVec3, platform::collections::HashMap, prelude::Entity};
use indexmap::map::Entry::{Occupied, Vacant};
use ndarray::ArrayView3;
use std::collections::BinaryHeap;

use crate::{
    in_bounds_3d, nav::NavCell, neighbor::Neighborhood, path::Path, raycast::line_of_sight,
    FxIndexMap, SmallestCostHolder,
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
    blocking: &HashMap<UVec3, Entity>,
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
        let mut index = index;
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

            if blocking.contains_key(&neighbor) {
                continue;
            }

            let parent_index = visited.get_index(index).unwrap().1 .0;

            if parent_index != usize::MAX {
                let parent = visited.get_index(parent_index).unwrap().0;

                if line_of_sight(grid, neighbor, *parent) {
                    index = parent_index;
                }
            };

            let new_cost = cost + neighbor_cell.cost;
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
