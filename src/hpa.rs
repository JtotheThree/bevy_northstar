use std::collections::BinaryHeap;
use indexmap::map::Entry::{Occupied, Vacant};

use bevy::{ecs::entity::Entity, math::UVec3, platform::collections::HashMap};

use crate::{astar::astar_grid, grid::Grid, nav_mask::NavMaskData, neighbor::Neighborhood, path::Path, FxIndexMap, SmallestCostHolder};

pub(crate) fn hpa<N: Neighborhood>(
    grid: &Grid<N>,
    start: UVec3,
    goal: UVec3,
    size_hint: usize,
    partial: bool,
    blocking: &HashMap<UVec3, Entity>,
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

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
        let (neighbors, current_pos) = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();

            if *current_pos == goal {
                return None;
            }

            if cost > current_cost {
                continue;
            }

            let node = grid.graph().node_at(*current_pos).unwrap();
            let neighbors = node.edges();

            (neighbors, *current_pos)
        };

        for neighbor in neighbors.iter() {
            let neighbor_node = grid.graph().node_at(*neighbor).unwrap();
            if neighbor_node.edges().is_empty() {
                continue;
            }

            let neighbor_cell = grid.navcell(*neighbor);

            let mask_cell = mask.get(neighbor_cell.clone(), *neighbor);

            if mask_cell.is_impassable() {
                continue;
            }

            let neighbor_chunk = grid.chunk_at_position(*neighbor).unwrap();
            if mask.chunk_in_mask(neighbor_chunk.index()) {
                // Get a view of just the chunk from the grid
                let chunk_ref = grid.chunk_at_position(*neighbor).unwrap();
                let chunk = grid.chunk_view(chunk_ref);
                let path = astar_grid(grid.neighborhood(), &chunk, start, goal, size_hint, partial, blocking, mask);

                if path.is_none() {
                    continue;
                }
            }

            let new_cost = cost + grid.graph().edge_cost(current_pos, *neighbor).unwrap();

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

    None
}