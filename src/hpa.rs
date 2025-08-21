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

            let mut new_cost = cost + grid.graph().edge_cost(current_pos, *neighbor).unwrap();

            let neighbor_chunk = grid.chunk_at_position(*neighbor).unwrap();
            if mask.chunk_in_mask(neighbor_chunk.index()) {
                // Get a view of just the chunk from the grid
                let chunk_ref = grid.chunk_at_position(*neighbor).unwrap();
                let chunk = grid.chunk_view(chunk_ref);
                let path = astar_grid(grid.neighborhood(),  &grid.view(), current_pos, *neighbor, size_hint, partial, blocking, mask);

                if path.is_none() {
                    continue;
                }

                new_cost = cost + path.unwrap().cost();
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

    None
}

#[cfg(test)]
mod tests {
    use crate::{grid::GridSettingsBuilder, nav::Nav, prelude::{NavCellMask, NavMaskLayer, OrdinalNeighborhood3d, Region3d}};

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
            64,
            false,
            &HashMap::new(),
            &NavMaskData::new(),
        )
        .unwrap();


        assert_eq!(path.cost(), 21);
        assert_eq!(path.len(), 11);
        // Ensure first position is the start position
        assert_eq!(path.path()[0], start);
        // Ensure last position is the goal position
        assert_eq!(path.path()[10], goal);
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
        layer.insert_region(
            &grid,
            Region3d::new(UVec3::new(5, 5, 0), UVec3::new(10, 10, 0)),
            NavCellMask::ModifyCost(5000),
        ).ok();

        let mut mask = NavMaskData::new();
        mask.add_layer(layer);

        let path = hpa(
            &grid,
            start,
            goal,
            64,
            false,
            &HashMap::new(),
            &mask,
        )
        .unwrap();

        println!("Path: {:?}", path.path());

        // Assert that 8, 6, 0 is not in the path
        assert!(!path.path().contains(&UVec3::new(8, 6, 0)));

    }
}