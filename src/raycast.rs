//! Raycasting and pathfinding utilities for 2D/3D grids.
use bevy::math::{IVec3, UVec3};
use ndarray::ArrayView3;

use crate::nav::NavCell;

/// Yielding function for tracing a line in a grid.
pub(crate) fn trace_line<F>(
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    ordinal: bool,
    filtered: bool,
    aliasing: bool,
    mut yield_each_step: F,
) -> bool
where
    F: FnMut(UVec3) -> bool,
{
    let mut current = start;
    let shape = grid.shape();
    let bounds = UVec3::new(shape[0] as u32, shape[1] as u32, shape[2] as u32);

    let delta = goal.as_ivec3() - start.as_ivec3();
    let step = delta.map(i32::signum);
    let abs = delta.map(i32::abs);

    let mut err_xy = abs.x - abs.y;
    let mut err_xz = abs.x - abs.z;

    let mut last_dir: Option<IVec3> = None;
    let mut last_corner = start;

    while current != goal {
        if current.x >= bounds.x || current.y >= bounds.y || current.z >= bounds.z {
            return false;
        }

        if !yield_each_step(current) {
            return false;
        }

        let next = if ordinal {
            let dx2 = 2 * err_xy;
            let dz2 = 2 * err_xz;

            let mut next = current;

            if dx2 >= -abs.y && dz2 >= -abs.z {
                err_xy -= abs.y;
                err_xz -= abs.z;
                next.x = next.x.saturating_add_signed(step.x);
            }
            if dx2 < abs.x {
                err_xy += abs.x;
                next.y = next.y.saturating_add_signed(step.y);
            }
            if bounds.z > 1 && dz2 < abs.x {
                err_xz += abs.x;
                next.z = next.z.saturating_add_signed(step.z);
            }

            next
        } else {
            let dx2 = 2 * err_xy;
            let dz2 = 2 * err_xz;

            if dx2 >= -abs.y && dz2 >= -abs.z {
                err_xy -= abs.y;
                err_xz -= abs.z;
                UVec3::new(
                    current.x.saturating_add_signed(step.x),
                    current.y,
                    current.z,
                )
            } else if dx2 < abs.x {
                err_xy += abs.x;
                UVec3::new(
                    current.x,
                    current.y.saturating_add_signed(step.y),
                    current.z,
                )
            } else if bounds.z > 1 && dz2 < abs.x {
                err_xz += abs.x;
                UVec3::new(
                    current.x,
                    current.y,
                    current.z.saturating_add_signed(step.z),
                )
            } else {
                return false;
            }
        };

        if !aliasing {
            let dir = (next.as_ivec3() - current.as_ivec3()).signum();
            if let Some(last) = last_dir {
                if dir != last {
                    let diff = (current.as_ivec3() - last_corner.as_ivec3()).abs();
                    if diff.x + diff.y + diff.z < 2 {
                        return false;
                    }
                    last_corner = current;
                }
            }
            last_dir = Some(dir);
        }

        if filtered
            && !grid[[current.x as usize, current.y as usize, current.z as usize]]
                .neighbor_iter(current)
                .any(|n| n == next)
        {
            return false;
        }

        current = next;
    }

    yield_each_step(goal)
}

/// Checks if there is line of sight between two points in the grid ignoring any neighborhood filters.
/// Use this to check if two points can see each other without any obstacles in between.
///
/// # Arguments
/// * `grid` - The [`crate::grid::Grid`] to check the line of sight in.
/// * `start` - The starting position in the grid.
/// * `goal` - The goal position in the grid.
/// * `ordinal` - Whether to use ordinal or cardinal movement.
///
/// # Returns
/// `true` if there is a line of sight between the two points, `false` otherwise.
pub fn has_line_of_sight(
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    ordinal: bool,
) -> bool {
    trace_line(grid, start, goal, ordinal, false, true, |pos| {
        !grid[[pos.x as usize, pos.y as usize, pos.z as usize]].is_impassable()
    })
}

/// Bresenham line algorithm for tracing a direct line in a grid.
/// This function traces a path from `start` to `goal` in the grid
/// It can be used to find all cells between two points in a grid while respecting the grid's neighborhood and movement costs.
///
/// This is mostly used internally, but could be useful if you want to "raycast" between two points and get every cell crossed over by the raycast.
///
/// # Arguments
/// * `grid` - The grid to trace the path in.
/// * `start` - The starting position in the grid.
/// * `goal` - The goal position in the grid.
/// * `ordinal` - Whether to use ordinal or cardinal movement.
/// * `filtered` - Whether to apply neighborhood filters.
/// * `aliased` - Setting to true allows aliasing which means it might be jagged. If this is false the path will be rejected if it can't reach to goal without aliazing.
///
/// # Returns
/// An `Option<Vec<UVec3>>` containing the path if successful, or `None` if the path could not be traced.
pub fn bresenham_path(
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    ordinal: bool,
    filtered: bool,
    aliased: bool,
) -> Option<Vec<UVec3>> {
    let mut path = Vec::with_capacity(32);
    let success = trace_line(grid, start, goal, ordinal, filtered, aliased, |pos| {
        if grid[[pos.x as usize, pos.y as usize, pos.z as usize]].is_impassable() {
            false
        } else {
            path.push(pos);
            true
        }
    });
    if success {
        Some(path)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use bevy::math::UVec3;
    use ndarray::Array3;

    use crate::{
        grid::{
            ChunkSettings, CollisionSettings, GridInternalSettings, GridSettings, NavSettings,
            NeighborhoodSettings,
        },
        nav::NavCell,
        prelude::*,
        raycast::{bresenham_path, has_line_of_sight},
    };

    const GRID_SETTINGS: GridSettings = GridSettings(GridInternalSettings {
        dimensions: UVec3::new(12, 12, 1),
        chunk_settings: ChunkSettings {
            size: 4,
            depth: 1,
            diagonal_connections: false,
        },
        cost_settings: NavSettings {
            default_movement_cost: 1,
            default_impassible: false,
        },
        collision_settings: CollisionSettings {
            enabled: true,
            avoidance_distance: 4,
        },
        neighborhood_settings: NeighborhoodSettings {
            filters: Vec::new(),
        },
    });

    #[test]
    fn test_line_of_sight() {
        let mut grid = Array3::from_elem((10, 10, 1), NavCell::new(Nav::Passable(1)));
        grid[[5, 5, 0]] = NavCell::new(Nav::Impassable);

        let start = UVec3::new(0, 0, 0);
        let end = UVec3::new(9, 9, 0);

        assert!(!has_line_of_sight(&grid.view(), start, end, false));
    }

    #[test]
    fn test_bresenhan_path() {
        let mut grid: Grid<OrdinalNeighborhood> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = bresenham_path(
            &grid.view(),
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            grid.neighborhood.is_ordinal(),
            false,
            false,
        );

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 11);

        let mut grid: Grid<CardinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = bresenham_path(
            &grid.view(),
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            grid.neighborhood.is_ordinal(),
            false,
            true,
        );

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 21);

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.set_nav(UVec3::new(5, 5, 0), Nav::Impassable);
        grid.build();

        let path = bresenham_path(
            &grid.view(),
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            grid.neighborhood.is_ordinal(),
            false,
            false,
        );

        assert!(path.is_none());
    }
}
