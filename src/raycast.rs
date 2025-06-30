//! Raycasting and pathfinding utilities for 2D/3D grids.
use bevy::math::UVec3;
use ndarray::ArrayView3;

use crate::nav::NavCell;

/// Check if there is a line of sight between two positions in the grid allowing diagonal movement.
///
/// Arguments:
/// * `grid` - A 3D array view of [`NavCell`]s representing the grid.
/// * `start` - The starting [`UVec3`].
/// * `end` - The ending position [`UVec3`].
///
/// Returns:
/// * `true` if there is a line of sight between the two positions.
/// * `false` if there is an obstacle between the two positions.
///
/// # Example
///
/// ```
/// use bevy::math::UVec3;
/// use ndarray::Array3;
/// use bevy_northstar::nav::{Nav, NavCell};
/// use bevy_northstar::raycast::line_of_sight;
///
/// let mut grid = Array3::from_elem((10, 10, 1), NavCell::new(Nav::Passable(1)));
/// grid[[5, 5, 0]] = NavCell::new(Nav::Impassable);
///
/// let start = UVec3::new(0, 0, 0);
/// let end = UVec3::new(9, 9, 0);
///
/// assert_eq!(line_of_sight(&grid.view(), start, end), false);
/// ```
pub fn line_of_sight(grid: &ArrayView3<NavCell>, start: UVec3, end: UVec3) -> bool {
    // TDDO: This can be optimized using integers
    let start = start.as_vec3();
    let end = end.as_vec3();

    let mut x = start.x;
    let mut y = start.y;
    let mut z = start.z;

    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let dz = end.z - start.z;

    let steps = dx.abs().max(dy.abs()).max(dz.abs());
    if steps == 0.0 {
        return true;
    }

    let x_step = dx / steps;
    let y_step = dy / steps;
    let z_step = dz / steps;

    for _ in 0..=steps as i32 {
        let grid_x = x.round() as usize;
        let grid_y = y.round() as usize;
        let grid_z = z.round() as usize;

        if grid[[grid_x, grid_y, grid_z]].is_impassable() {
            return false; // Hit an obstacle
        }

        x += x_step;
        y += y_step;
        z += z_step;
    }

    true
}

pub(crate) fn path_line_trace(
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
) -> Option<Vec<UVec3>> {
    let mut path = vec![start];
    let mut current = start;

    // Max steps to prevent infinite loops
    let max_steps = (goal.as_ivec3() - start.as_ivec3()).abs().max_element() * 2;

    if max_steps == 0 {
        return Some(path); // Already at the goal
    }

    for _ in 0..=max_steps {
        if current == goal {
            return Some(path);
        }

        let dir_to_goal = (goal.as_ivec3() - current.as_ivec3()).as_vec3().normalize();

        let cell = &grid[[current.x as usize, current.y as usize, current.z as usize]];

        let mut best = None;
        let mut best_dot = -f32::INFINITY;

        for neighbor in cell.neighbor_iter(current) {
            let to_neighbor = (neighbor.as_ivec3() - current.as_ivec3())
                .as_vec3()
                .normalize();
            let dot = dir_to_goal.dot(to_neighbor);
            if dot > best_dot {
                best = Some(neighbor);
                best_dot = dot;
            }
        }

        if let Some(next) = best {
            path.push(next);
            current = next;
        } else {
            return None; // no valid step toward goal
        }
    }

    None
}

// Trace a line from start to goal and get the Bresenham path only if the path doesn't collide with a wall
// This should take into account the Neighborhood and the grid
pub(crate) fn bresenham_path_filtered(
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    ordinal: bool,
) -> Option<Vec<UVec3>> {
    let mut path = Vec::new();
    let mut current = start;

    let (width, height, depth) = (
        grid.shape()[0] as u32,
        grid.shape()[1] as u32,
        grid.shape()[2] as u32,
    );

    // Differences in each dimension
    let dx = (goal.x as i32 - start.x as i32).abs();
    let dy = (goal.y as i32 - start.y as i32).abs();
    let dz = if depth > 1 {
        (goal.z as i32 - start.z as i32).abs()
    } else {
        0 // Ignore z axis if grid depth is 1
    };

    let sx: i32 = if start.x < goal.x { 1 } else { -1 };
    let sy: i32 = if start.y < goal.y { 1 } else { -1 };
    let sz: i32 = if depth > 1 && start.z < goal.z { 1 } else { -1 };

    let mut err_xy = dx - dy;
    let mut err_xz = dx - dz;

    while current != goal {
        // Bounds check
        if current.x >= width || current.y >= height || current.z >= depth {
            return None;
        }

        path.push(current);

        if grid[[current.x as usize, current.y as usize, current.z as usize]].is_impassable() {
            return None;
        }

        // Error-based stepping
        let double_err_xy = 2 * err_xy;
        let double_err_xz = 2 * err_xz;

        if ordinal {
            let mut moved = false;

            if double_err_xy >= -dy && double_err_xz >= -dz {
                // Move along x-axis
                err_xy -= dy;
                err_xz -= dz;

                let next = UVec3::new(current.x.saturating_add_signed(sx), current.y, current.z);

                let mut neighbors = grid
                    [[current.x as usize, current.y as usize, current.z as usize]]
                .neighbor_iter(current);

                if neighbors.any(|n| n == next) {
                    current.x = next.x;
                    moved = true;
                } else {
                    return None; // No valid step along x-axis
                }
            }
            if double_err_xy < dx {
                // Move along y-axis
                err_xy += dx;

                let next = UVec3::new(current.x, current.y.saturating_add_signed(sy), current.z);

                let mut neighbors = grid
                    [[current.x as usize, current.y as usize, current.z as usize]]
                .neighbor_iter(current);

                if neighbors.any(|n| n == next) {
                    current.y = next.y;
                    moved = true;
                } else {
                    return None; // No valid step along x-axis
                }
            }
            if depth > 1 && double_err_xz < dx {
                // Move along z-axis (if applicable)
                err_xz += dx;

                let next = UVec3::new(current.x, current.y, current.z.saturating_add_signed(sz));

                let mut neighbors = grid
                    [[current.x as usize, current.y as usize, current.z as usize]]
                .neighbor_iter(current);

                if neighbors.any(|n| n == next) {
                    current.z = next.z;
                    moved = true;
                } else {
                    return None; // No valid step along x-axis
                }
            }

            if !moved {
                return None; // No valid step found
            }
        } else {
            let mut moved = false;

            if double_err_xy >= -dy && double_err_xz >= -dz {
                // Move along x-axis
                err_xy -= dy;
                err_xz -= dz;

                let next = UVec3::new(current.x.saturating_add_signed(sx), current.y, current.z);

                let mut neighbors = grid
                    [[current.x as usize, current.y as usize, current.z as usize]]
                .neighbor_iter(current);

                if neighbors.any(|n| n == next) {
                    current.x = next.x;
                    moved = true;
                } else {
                    return None; // No valid step along x-axis
                }
            } else if double_err_xy < dx {
                // Move along y-axis
                err_xy += dx;

                let next = UVec3::new(current.x, current.y.saturating_add_signed(sy), current.z);

                let mut neighbors = grid
                    [[current.x as usize, current.y as usize, current.z as usize]]
                .neighbor_iter(current);

                if neighbors.any(|n| n == next) {
                    current.y = next.y;
                    moved = true;
                } else {
                    return None; // No valid step along y-axis
                }
            } else if depth > 1 && double_err_xz < dx {
                // Move along z-axis (if applicable)
                err_xz += dx;

                let next = UVec3::new(current.x, current.y, current.z.saturating_add_signed(sz));

                let mut neighbors = grid
                    [[current.x as usize, current.y as usize, current.z as usize]]
                .neighbor_iter(current);

                if neighbors.any(|n| n == next) {
                    current.z = next.z;
                    moved = true;
                } else {
                    return None; // No valid step along z-axis
                }
            }

            if !moved {
                return None; // No valid step found
            }
        }
    }

    path.push(goal);

    Some(path)
}

// Trace a line from start to goal and get the Bresenham path only if the path doesn't collide with a wall
// This should take into account the Neighborhood and the grid
pub(crate) fn bresenham_path(
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    ordinal: bool,
) -> Option<Vec<UVec3>> {
    let mut path = Vec::new();
    let mut current = start;

    let (width, height, depth) = (
        grid.shape()[0] as u32,
        grid.shape()[1] as u32,
        grid.shape()[2] as u32,
    );

    // Differences in each dimension
    let dx = (goal.x as i32 - start.x as i32).abs();
    let dy = (goal.y as i32 - start.y as i32).abs();
    let dz = if depth > 1 {
        (goal.z as i32 - start.z as i32).abs()
    } else {
        0 // Ignore z axis if grid depth is 1
    };

    let sx: i32 = if start.x < goal.x { 1 } else { -1 };
    let sy: i32 = if start.y < goal.y { 1 } else { -1 };
    let sz: i32 = if depth > 1 && start.z < goal.z { 1 } else { -1 };

    let mut err_xy = dx - dy;
    let mut err_xz = dx - dz;

    while current != goal {
        // Bounds check
        if current.x >= width || current.y >= height || current.z >= depth {
            return None;
        }

        path.push(current);

        if grid[[current.x as usize, current.y as usize, current.z as usize]].is_impassable() {
            return None;
        }

        /*if blocking.contains_key(&current) {
            log::info!("Bresenham path blocked by entity: {:?}, {:?}", blocking[&current], blocking);
            return None;
        }*/

        // Error-based stepping
        let double_err_xy = 2 * err_xy;
        let double_err_xz = 2 * err_xz;

        if ordinal {
            /*if !allow_corner_clipping {
                // Determine the tentative next cell (but do NOT update current yet)
                let mut next = current;

                if double_err_xy >= -dy && double_err_xz >= -dz {
                    next.x = next.x.saturating_add_signed(sx);
                }
                if double_err_xy < dx {
                    next.y = next.y.saturating_add_signed(sy);
                }
                if depth > 1 && double_err_xz < dx {
                    next.z = next.z.saturating_add_signed(sz);
                }

                let delta = next.as_ivec3() - current.as_ivec3();

                // Check 2D XY corner clipping
                if delta.x != 0 && delta.y != 0 {
                    let step_x = UVec3::new((current.x as i32 + delta.x) as u32, current.y, current.z);
                    let step_y = UVec3::new(current.x, (current.y as i32 + delta.y) as u32, current.z);
                    if grid[[step_x.x as usize, step_x.y as usize, step_x.z as usize]].solid ||
                       grid[[step_y.x as usize, step_y.y as usize, step_y.z as usize]].solid {
                        return None;
                    }
                }

                // Check XZ
                if depth > 1 && delta.x != 0 && delta.z != 0 {
                    let step_x = UVec3::new((current.x as i32 + delta.x) as u32, current.y, current.z);
                    let step_z = UVec3::new(current.x, current.y, (current.z as i32 + delta.z) as u32);

                    if grid[[step_x.x as usize, step_x.y as usize, step_x.z as usize]].solid ||
                       grid[[step_z.x as usize, step_z.y as usize, step_z.z as usize]].solid {
                        return None;
                    }
                }

                // Check YZ
                if depth > 1 && delta.y != 0 && delta.z != 0 {
                    let step_y = UVec3::new(current.x, (current.y as i32 + delta.y) as u32, current.z);
                    let step_z = UVec3::new(current.x, current.y, (current.z as i32 + delta.z) as u32);

                    if grid[[step_y.x as usize, step_y.y as usize, step_y.z as usize]].solid ||
                       grid[[step_z.x as usize, step_z.y as usize, step_z.z as usize]].solid {
                        return None;
                    }
                }

                // Optional: XYZ diagonal clipping (for full 3D corner case)
                if depth > 1 && delta.x != 0 && delta.y != 0 && delta.z != 0 {
                    let xy = UVec3::new((current.x as i32 + delta.x) as u32, (current.y as i32 + delta.y) as u32, current.z);
                    let xz = UVec3::new((current.x as i32 + delta.x) as u32, current.y, (current.z as i32 + delta.z) as u32);
                    let yz = UVec3::new(current.x, (current.y as i32 + delta.y) as u32, (current.z as i32 + delta.z) as u32);

                    if grid[[xy.x as usize, xy.y as usize, xy.z as usize]].solid ||
                       grid[[xz.x as usize, xz.y as usize, xz.z as usize]].solid ||
                       grid[[yz.x as usize, yz.y as usize, yz.z as usize]].solid {
                        return None;
                    }
                }
            }*/

            if double_err_xy >= -dy && double_err_xz >= -dz {
                // Move along x-axis
                err_xy -= dy;
                err_xz -= dz;
                current.x = current.x.saturating_add_signed(sx);
            }
            if double_err_xy < dx {
                // Move along y-axis
                err_xy += dx;
                current.y = current.y.saturating_add_signed(sy);
            }
            if depth > 1 && double_err_xz < dx {
                // Move along z-axis (if applicable)
                err_xz += dx;
                current.z = current.z.saturating_add_signed(sz);
            }
        } else if double_err_xy >= -dy && double_err_xz >= -dz {
            // Move along x-axis
            err_xy -= dy;
            err_xz -= dz;
            current.x = current.x.saturating_add_signed(sx);
        } else if double_err_xy < dx {
            // Move along y-axis
            err_xy += dx;
            current.y = current.y.saturating_add_signed(sy);
        } else if depth > 1 && double_err_xz < dx {
            // Move along z-axis (if applicable)
            err_xz += dx;
            current.z = current.z.saturating_add_signed(sz);
        }
    }

    path.push(goal);

    Some(path)
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
        raycast::{bresenham_path, line_of_sight},
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

        assert!(!line_of_sight(&grid.view(), start, end));
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
        );

        assert!(path.is_none());
    }
}
