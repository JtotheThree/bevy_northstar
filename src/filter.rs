//! This module defines some preset `NeighborFilter`s for filtering neighbors based on specific movement rules.
use bevy::{
    math::{IVec3, UVec3},
    platform::collections::HashSet,
};
use ndarray::ArrayView3;
use std::fmt::Debug;

use crate::{nav::NavCell, prelude::ORDINAL_3D_OFFSETS};

/// [`NeighborFilter`] trait to add custom filtering logic to the neighbors returned by the [`crate::neighbor::Neighborhood::neighbors()`] method.
/// Add a filter to the grid settings with [`crate::grid::GridSettingsBuilder::add_neighbor_filter()`].
pub trait NeighborFilter {
    /// Filters the neighbors of a given position in the grid.
    fn filter(&self, pos: UVec3, mask: u32, grid: &ArrayView3<NavCell>) -> u32;
}

/// Disallow diagonal movement if both adjacent cardinals in the direction of movement are solid.
/// Prevents agents from clipping through wall corners like this where is x is a wall:
/// |x|/|
/// |/|x|
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NoCornerClipping;

impl NeighborFilter for NoCornerClipping {
    fn filter(&self, pos: UVec3, mut mask: u32, grid: &ArrayView3<NavCell>) -> u32 {
        let shape = grid.shape();
        let origin = pos.as_ivec3();

        for (i, &offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            // Skip if this neighbor isn't currently included
            if (mask >> i) & 1 == 0 {
                continue;
            }

            // Only apply filter for 2D diagonal moves
            if offset.z == 0 && offset.x != 0 && offset.y != 0 {
                // Get the adjacent cardinals
                let cardinal1 = origin + IVec3::new(offset.x, 0, 0);
                let cardinal2 = origin + IVec3::new(0, offset.y, 0);

                let solid_cardinals = [cardinal1, cardinal2]
                    .iter()
                    .filter(|&&c| {
                        c.cmplt(IVec3::ZERO).any()
                            || c.x >= shape[0] as i32
                            || c.y >= shape[1] as i32
                            || c.z >= shape[2] as i32
                            || grid[[c.x as usize, c.y as usize, c.z as usize]].is_impassable()
                    })
                    .count();

                // Disallow diagonal if both cardinals are solid
                if solid_cardinals == 2 {
                    mask &= !(1 << i); // Clear the bit
                }
            }
        }

        mask
    }
}

/// Disallow diagonal movement if *any* adjacent cardinal in the direction of movement is solid.
/// Prevents cutting around corners like this where x is the wall:
/// |x|/|
/// |/| |
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NoCornerCutting;

impl NeighborFilter for NoCornerCutting {
    fn filter(&self, pos: UVec3, mut mask: u32, grid: &ArrayView3<NavCell>) -> u32 {
        let origin = pos.as_ivec3();
        let shape = grid.shape();

        // Build a lookup set of which cardinals are solid
        let mut solid_dirs = HashSet::new();
        for offset in ORDINAL_3D_OFFSETS.iter() {
            let neighbor = origin + *offset;
            if neighbor.cmplt(IVec3::ZERO).any()
                || neighbor.x >= shape[0] as i32
                || neighbor.y >= shape[1] as i32
                || neighbor.z >= shape[2] as i32
            {
                continue;
            }

            let neighbor = UVec3::new(neighbor.x as u32, neighbor.y as u32, neighbor.z as u32);

            if grid[[
                neighbor.x as usize,
                neighbor.y as usize,
                neighbor.z as usize,
            ]]
            .is_impassable()
            {
                solid_dirs.insert(*offset);
            }
        }

        // For each diagonal, disallow if *any* adjacent cardinal in that direction is solid
        for (i, offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            if (mask >> i) & 1 == 0 {
                continue;
            }

            let dx = offset.x.signum();
            let dy = offset.y.signum();
            let dz = offset.z.signum();

            let cardinal_dirs = [
                IVec3::new(dx, 0, 0),
                IVec3::new(0, dy, 0),
                IVec3::new(0, 0, dz),
            ];

            if cardinal_dirs.iter().any(|dir| solid_dirs.contains(dir)) {
                mask &= !(1 << i); // Disallow this ordinal
            }
        }

        mask
    }
}

/// This filter is the same as [`NoCornerCutting`]` with the exception that it only disallows purely horizontal diagonals.
/// This will work better in a 3d tilemap situation where you want your player to be able to jump heights but not cut corners horizontally.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NoCornerCuttingFlat;

impl NeighborFilter for NoCornerCuttingFlat {
    fn filter(&self, pos: UVec3, mut mask: u32, grid: &ArrayView3<NavCell>) -> u32 {
        let origin = pos.as_ivec3();
        let shape = grid.shape();

        // Build a lookup set of which cardinals are solid
        let mut solid_dirs = HashSet::new();
        for offset in ORDINAL_3D_OFFSETS.iter() {
            let neighbor = origin + *offset;
            if neighbor.cmplt(IVec3::ZERO).any()
                || neighbor.x >= shape[0] as i32
                || neighbor.y >= shape[1] as i32
                || neighbor.z >= shape[2] as i32
            {
                continue;
            }

            let neighbor = UVec3::new(neighbor.x as u32, neighbor.y as u32, neighbor.z as u32);

            if grid[[
                neighbor.x as usize,
                neighbor.y as usize,
                neighbor.z as usize,
            ]]
            .is_impassable()
            {
                solid_dirs.insert(*offset);
            }
        }

        // For each diagonal, disallow only if it is purely horizontal and blocked
        for (i, offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            if (mask >> i) & 1 == 0 {
                continue;
            }

            let dx = offset.x.signum();
            let dy = offset.y.signum();
            let dz = offset.z.signum();

            // Skip diagonals that involve a vertical (z-axis) change
            if dz != 0 {
                continue;
            }

            let cardinal_dirs = [IVec3::new(dx, 0, 0), IVec3::new(0, dy, 0)];

            if cardinal_dirs.iter().any(|dir| solid_dirs.contains(dir)) {
                mask &= !(1 << i); // Disallow this ordinal
            }
        }

        mask
    }
}

/// Disallow diagonal movement involving Z.
/// Keeps cardinal movement and 2D (XY) diagonals.
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DisallowDiagonalZMovement;

impl NeighborFilter for DisallowDiagonalZMovement {
    fn filter(&self, _pos: UVec3, mut mask: u32, _grid: &ArrayView3<NavCell>) -> u32 {
        for (i, &offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            if (mask >> i) & 1 == 0 {
                continue;
            }

            // Disallow any movement involving Z and also X or Y
            if offset.z != 0 && (offset.x != 0 || offset.y != 0) {
                mask &= !(1 << i);
            }
        }

        mask
    }
}

#[cfg(test)]
mod tests {
    use crate::nav::Nav;

    use super::*;
    use ndarray::Array3;

    #[test]
    fn test_no_corner_clipping() {
        let mut grid = Array3::<NavCell>::default((3, 3, 1));
        grid[[1, 2, 0]] = NavCell::new(Nav::Impassable);
        grid[[2, 1, 0]] = NavCell::new(Nav::Impassable);

        let pos = UVec3::new(1, 1, 0);
        let mask = u32::MAX; // All directions allowed

        let filter = NoCornerClipping;
        let filtered_mask = filter.filter(pos, mask, &grid.view());

        for (i, offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            if *offset == IVec3::new(1, 1, 0) {
                assert_eq!(
                    (filtered_mask >> i) & 1,
                    0,
                    "Diagonal Northeast should be filtered"
                );
            } else {
                // All others still allowed
                assert_eq!(
                    (filtered_mask >> i) & 1,
                    1,
                    "Direction {offset:?} should remain allowed"
                );
            }
        }
    }

    #[test]
    fn test_no_corner_cutting() {
        let mut grid = Array3::<NavCell>::default((3, 3, 1));
        grid[[2, 1, 0]] = NavCell::new(Nav::Impassable);

        // |o|o|o|
        // |o|*|x|
        // |o|o|o|

        let pos = UVec3::new(1, 1, 0);
        let mask = u32::MAX; // All directions allowed

        let filter = NoCornerCutting;
        let filtered_mask = filter.filter(pos, mask, &grid.view());

        for (i, offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            let should_be_disallowed = offset.x > 0;

            let allowed = (filtered_mask >> i) & 1 == 1;
            if should_be_disallowed {
                assert!(
                    !allowed,
                    "Direction {offset:?} should be disallowed due to blocked EAST"
                );
            } else {
                assert!(allowed, "Direction {offset:?} should remain allowed");
            }
        }
    }

    #[test]
    fn test_disallow_diagonal_z_movement() {
        let grid = Array3::<NavCell>::default((3, 3, 3));
        let pos = UVec3::new(1, 1, 1);
        let mask = u32::MAX;

        let filter = DisallowDiagonalZMovement;
        let filtered_mask = filter.filter(pos, mask, &grid.view());

        for (i, offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            let bit_set = (filtered_mask >> i) & 1 == 1;

            if offset.z != 0 && (offset.x != 0 || offset.y != 0) {
                assert!(
                    !bit_set,
                    "Direction {offset:?} should be disallowed (Z-diagonal)"
                );
            } else {
                assert!(bit_set, "Direction {offset:?} should remain allowed");
            }
        }
    }
}
