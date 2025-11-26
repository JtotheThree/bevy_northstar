//! This module defines the `Neighborhood` trait and its implementations for different types of neighborhoods.
use bevy::math::{IVec3, UVec3};
use ndarray::ArrayView3;
use std::sync::Arc;

use crate::{filter::NeighborFilter, grid::NeighborhoodSettings, nav::NavCell};

/// The `Neighborhood` trait defines the interface for different neighborhood types.
/// You can implement this trait to define custom neighborhoods and hueristics.
pub trait Neighborhood: Clone + Default + Sync + Send {
    /// Returns the possible directions at a position by the neighborhood.
    fn directions(&self) -> &'static [(i32, i32, i32)];

    /// Returns a u32 bitmask representing the neighbors of a given position in the grid.
    /// If you implement this method, you will need to ensure that the returned bit mask is in the correct order.
    /// The default implementation should handle almost all directional movement cases, you can likely use `filter_neighbors` instead
    /// if you want customer neighbor logic. It should handle most custom cases.
    ///
    /// You will also need to ensure the neighbors are within the bounds of the grid.
    ///
    /// Bit Mask Format:
    /// Index: Offset (x, y, z)
    /// ------ -----------------
    ///   0     (-1, -1, -1) ← North-West-Down
    ///   1     ( 0, -1, -1) ← North-Down
    ///   2     ( 1, -1, -1) ← South-West-Down
    ///   3     (-1,  0, -1) ← West-Down
    ///   4     ( 0,  0, -1) ← Down
    ///   5     ( 1,  0, -1) ← East-Down
    ///   6     (-1,  1, -1) ← North-East-Down
    ///   7     ( 0,  1, -1) ← North-Down
    ///   8     ( 1,  1, -1) ← South-East-Down
    ///   9     (-1, -1,  0) ← North-West
    ///  10     ( 0, -1,  0) ← North
    ///  11     ( 1, -1,  0) ← South-West
    ///  12     (-1,  0,  0) ← West
    ///  13     ( 1,  0,  0) ← East
    ///  14     (-1,  1,  0) ← North-East
    ///  15     ( 0,  1,  0) ← South
    ///  16     ( 1,  1,  0) ← South-East
    ///  17     (-1, -1,  1) ← North-West-Up
    ///  18     ( 0, -1,  1) ← North-Up
    ///  19     ( 1, -1,  1) ← South-West-Up
    ///  20     (-1,  0,  1) ← West-Up
    ///  21     ( 0,  0,  1) ← Up
    ///  22     ( 1,  0,  1) ← East-Up
    ///  23     (-1,  1,  1) ← North-East-Up
    ///  24     ( 0,  1,  1) ← South-Up
    ///  25     ( 1,  1,  1) ← South-East-Up
    fn neighbors(&self, grid: &ArrayView3<NavCell>, pos: UVec3) -> u32 {
        let shape = grid.shape();
        let mut bits = 0;
        let origin = pos.as_ivec3();

        for &dir in self.directions() {
            let offset = IVec3::new(dir.0, dir.1, dir.2);
            if let Some(i) = ORDINAL_3D_OFFSETS.iter().position(|&o| o == offset) {
                let neighbor = origin + offset;

                if neighbor.cmplt(IVec3::ZERO).any() {
                    continue;
                }

                let nx = neighbor.x as usize;
                let ny = neighbor.y as usize;
                let nz = neighbor.z as usize;

                if nx < shape[0]
                    && ny < shape[1]
                    && nz < shape[2]
                    && !grid[[nx, ny, nz]].is_impassable()
                {
                    bits |= 1 << i;
                }
            } else {
                panic!("Direction {offset:?} not in ORDINAL_3D_OFFSETS!");
            }
        }

        // Apply filters
        for filter in self.filters() {
            bits = filter.filter(pos, bits, grid);
        }

        bits
    }

    /// Returns the filters applied to the neighborhood.
    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>];

    /// Returns the heuristic cost from a position to a target.
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32;

    /// Returns true if the neighborhood allows ordinal movement.
    fn is_ordinal(&self) -> bool {
        false
    }
    /// Returns any settings for the neighborhood.
    fn settings(&self) -> Option<NeighborhoodSettings> {
        None
    }

    /// Optional constructor override for settings-based neighborhood
    fn from_settings(_settings: &NeighborhoodSettings) -> Self {
        Self::default()
    }
}

/// Use `CardinalNeighborhood` for standard 2D pathfinding with no diagonal movement.
#[derive(Clone, Default)]
pub struct CardinalNeighborhood {
    pub(crate) filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

impl Neighborhood for CardinalNeighborhood {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 4] = [
            (-1, 0, 0), // North
            (1, 0, 0),  // South
            (0, -1, 0), // West
            (0, 1, 0),  // East
        ];
        &DIRECTIONS
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        ((pos.x as i32 - target.x as i32).abs() + (pos.y as i32 - target.y as i32).abs()) as u32
    }

    fn from_settings(settings: &NeighborhoodSettings) -> Self {
        Self {
            filters: settings.filters.clone(),
        }
    }

    #[inline(always)]
    fn settings(&self) -> Option<NeighborhoodSettings> {
        Some(NeighborhoodSettings {
            filters: self.filters.clone(),
        })
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &self.filters
    }
}

/// Use `CardinalNeighborhood3d` for 3D pathfinding with no diagonal movement.
/// This neighborhood allows movement in the cardinal directions in 3D space only in UP or DOWN directions.
#[derive(Clone, Default)]
pub struct CardinalNeighborhood3d {
    pub(crate) filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

impl Neighborhood for CardinalNeighborhood3d {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // Cardinal directions in 3D: N, S, E, W, UP, DOWN
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 6] = [
            (-1, 0, 0), // North
            (1, 0, 0),  // South
            (0, -1, 0), // West
            (0, 1, 0),  // East
            (0, 0, -1), // Down
            (0, 0, 1),  // Up
        ];
        &DIRECTIONS
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.max(target.x) - pos.x.min(target.x);
        let dy = pos.y.max(target.y) - pos.y.min(target.y);
        let dz = pos.z.max(target.z) - pos.z.min(target.z);
        dx + dy + dz
    }

    fn from_settings(settings: &NeighborhoodSettings) -> Self {
        Self {
            filters: settings.filters.clone(),
        }
    }

    #[inline(always)]
    fn settings(&self) -> Option<NeighborhoodSettings> {
        Some(NeighborhoodSettings {
            filters: self.filters.clone(),
        })
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &self.filters
    }
}

/// Use `CardinalIsometricNeighborhood` for tile-based Cardinal 3d movment.
/// It doesn't allow diagonal movement, but still allows vertical movement in a stair-like fashion.
#[derive(Clone, Default)]
pub struct CardinalIsoNeighborhood {
    pub(crate) filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

impl Neighborhood for CardinalIsoNeighborhood {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        static DIRECTIONS: [(i32, i32, i32); 12] = [
            (-1, 0, 0), // North
            (-1, 0, 1), // North-Up
            (-1, 0, -1), // North-Down
            (1, 0, -1), // South-Down
            (1, 0, 1), // South-Up
            (1, 0, 0),  // South
            (0, -1, 0), // West
            (0, -1, 1), // West-Up
            (0, -1, -1), // West-Down
            (0, 1, -1), // East-Down
            (0, 1, 1), // East-Up
            (0, 1, 0),  // East
        ];
        &DIRECTIONS
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.abs_diff(target.x);
        let dy = pos.y.abs_diff(target.y);
        let dz = pos.z.abs_diff(target.z);
        
        // You need to move dx in X, dy in Y, and dz in Z
        // You can combine one of (dx or dy) with dz, saving min(dx,dy,dz) moves
        dx + dy + dz - dz.min(dx.min(dy))
    }

    fn from_settings(settings: &NeighborhoodSettings) -> Self {
        Self {
            filters: settings.filters.clone(),
        }
    }

    #[inline(always)]
    fn settings(&self) -> Option<NeighborhoodSettings> {
        Some(NeighborhoodSettings {
            filters: self.filters.clone(),
        })
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &self.filters
    }
}

/// Use `OrdinalNeighborhood` for 2D pathfinding with diagonal movement.
/// This neighborhood allows movement in all 8 directions.
#[derive(Clone, Default)]
pub struct OrdinalNeighborhood {
    pub(crate) filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

impl Neighborhood for OrdinalNeighborhood {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // Ordinal directions in 2D: N, S, E, W, NE, SE, SW, NW
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 8] = [
            (-1, 0, 0),  // North
            (1, 0, 0),   // South
            (0, -1, 0),  // West
            (0, 1, 0),   // East
            (-1, -1, 0), // North-East
            (1, -1, 0),  // South-East
            (1, 1, 0),   // South-West
            (-1, 1, 0),  // North-West
        ];
        &DIRECTIONS
    }

    /*#[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.abs_diff(target.x);
        let dy = pos.y.abs_diff(target.y);
        dx.max(dy)
    }*/

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = (target.x as i32 - pos.x as i32).unsigned_abs();
        let dy = (target.y as i32 - pos.y as i32).unsigned_abs();
        let dz = (target.z as i32 - pos.z as i32).unsigned_abs();

        let base = dx.max(dy).max(dz) * 1000;

        let tie_breaker = dx + dy + dz;

        base + tie_breaker // returns scaled u32
    }

    #[inline(always)]
    fn is_ordinal(&self) -> bool {
        true
    }

    fn from_settings(settings: &NeighborhoodSettings) -> Self {
        Self {
            filters: settings.filters.clone(),
        }
    }

    #[inline(always)]
    fn settings(&self) -> Option<NeighborhoodSettings> {
        Some(NeighborhoodSettings {
            filters: self.filters.clone(),
        })
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &self.filters
    }
}

/// Use `OrdinalNeighborhood3d` for 3D pathfinding with diagonal movement.
/// This neighborhood allows movement in all 26 directions.
/// It's the 3D version of `OrdinalNeighborhood`.
#[derive(Clone, Default)]
pub struct OrdinalNeighborhood3d {
    pub(crate) filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

impl Neighborhood for OrdinalNeighborhood3d {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // Ordinal directions in 3D: N, S, E, W, NE, SE, SW, NW, UP, DOWN
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 26] = [
            (-1, -1, -1), // North-West-Down
            (-1, -1, 0),  // North-West
            (-1, -1, 1),  // North-West-Up
            (-1, 0, -1),  // North-Down
            (-1, 0, 0),   // North
            (-1, 0, 1),   // North-Up
            (-1, 1, -1),  // North-East-Down
            (-1, 1, 0),   // North-East
            (-1, 1, 1),   // North-East-Up
            (0, -1, -1),  // West-Down
            (0, -1, 0),   // West
            (0, -1, 1),   // West-Up
            (0, 0, -1),   // Down
            (0, 0, 1),    // Up
            (0, 1, -1),   // East-Down
            (0, 1, 0),    // East
            (0, 1, 1),    // East-Up
            (1, -1, -1),  // South-West-Down
            (1, -1, 0),   // South-West
            (1, -1, 1),   // South-West-Up
            (1, 0, -1),   // South-Down
            (1, 0, 0),    // South
            (1, 0, 1),    // South-Up
            (1, 1, -1),   // South-East-Down
            (1, 1, 0),    // South-East
            (1, 1, 1),    // South-East-Up
        ];
        &DIRECTIONS
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        /* let dx = (target.x as i32 - pos.x as i32).abs() as u32;
        let dy = (target.y as i32 - pos.y as i32).abs() as u32;
        let dz = (target.z as i32 - pos.z as i32).abs() as u32;

        let base = dx + dy + dz;

        // Tie breaker: prefer straighter (less "jittery") paths
        let max_axis = dx.max(dy).max(dz);

        // Scaled to preserve integer math
        base * 1000 + max_axis */
        let dx = pos.x.abs_diff(target.x);
        let dy = pos.y.abs_diff(target.y);
        let dz = pos.z.abs_diff(target.z);
        dx.max(dy).max(dz)
    }

    #[inline(always)]
    fn is_ordinal(&self) -> bool {
        true
    }

    fn from_settings(settings: &NeighborhoodSettings) -> Self {
        Self {
            filters: settings.filters.clone(),
        }
    }

    #[inline(always)]
    fn settings(&self) -> Option<NeighborhoodSettings> {
        Some(NeighborhoodSettings {
            filters: self.filters.clone(),
        })
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &self.filters
    }
}

pub(crate) const ORDINAL_3D_OFFSETS: [IVec3; 26] = {
    let mut offsets = [IVec3::ZERO; 26];
    let mut index = 0;
    let mut dz = -1;
    while dz <= 1 {
        let mut dy = -1;
        while dy <= 1 {
            let mut dx = -1;
            while dx <= 1 {
                if dx != 0 || dy != 0 || dz != 0 {
                    offsets[index] = IVec3::new(dx, dy, dz);
                    index += 1;
                }
                dx += 1;
            }
            dy += 1;
        }
        dz += 1;
    }
    offsets
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cardinal_neighbors() {
        let neighborhood = CardinalNeighborhood {
            filters: Vec::new(),
        };
        let cells: [NavCell; 9] = std::array::from_fn(|_| NavCell::default());
        let grid = ArrayView3::from_shape((3, 3, 1), &cells).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(1, 1, 0));

        assert_eq!(neighbors.count_ones(), 4);
    }

    #[test]
    fn test_cardinal_neighbors_3d() {
        let neighborhood = CardinalNeighborhood3d {
            filters: Vec::new(),
        };
        let cells: [NavCell; 27] = std::array::from_fn(|_| NavCell::default());
        let grid = ArrayView3::from_shape((3, 3, 3), &cells).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(1, 1, 1));

        assert_eq!(neighbors.count_ones(), 6);
    }

    #[test]
    fn test_ordinal_neighbors_3d() {
        let neighborhood = OrdinalNeighborhood3d {
            filters: Vec::new(),
        };
        let cells: [NavCell; 27] = std::array::from_fn(|_| NavCell::default());
        let grid = ArrayView3::from_shape((3, 3, 3), &cells).unwrap();

        let neighbors_mask = neighborhood.neighbors(&grid, UVec3::new(1, 1, 1));

        // All 26 neighbors should be available (all bits set)
        assert_eq!(neighbors_mask.count_ones(), 26);

        // Check that each bit corresponds to a valid neighbor position
        let mut expected_positions = Vec::new();
        let center = UVec3::new(1, 1, 1);
        let mut bit = 0;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue; // skip center
                    }
                    let pos = UVec3::new(
                        (center.x as i32 + dx) as u32,
                        (center.y as i32 + dy) as u32,
                        (center.z as i32 + dz) as u32,
                    );
                    // The bit at position `bit` should be set
                    assert!(
                        (neighbors_mask & (1 << bit)) != 0,
                        "Neighbor bit {bit} (pos {pos:?}) not set"
                    );
                    expected_positions.push(pos);
                    bit += 1;
                }
            }
        }
        assert_eq!(bit, 26);
    }

    #[test]
    #[allow(clippy::identity_op)]
    fn test_ordinal_neighbors_at_0() {
        let neighborhood = OrdinalNeighborhood3d {
            filters: Vec::new(),
        };
        let cells: [NavCell; 27] = std::array::from_fn(|_| NavCell::default());
        let grid = ArrayView3::from_shape((3, 3, 3), &cells).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(0, 0, 0));

        // Count the number of set bits (neighbors)
        assert_eq!(neighbors.count_ones(), 7);

        // Check that the correct neighbor bits are set
        let expected_positions = [
            (0, 0, 1),
            (0, 1, 0),
            (0, 1, 1),
            (1, 0, 0),
            (1, 0, 1),
            (1, 1, 0),
            (1, 1, 1),
        ];

        let mut bit = 0;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let pos = (
                        (0_i32 + dx) as u32,
                        (0_i32 + dy) as u32,
                        (0_i32 + dz) as u32,
                    );
                    if expected_positions.contains(&pos) {
                        assert!(
                            (neighbors & (1 << bit)) != 0,
                            "Expected neighbor at {pos:?} (bit {bit}) to be set"
                        );
                    } else {
                        assert!(
                            (neighbors & (1 << bit)) == 0,
                            "Unexpected neighbor at {pos:?} (bit {bit}) set"
                        );
                    }
                    bit += 1;
                }
            }
        }
    }

    #[test]
    #[allow(clippy::identity_op)]
    fn test_ordinal_neighbors_no_depth() {
        let neighborhood = OrdinalNeighborhood3d {
            filters: Vec::new(),
        };
        let cells: [NavCell; 9] = std::array::from_fn(|_| NavCell::default());

        let grid = ArrayView3::from_shape((3, 3, 1), &cells).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(1, 1, 0));

        // There should be 8 neighbors (all 2D surrounding positions)
        assert_eq!(neighbors.count_ones(), 8);

        let expected_positions = [
            (0, 0, 0),
            (0, 1, 0),
            (0, 2, 0),
            (1, 0, 0),
            (1, 2, 0),
            (2, 0, 0),
            (2, 1, 0),
            (2, 2, 0),
        ];

        let mut bit = 0;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let pos = (
                        (1_i32 + dx) as u32,
                        (1_i32 + dy) as u32,
                        (0_i32 + dz) as u32,
                    );
                    if expected_positions.contains(&pos) {
                        assert!(
                            (neighbors & (1 << bit)) != 0,
                            "Expected neighbor at {pos:?} (bit {bit}) to be set"
                        );
                    } else {
                        assert!(
                            (neighbors & (1 << bit)) == 0,
                            "Unexpected neighbor at {pos:?} (bit {bit}) set"
                        );
                    }
                    bit += 1;
                }
            }
        }

        assert_eq!(bit, 26);
    }

    #[test]
    fn test_ordinal_heuristic() {
        let neighborhood = OrdinalNeighborhood3d {
            filters: Vec::new(),
        };

        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(1, 1, 1)),
            1
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(1, 0, 0)),
            1
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(0, 0, 0)),
            0
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(2, 2, 2)),
            2
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(7, 7, 7)),
            7
        );
    }
}
