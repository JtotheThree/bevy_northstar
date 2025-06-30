//! `Nav` and `NavCell` structs for navigation and movement cost data.
use bevy::math::UVec3;

use crate::{prelude::ORDINAL_3D_OFFSETS, MovementCost};

/// Navigation state for a cell (position) in the `Grid`.
#[derive(Clone, Copy, Debug)]
pub enum Nav {
    /// Agents can move this cell with a defined movement cost.
    Passable(MovementCost),
    /// Agents cannot move through this cell ever, it is solid.
    Impassable,
    /// Agents can use this cell to transition to a different height level,
    /// such as a ramp/slope/staircase, with a defined movement cost.
    Ramp(MovementCost),
}

/// [`NavCell`] represents the navigation data for a position in the grid.
/// Normal use shouldn't require direct interaction with this struct,
#[derive(Debug, Clone)]
pub struct NavCell {
    /// Whether the cell is walkable, solid, or a ramp and its associated movement cost.
    pub(crate) nav: Nav,
    // Internal storage for fast lookup
    pub(crate) cost: MovementCost,
    // Cached neighbors for this cell.
    pub(crate) neighbor_bits: u32,
}

impl NavCell {
    /// Creates a new `NavCell` with the given `Nav` state.
    pub fn new(nav: Nav) -> Self {
        Self {
            nav,
            cost: match nav {
                Nav::Passable(cost) => cost,
                Nav::Ramp(cost) => cost,
                Nav::Impassable => 0,
            },
            neighbor_bits: 0,
        }
    }

    /// Is this cell passable?
    pub fn is_passable(&self) -> bool {
        matches!(self.nav, Nav::Passable { .. } | Nav::Ramp { .. })
    }

    /// Is this cell impassable?
    pub fn is_impassable(&self) -> bool {
        matches!(self.nav, Nav::Impassable)
    }

    /// Is this cell a ramp?
    pub fn is_ramp(&self) -> bool {
        matches!(self.nav, Nav::Ramp { .. })
    }

    /// Returns the `Nav` state associated with this cell.
    pub fn nav(&self) -> Nav {
        self.nav
    }

    /// Returns an iterator over the neighboring positions that are passable.
    pub fn neighbor_iter(&self, pos: UVec3) -> impl Iterator<Item = UVec3> + '_ {
        let origin = pos.as_ivec3();
        ORDINAL_3D_OFFSETS
            .iter()
            .enumerate()
            .filter_map(move |(i, offset)| {
                if (self.neighbor_bits >> i) & 1 != 0 {
                    Some((origin + *offset).as_uvec3())
                } else {
                    None
                }
            })
    }
}

impl Default for NavCell {
    fn default() -> Self {
        Self {
            nav: Nav::Passable(1),
            cost: 1,
            neighbor_bits: 0,
        }
    }
}
