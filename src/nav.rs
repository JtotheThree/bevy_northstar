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
    /// Agents can use this cell to transition or warp to another destination.
    /// Used for stairs, ramps, ladders, as well as actual portals.
    /// The other position must be a passable cell or another portal.
    Portal(Portal),
}

impl PartialEq for Nav {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Nav::Passable(a), Nav::Passable(b)) => a == b,
            (Nav::Impassable, Nav::Impassable) => true,
            (Nav::Portal(a), Nav::Portal(b)) => a == b,
            _ => false,
        }
    }
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
    // Special neighbors for this cell, such as portals or ladders.
    pub(crate) special_neighbors: Vec<UVec3>,
}

impl NavCell {
    /// Creates a new `NavCell` with the given `Nav` state.
    pub fn new(nav: Nav) -> Self {
        Self {
            nav,
            cost: match nav {
                Nav::Passable(cost) => cost,
                Nav::Impassable => 0,
                Nav::Portal(portal) => portal.cost,
            },
            neighbor_bits: 0,
            special_neighbors: Vec::new(),
        }
    }

    /// Is this cell passable?
    pub fn is_passable(&self) -> bool {
        matches!(self.nav, Nav::Passable { .. } | Nav::Portal { .. })
    }

    /// Is this cell impassable?
    pub fn is_impassable(&self) -> bool {
        matches!(self.nav, Nav::Impassable)
    }

    /// Is this cell a portal?
    pub fn is_portal(&self) -> bool {
        matches!(self.nav, Nav::Portal { .. })
    }

    /// Returns the `Nav` state associated with this cell.
    pub fn nav(&self) -> Nav {
        self.nav
    }

    /// Returns an a vector of the neighboring positions that are passable.
    /*pub fn neighbors(&self, pos: UVec3) -> Vec<UVec3> {
        let origin = pos.as_ivec3();
        let mut neighbors = Vec::with_capacity(26 + self.special_neighbors.len()); // Max 26 3D neighbors
        
        // Use bit manipulation to iterate only set bits
        let mut bits = self.neighbor_bits;
        let mut i = 0;
        while bits != 0 {
            if bits & 1 != 0 {
                neighbors.push((origin + ORDINAL_3D_OFFSETS[i]).as_uvec3());
            }
            bits >>= 1;
            i += 1;
        }
        
        // Extend with special neighbors (no clone needed)
        neighbors.extend_from_slice(&self.special_neighbors);
        
        neighbors
    }

    /// Returns an iterator over the neighboring positions that are passable.
    pub fn neighbor_iter(&self, pos: UVec3) -> impl Iterator<Item = UVec3> + '_ {
        self.neighbors(pos).into_iter()
    }*/


    /*pub fn neighbor_iter(&self, pos: UVec3) -> impl Iterator<Item = UVec3> + '_ {
        let origin = pos.as_ivec3();
        let standard = ORDINAL_3D_OFFSETS
            .iter()
            .enumerate()
            .filter_map(move |(i, offset)| {
                if (self.neighbor_bits >> i) & 1 != 0 {
                    Some((origin + *offset).as_uvec3())
                } else {
                    None
                }
            });

        let special = self.special_neighbors.clone().into_iter();

        standard.chain(special)
    }*/

    /*pub fn neighbor_iter(&self, pos: UVec3) -> impl Iterator<Item = UVec3> + '_ {
        let origin = pos.as_ivec3();
        
        // More efficient bit iteration - skip zeros
        let bits = self.neighbor_bits;
        let standard = (0..32)
            .filter_map(move |i| {
                if (bits >> i) & 1 != 0 && i < ORDINAL_3D_OFFSETS.len() {
                    Some((origin + ORDINAL_3D_OFFSETS[i]).as_uvec3())
                } else {
                    None
                }
            });

        let special = self.special_neighbors.iter().copied();
        standard.chain(special)
    }*/

    pub fn neighbor_iter(&self, pos: UVec3) -> impl Iterator<Item = UVec3> + '_ {
        let origin = pos.as_ivec3();
        
        // Mask to only consider valid offset bits
        let mut bits = self.neighbor_bits & ((1u32 << ORDINAL_3D_OFFSETS.len()) - 1);
        
        let standard = std::iter::from_fn(move || {
            if bits == 0 {
                return None;
            }
            
            let i = bits.trailing_zeros() as usize;
            bits &= bits - 1; // Clear the lowest set bit
            
            Some((origin + ORDINAL_3D_OFFSETS[i]).as_uvec3())
        });

        let special = self.special_neighbors.iter().copied();
        standard.chain(special)
    }
}

impl Default for NavCell {
    fn default() -> Self {
        Self {
            nav: Nav::Passable(1),
            cost: 1,
            neighbor_bits: 0,
            special_neighbors: Vec::new(),
        }
    }
}

/// Represents a portal that can be used to transition to another cell in the grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Portal {
    /// The movement cost
    pub cost: MovementCost,
    /// The target position this portal goes to.
    /// For ramps you can set the target to same x,y and use a higher z position.
    pub target: UVec3,
    /// If one_way is true, a reverse portal at the target position will be created.
    /// This is useful for ramps or ladders whare you'd expect to go up or down.
    pub one_way: bool,
}

impl Portal {
    /// Creates a new `Portal` with the given target position and movement cost.
    /// Set `one_way` to true if you do not want to crate a reverse portal at the target position.
    pub fn to(target: UVec3, cost: MovementCost, one_way: bool) -> Self {
        Self {
            target,
            cost,
            one_way,
        }
    }

    /// Returns the target position of the portal.
    pub fn to_cell(&self) -> UVec3 {
        self.target
    }
}
