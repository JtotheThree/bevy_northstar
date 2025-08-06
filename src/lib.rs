//#![deny(missing_docs)]
#![doc = include_str!("../README.md")]

use bevy::ecs::query::Without;
use bevy::math::{IVec3, UVec3};
use indexmap::IndexMap;
use rustc_hash::FxHasher;
use std::cmp::Ordering;
use std::hash::BuildHasherDefault;

mod astar;
mod chunk;
pub mod components;
pub mod debug;
mod dijkstra;
pub mod dir;
pub mod filter;
mod flood_fill;
mod graph;
pub mod grid;
mod macros;
pub mod nav;
pub mod neighbor;
mod node;
pub mod path;
pub mod pathfind;
pub mod plugin;
pub mod raycast;
mod thetastar;

/// Crate Prelude
pub mod prelude {
    pub use crate::components::*;
    pub use crate::debug::{DebugTilemapType, NorthstarDebugPlugin};
    pub use crate::dir::Dir;
    pub use crate::filter;
    pub use crate::grid::{Grid, GridSettingsBuilder};
    pub use crate::nav::{Nav, Portal};
    pub use crate::neighbor::*;
    pub use crate::path::Path;
    pub use crate::plugin::{
        BlockingMap, NorthstarPlugin, NorthstarPluginSettings, PathfindSettings, PathingSet, Stats,
    };
    pub use crate::MovementCost;
    pub use crate::{CardinalGrid, CardinalGrid3d, OrdinalGrid, OrdinalGrid3d};
}

/// Alias for movement cost type.
pub type MovementCost = u32;

/// Alias for a 2d CardinalNeighborhood grid. Allows only 4 directions (N, S, E, W).
pub type CardinalGrid = grid::Grid<neighbor::CardinalNeighborhood>;
/// Alias for a 3d CardinalNeighborhood grid. Allows cardinal directions and up and down.
pub type CardinalGrid3d = grid::Grid<neighbor::CardinalNeighborhood3d>;
/// Alias for a 2d OrdinalNeighborhood grid. Allows all 8 direcitons.
pub type OrdinalGrid = grid::Grid<neighbor::OrdinalNeighborhood>;
/// Alias for a 3d OrdinalNeighborhood grid. Allows all 26 directions.
pub type OrdinalGrid3d = grid::Grid<neighbor::OrdinalNeighborhood3d>;

/// No pathing failure markers
pub type WithoutPathingFailures = (
    Without<components::NextPos>,
    Without<components::AvoidanceFailed>,
    Without<components::RerouteFailed>,
);

pub(crate) type NodeId = usize;

type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;

pub(crate) struct SmallestCostHolder<Id> {
    estimated_cost: Id,
    cost: Id,
    index: usize,
}

impl<Id: PartialEq> PartialEq for SmallestCostHolder<Id> {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl<Id: Eq> Eq for SmallestCostHolder<Id> {}

/* Greedy A* implementation from the rust Pathfinding crate
  It's meant to be faster, but is actually quite a bit slower testing it in the stress demo
  and ~10% slower in the benchmarks.

impl<Id: Ord> PartialOrd for SmallestCostHolder<Id> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<Id: Ord> Ord for SmallestCostHolder<Id> {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}
*/

impl<Id: Ord + std::ops::Add<Output = Id> + Copy> PartialOrd for SmallestCostHolder<Id> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<Id: Ord + std::ops::Add<Output = Id> + Copy> Ord for SmallestCostHolder<Id> {
    fn cmp(&self, other: &Self) -> Ordering {
        let self_total = self.cost + self.estimated_cost;
        let other_total = other.cost + other.estimated_cost;

        // Reverse ordering for min-heap behavior
        other_total.cmp(&self_total)
    }
}

#[inline(always)]
pub(crate) fn in_bounds_3d(pos: UVec3, min: UVec3, max: UVec3) -> bool {
    pos.x.wrapping_sub(min.x) < (max.x - min.x)
        && pos.y.wrapping_sub(min.y) < (max.y - min.y)
        && pos.z.wrapping_sub(min.z) < (max.z - min.z)
}

#[inline(always)]
/// Returns the min and max corners of a cubic window around the given center position.
/// This is inclusive on both ends.
fn position_in_cubic_window(pos: UVec3, center: IVec3, radius: i32, grid_shape: IVec3) -> bool {
    let min = (center - IVec3::splat(radius)).clamp(IVec3::ZERO, grid_shape - 1);
    let max = (center + IVec3::splat(radius)).clamp(IVec3::ZERO, grid_shape - 1);

    // Check if position is in the cubic window
    pos.as_ivec3().cmplt(min).any()
        || pos.as_ivec3().cmple(max).any()
        || pos.as_ivec3().cmpeq(center).all()
        || pos.as_ivec3().cmpeq(min).all()
        || pos.as_ivec3().cmpeq(max).all()
        || (pos.as_ivec3() - center).abs().max_element() <= radius
}
