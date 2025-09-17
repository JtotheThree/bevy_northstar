#![deny(missing_docs)]
#![doc = include_str!("../README.md")]

use bevy::ecs::query::Without;
use bevy::math::{IVec3, UVec3};
use bevy::reflect::Reflect;
use indexmap::IndexMap;
use ndarray::ArrayView3;
use rustc_hash::FxHasher;
use std::cmp::Ordering;
use std::hash::BuildHasherDefault;

use crate::nav::NavCell;

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
mod hpa;
mod macros;
pub mod nav;
pub mod nav_mask;
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
    pub use crate::nav_mask::{NavCellMask, NavMask, NavMaskLayer, NavMaskResult};
    pub use crate::neighbor::*;
    pub use crate::path::Path;
    pub use crate::pathfind::PathfindArgs;
    pub use crate::plugin::{
        BlockingMap, NorthstarPlugin, NorthstarPluginSettings, PathfindSettings, PathingSet, Stats,
    };
    pub use crate::MovementCost;
    pub use crate::NavRegion;
    pub use crate::SearchLimits;
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

/// Sets the limits for the pathfinding request.
#[derive(Clone, Copy, Debug, Default, Reflect)]
pub struct SearchLimits {
    /// Limit the search to a specific region
    pub boundary: Option<NavRegion>,
    /// Limits the search to abort if it exceeds a certain distance
    pub distance: Option<u32>,
    /// If true, the pathfinding will return the best path in the direction of the goal if it isn't reachable.
    pub partial: bool,
}

/// A Region3d with an iter method to iterate over all positions in the region.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Reflect)]
pub struct NavRegion {
    /// The minimum position of the region.
    pub min: UVec3,
    /// The maximum position of the region (exclusive).
    pub max: UVec3,
}

impl NavRegion {
    /// Creates a new Region3d with the given minimum and maximum positions.
    pub fn new(min: UVec3, max: UVec3) -> Self {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "Invalid region bounds"
        );
        Self { min, max }
    }

    /// Create a new Region3d from a grid's shape.
    pub fn from_grid(grid: &ArrayView3<NavCell>) -> Self {
        let shape = grid.shape();
        Self {
            min: UVec3::new(0, 0, 0),
            max: UVec3::new(shape[0] as u32, shape[1] as u32, shape[2] as u32),
        }
    }

    /// Tests if a position is contained in the region.
    pub fn contains(&self, pos: UVec3) -> bool {
        pos.x >= self.min.x
            && pos.x <= self.max.x
            && pos.y >= self.min.y
            && pos.y <= self.max.y
            && pos.z >= self.min.z
            && pos.z <= self.max.z
    }

    /// Returns an iterator over all positions in the region.
    pub fn iter(&self) -> NavRegionIter {
        NavRegionIter {
            region: *self,
            current: self.min,
        }
    }
}

/// An iterator over all positions in a Region3d.
pub struct NavRegionIter {
    region: NavRegion,
    current: UVec3,
}

impl Iterator for NavRegionIter {
    type Item = UVec3;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current.z > self.region.max.z {
            return None;
        }

        let result = self.current;

        self.current.x += 1;
        if self.current.x > self.region.max.x {
            self.current.x = self.region.min.x;
            self.current.y += 1;

            if self.current.y > self.region.max.y {
                self.current.y = self.region.min.y;
                self.current.z += 1;
            }
        }

        Some(result)
    }
}

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

pub(crate) fn are_adjacent(pos1: UVec3, pos2: UVec3, ordinal: bool) -> bool {
    if pos1 == pos2 {
        return false; // Same position is not adjacent
    }

    let diff = pos1.as_ivec3() - pos2.as_ivec3();
    let abs_diff = diff.abs();

    if ordinal {
        // Ordinal movement: allow diagonal connections
        // Adjacent if all coordinate differences are 0 or 1
        abs_diff.x <= 1 && abs_diff.y <= 1 && abs_diff.z <= 1
    } else {
        // Cardinal movement: only orthogonal connections
        // Adjacent if exactly one coordinate differs by 1, others are 0
        let non_zero_count =
            (abs_diff.x > 0) as u32 + (abs_diff.y > 0) as u32 + (abs_diff.z > 0) as u32;
        non_zero_count == 1 && abs_diff.max_element() == 1
    }
}

pub(crate) fn size_hint_grid<N: neighbor::Neighborhood>(
    neighborhood: &N,
    grid_shape: &[usize],
    start: UVec3,
    goal: UVec3,
) -> usize {
    let manhattan_distance = neighborhood.heuristic(start, goal) as usize;
    let grid_size = grid_shape.iter().product::<usize>();

    if grid_size < 1000 || manhattan_distance < 10 {
        return (manhattan_distance * 4).clamp(64, 512);
    }

    let search_radius = (manhattan_distance as f32 * 0.3) as usize; // 30% of path length
    let estimated_nodes_explored = manhattan_distance * search_radius;

    estimated_nodes_explored
        .max(128)
        .min(grid_size / 4)
        .min(8192)
}

pub(crate) fn size_hint_graph<N: neighbor::Neighborhood>(
    neighborhood: &N,
    graph: &graph::Graph,
    start: UVec3,
    goal: UVec3,
) -> usize {
    let manhattan_distance = neighborhood.heuristic(start, goal) as usize;
    let graph_size = graph.nodes().len();

    (manhattan_distance * 2)
        .max(32)
        .min(graph_size / 2)
        .min(1024)
}
