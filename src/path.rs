//! This module defines the important `Path` component.
use bevy::gizmos::grid;
use bevy::math::{IVec3, UVec3};
use bevy::prelude::Component;
use bevy::reflect::Reflect;
use std::collections::VecDeque;

use crate::grid::Grid;
use crate::neighbor::Neighborhood;

/// The path struct and component containing the path result of a pathfinding operation.
///
/// This is returned by pathfinding functions.
/// If using [`crate::plugin::NorthstarPlugin`] this is inserted as a component on an entity after the plugin
/// systems have pathfound to the goal position.
///
/// Example usage:
/// ```rust
/// use bevy::prelude::*;
/// use bevy_northstar::prelude::*;
///
/// fn example_path(grid: Single<&Grid<CardinalNeighborhood>>) {
///    let grid = grid.into_inner();
///
///    let start = IVec3::new(0, 0, 0);
///    let goal = IVec3::new(10, 0, 10);
///
///    let path = grid.pathfind(&mut PathfindArgs::new(start, goal)).unwrap();
///    assert_eq!(path.cost(), 20);
/// }
/// ```
#[derive(Debug, Clone, Component, Reflect)]
pub struct Path {
    pub(crate) path: VecDeque<IVec3>,
    pub(crate) graph_path: VecDeque<IVec3>,
    cost: u32,
    is_reversed: bool,
    is_partial: bool,
}

impl Path {
    /// Create a new path from a vector of `IVec3` positions
    /// # Arguments
    /// * `path` - A vector of `IVec3` positions
    /// * `total_cost` - The total movement cost of the path
    ///
    pub fn new(path: Vec<IVec3>, cost: u32) -> Self {
        let path = path.into_iter().collect();

        Self {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
            is_partial: false,
        }
    }

    /// Create a new path from a slice of `IVec3` positions
    /// # Arguments
    /// * `path` - A slice of `IVec3` positions
    /// * `total_cost` - The total movement cost of the path
    ///
    pub fn from_slice(path: &[IVec3], cost: u32) -> Self {
        let path = path.iter().cloned().collect();

        Self {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
            is_partial: false,
        }
    }

    /// Returns true if the path contains the given position
    pub fn is_position_in_path(&self, pos: IVec3) -> bool {
        self.path.contains(&pos)
    }

    /// Returns the path as a slice of `IVec3` positions.
    /// Useful to represent the path for UI etc.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use bevy::prelude::*;
    /// use bevy_northstar::prelude::*;
    ///
    /// let path = Path::new(vec![IVec3::new(1, 2, 3), IVec3::new(4, 5, 6)], 10);
    /// assert_eq!(path.path(), &[IVec3::new(1, 2, 3), IVec3::new(4, 5, 6)]);
    /// ```
    pub fn path(&self) -> &[IVec3] {
        self.path.as_slices().0
    }

    /// Returns a mutable slice of the path positions.
    /// You can manually alter the path using this slice.
    pub fn as_mut_slices(&mut self) -> &mut [IVec3] {
        self.path.make_contiguous();
        self.path.as_mut_slices().0
    }

    /// Returns the HPA* high level graph path as a slice of `IVec3` positions.
    pub fn graph_path(&self) -> &[IVec3] {
        self.graph_path.as_slices().0
    }

    /// Returns the movement cost of the path
    pub fn cost(&self) -> u32 {
        self.cost
    }

    /// Returns the length of the path
    pub fn len(&self) -> usize {
        self.path.len()
    }

    /// Returns true if the path is empty
    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    /// Reverse the path in place.
    pub fn reverse(&mut self) {
        self.path.make_contiguous().reverse();
        self.is_reversed = !self.is_reversed;
    }

    /// Pops the first position of the path.
    pub fn pop(&mut self) -> Option<IVec3> {
        // Remove the first element of the path
        self.path.pop_front()
    }

    /// Returns the next position in the path without removing it.
    pub fn next(&self) -> Option<IVec3> {
        // Get the next position in the path
        self.path.front().cloned()
    }

    /// Shifts all positions in the path by the given offset.
    pub(crate) fn translate_by(&mut self, offset: IVec3) {
        for pos in &mut self.path {
            *pos += offset;
        }
    }

    /// Returns true if the path is a partial path that was returned because the goal could not be reached.
    pub fn is_partial(&self) -> bool {
        self.is_partial
    }

    pub(crate) fn set_partial(&mut self, is_partial: bool) {
        self.is_partial = is_partial;
    }
}

impl PartialEq for Path {
    fn eq(&self, other: &Self) -> bool {
        self.path == other.path
    }
}

impl Eq for Path {}

// Implement iter for Path
impl IntoIterator for Path {
    type Item = IVec3;
    type IntoIter = std::collections::vec_deque::IntoIter<IVec3>;

    fn into_iter(self) -> Self::IntoIter {
        self.path.into_iter()
    }
}

#[derive(Debug, Clone, Component, Reflect)]
pub(crate) struct PathLocal {
    pub(crate) path: VecDeque<UVec3>,
    pub(crate) graph_path: VecDeque<UVec3>,
    cost: u32,
    is_reversed: bool,
    is_partial: bool,
}

impl PathLocal {

    /// Create a new path from a vector of `UVec3` positions
    /// # Arguments
    /// * `path` - A vector of `UVec3` positions
    /// * `total_cost` - The total movement cost of the path
    ///
    pub(crate) fn new(path: Vec<UVec3>, cost: u32) -> Self {
        let path = path.into_iter().collect();

        Self {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
            is_partial: false,
        }
    }

    /// Create a new path from a slice of `UVec3` positions
    /// # Arguments
    /// * `path` - A slice of `UVec3` positions
    /// * `total_cost` - The total movement cost of the path
    ///
    pub(crate) fn from_slice(path: &[UVec3], cost: u32) -> Self {
        let path = path.iter().cloned().collect();

        Self {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
            is_partial: false,
        }
    }

    /// Returns true if the path contains the given position
    pub(crate) fn is_position_in_path(&self, pos: UVec3) -> bool {
        self.path.contains(&pos)
    }

    /// Returns the path as a slice of `UVec3` positions.
    /// Useful to represent the path for UI etc.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use bevy::prelude::*;
    /// use bevy_northstar::prelude::*;
    ///
    /// let path = Path::new(vec![UVec3::new(1, 2, 3), UVec3::new(4, 5, 6)], 10);
    /// assert_eq!(path.path(), &[UVec3::new(1, 2, 3), UVec3::new(4, 5, 6)]);
    /// ```
    pub(crate) fn path(&self) -> &[UVec3] {
        self.path.as_slices().0
    }

    /// Returns a mutable slice of the path positions.
    /// You can manually alter the path using this slice.
    pub(crate) fn as_mut_slices(&mut self) -> &mut [UVec3] {
        self.path.make_contiguous();
        self.path.as_mut_slices().0
    }

    /// Returns the HPA* high level graph path as a slice of `UVec3` positions.
    pub(crate) fn graph_path(&self) -> &[UVec3] {
        self.graph_path.as_slices().0
    }

    /// Returns the movement cost of the path
    pub(crate) fn cost(&self) -> u32 {
        self.cost
    }

    /// Returns the length of the path
    pub(crate) fn len(&self) -> usize {
        self.path.len()
    }

    /// Returns true if the path is empty
    pub(crate) fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    /// Reverse the path in place.
    pub(crate) fn reverse(&mut self) {
        self.path.make_contiguous().reverse();
        self.is_reversed = !self.is_reversed;
    }

    /// Pops the first position of the path.
    pub(crate) fn pop(&mut self) -> Option<UVec3> {
        // Remove the first element of the path
        self.path.pop_front()
    }

    /// Returns the next position in the path without removing it.
    pub(crate) fn next(&self) -> Option<UVec3> {
        // Get the next position in the path
        self.path.front().cloned()
    }

    /// Shifts all positions in the path by the given offset.
    pub(crate) fn translate_by(&mut self, offset: UVec3) {
        for pos in &mut self.path {
            *pos += offset;
        }
    }

    /// Returns true if the path is a partial path that was returned because the goal could not be reached.
    pub(crate) fn is_partial(&self) -> bool {
        self.is_partial
    }

    pub(crate) fn set_partial(&mut self, is_partial: bool) {
        self.is_partial = is_partial;
    }
}

impl PartialEq for PathLocal {
    fn eq(&self, other: &Self) -> bool {
        self.path == other.path
    }
}

impl Eq for PathLocal {}

// Implement iter for Path
impl IntoIterator for PathLocal {
    type Item = UVec3;
    type IntoIter = std::collections::vec_deque::IntoIter<UVec3>;

    fn into_iter(self) -> Self::IntoIter {
        self.path.into_iter()
    }
}


pub(crate) fn path_to_local<N: Neighborhood + 'static>(
    grid: &Grid<N>, 
    path: &Path
) -> Option<PathLocal> {
    for pos in path.path() {
        let local_pos = grid.world_to_local(*pos);
        if local_pos.is_none() {
            return None;
        }
    }
    Some(PathLocal::new(
        path.path().iter().map(|p| grid.world_to_local(*p).unwrap()).collect(),
        path.cost()
    ))
}

pub(crate) fn path_to_world<N: Neighborhood + 'static>(
    grid: &Grid<N>, 
    path: &PathLocal
) -> Path {
    let world_path = path.path().iter().map(|p| grid.local_to_world(*p)).collect();
    Path::new(world_path, path.cost())
}