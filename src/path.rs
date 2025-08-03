//! This module defines the important `Path` component.
use bevy::math::UVec3;
use bevy::prelude::Component;
use bevy::reflect::Reflect;
use std::collections::VecDeque;

/// The path struct and component containing the path result of a pathfinding operation.
///
/// This is returned by pathfinding functions.
/// If using [`crate::plugin::NorthstarPlugin`] this is inserted as a component on an entity after the plugin
/// systems have pathfound to the goal position.
///
#[derive(Debug, Clone, Component, Reflect)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Path {
    pub(crate) path: VecDeque<UVec3>,
    pub(crate) graph_path: VecDeque<UVec3>,
    cost: u32,
    is_reversed: bool,
}

impl Path {
    /// Create a new path from a vector of `UVec3` positions
    /// # Arguments
    /// * `path` - A vector of `UVec3` positions
    /// * `total_cost` - The total movement cost of the path
    ///
    pub fn new(path: Vec<UVec3>, cost: u32) -> Self {
        let path = path.into_iter().collect();

        Path {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
        }
    }

    /// Create a new path from a slice of `UVec3` positions
    /// # Arguments
    /// * `path` - A slice of `UVec3` positions
    /// * `total_cost` - The total movement cost of the path
    ///
    pub fn from_slice(path: &[UVec3], cost: u32) -> Self {
        let path = path.iter().cloned().collect();

        Path {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
        }
    }

    /// Returns true if the path contains the given position
    pub fn is_position_in_path(&self, pos: UVec3) -> bool {
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
    pub fn path(&self) -> &[UVec3] {
        self.path.as_slices().0
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
    pub fn pop(&mut self) -> Option<UVec3> {
        // Remove the first element of the path
        self.path.pop_front()
    }

    /// Returns the next position in the path without removing it.
    pub fn next(&self) -> Option<UVec3> {
        // Get the next position in the path
        self.path.front().cloned()
    }

    /// Shifts all positions in the path by the given offset.
    pub(crate) fn translate_by(&mut self, offset: UVec3) {
        for pos in &mut self.path {
            *pos += offset;
        }
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
    type Item = UVec3;
    type IntoIter = std::collections::vec_deque::IntoIter<UVec3>;

    fn into_iter(self) -> Self::IntoIter {
        self.path.into_iter()
    }
}
