//! This module defines the `Dir` enum, which represents various directions in 3D space.
use bevy::math::{IVec3, Vec3};

pub use self::Dir::*;

/// Enum that represents the 6 cardinal directions and 12 ordinal directions in 3D space.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    UP = 4,
    DOWN = 5,
    NORTHEAST = 6,
    SOUTHEAST = 7,
    SOUTHWEST = 8,
    NORTHWEST = 9,
    NORTHEASTUP = 10,
    SOUTHEASTUP = 11,
    SOUTHWESTUP = 12,
    NORTHWESTUP = 13,
    NORTHEASTDOWN = 14,
    SOUTHEASTDOWN = 15,
    SOUTHWESTDOWN = 16,
    NORTHWESTDOWN = 17,
}

impl Dir {
    /// Returns an iterator over the cardinal directions.
    /// Cardinal directions are the 6 main directions: North, East, South, West, Up, Down.
    pub fn cardinal() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [NORTH, EAST, SOUTH, WEST, UP, DOWN].iter().copied()
    }

    /// Returns an iterator over the ordinal directions.
    /// Ordinal directions are the 12 diagonal directions:
    /// NE, SE, SW, NW, NEU, SEU, SWU, NWU, NED, SED, SWD, NWD.
    pub fn ordinal() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [
            NORTHEAST,
            SOUTHEAST,
            SOUTHWEST,
            NORTHWEST,
            NORTHEASTUP,
            SOUTHEASTUP,
            SOUTHWESTUP,
            NORTHWESTUP,
            NORTHEASTDOWN,
            SOUTHEASTDOWN,
            SOUTHWESTDOWN,
            NORTHWESTDOWN,
        ]
        .iter()
        .copied()
    }

    /// Returns an iterator over all directions.
    /// This includes both cardinal and ordinal directions.
    pub fn all() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [
            NORTH,
            EAST,
            SOUTH,
            WEST,
            UP,
            DOWN,
            NORTHEAST,
            SOUTHEAST,
            SOUTHWEST,
            NORTHWEST,
            NORTHEASTUP,
            SOUTHEASTUP,
            SOUTHWESTUP,
            NORTHWESTUP,
            NORTHEASTDOWN,
            SOUTHEASTDOWN,
            SOUTHWESTDOWN,
            NORTHWESTDOWN,
        ]
        .iter()
        .copied()
    }

    /// Returns a tuple vector representation of the direction.
    pub fn vector(self) -> (i32, i32, i32) {
        match self {
            NORTH => (0, 1, 0),
            EAST => (1, 0, 0),
            SOUTH => (0, -1, 0),
            WEST => (-1, 0, 0),
            UP => (0, 0, 1),
            DOWN => (0, 0, -1),
            NORTHEAST => (1, 1, 0),
            SOUTHEAST => (1, -1, 0),
            SOUTHWEST => (-1, -1, 0),
            NORTHWEST => (-1, 1, 0),
            NORTHEASTUP => (1, 1, 1),
            SOUTHEASTUP => (1, -1, 1),
            SOUTHWESTUP => (-1, -1, 1),
            NORTHWESTUP => (-1, 1, 1),
            NORTHEASTDOWN => (1, 1, -1),
            SOUTHEASTDOWN => (1, -1, -1),
            SOUTHWESTDOWN => (-1, -1, -1),
            NORTHWESTDOWN => (-1, 1, -1),
        }
    }

    /// Returns the `Dir` from a start to an end position.
    pub fn dir_to(start: &Vec3, end: &Vec3) -> Self {
        let delta = (end - start).normalize_or_zero();

        let vec = IVec3::new(
            delta.x.round() as i32,
            delta.y.round() as i32,
            delta.z.round() as i32,
        );

        match vec {
            IVec3 { x: 0, y: 1, z: 0 } => NORTH,
            IVec3 { x: 1, y: 0, z: 0 } => EAST,
            IVec3 { x: 0, y: -1, z: 0 } => SOUTH,
            IVec3 { x: -1, y: 0, z: 0 } => WEST,
            IVec3 { x: 0, y: 0, z: 1 } => UP,
            IVec3 { x: 0, y: 0, z: -1 } => DOWN,
            IVec3 { x: 1, y: 1, z: 0 } => NORTHEAST,
            IVec3 { x: 1, y: -1, z: 0 } => SOUTHEAST,
            IVec3 { x: -1, y: -1, z: 0 } => SOUTHWEST,
            IVec3 { x: -1, y: 1, z: 0 } => NORTHWEST,
            IVec3 { x: 1, y: 1, z: 1 } => NORTHEASTUP,
            IVec3 { x: 1, y: -1, z: 1 } => SOUTHEASTUP,
            IVec3 { x: -1, y: -1, z: 1 } => SOUTHWESTUP,
            IVec3 { x: -1, y: 1, z: 1 } => NORTHWESTUP,
            IVec3 { x: 1, y: 1, z: -1 } => NORTHEASTDOWN,
            IVec3 { x: 1, y: -1, z: -1 } => SOUTHEASTDOWN,
            IVec3 {
                x: -1,
                y: -1,
                z: -1,
            } => SOUTHWESTDOWN,
            IVec3 { x: -1, y: 1, z: -1 } => NORTHWESTDOWN,
            _ => panic!("Not a valid direction"),
        }
    }

    /// Returns the opposite direction.
    pub fn opposite(self) -> Dir {
        match self {
            NORTH => SOUTH,
            EAST => WEST,
            SOUTH => NORTH,
            WEST => EAST,
            UP => DOWN,
            DOWN => UP,
            NORTHEAST => SOUTHWEST,
            SOUTHEAST => NORTHWEST,
            SOUTHWEST => NORTHEAST,
            NORTHWEST => SOUTHEAST,
            NORTHEASTUP => SOUTHWESTDOWN,
            SOUTHEASTUP => NORTHWESTDOWN,
            SOUTHWESTUP => NORTHEASTDOWN,
            NORTHWESTUP => SOUTHEASTDOWN,
            NORTHEASTDOWN => SOUTHWESTUP,
            SOUTHEASTDOWN => NORTHWESTUP,
            SOUTHWESTDOWN => NORTHEASTUP,
            NORTHWESTDOWN => SOUTHEASTUP,
        }
    }

    /// Returns true if the direction is up or down.
    pub fn is_verticle(self) -> bool {
        matches!(
            self,
            UP | DOWN
                | NORTHEASTUP
                | SOUTHEASTUP
                | SOUTHWESTUP
                | NORTHWESTUP
                | NORTHEASTDOWN
                | SOUTHEASTDOWN
                | SOUTHWESTDOWN
                | NORTHWESTDOWN
        )
    }
}
