//! This module defines the `Dir` enum, which represents various directions in 3D space.

use bevy::math::{IVec3, Vec3};
use std::ops::Neg;

pub use self::Dir::*;

/// Enum that represents the 26 directions in 3D space.
#[allow(missing_docs)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Dir {
    NorthWestDown = 0,
    NorthDown = 1,
    NorthEastDown = 2,
    WestDown = 3,
    Down = 4,
    EastDown = 5,
    SouthWestDown = 6,
    SouthDown = 7,
    SouthEastDown = 8,

    NorthWest = 9,
    North = 10,
    NorthEast = 11,
    West = 12,
    East = 13,
    SouthWest = 14,
    South = 15,
    SouthEast = 16,

    NorthWestUp = 17,
    NorthUp = 18,
    NorthEastUp = 19,
    WestUp = 20,
    Up = 21,
    EastUp = 22,
    SouthWestUp = 23,
    SouthUp = 24,
    SouthEastUp = 25,
}

impl Dir {
    /// Returns the offset vector for the direction.
    pub const fn offset(self) -> IVec3 {
        use Dir::*;
        match self {
            NorthWestDown => IVec3::new(-1, 1, -1),
            NorthDown => IVec3::new(0, 1, -1),
            NorthEastDown => IVec3::new(1, 1, -1),
            WestDown => IVec3::new(-1, 0, -1),
            Down => IVec3::new(0, 0, -1),
            EastDown => IVec3::new(1, 0, -1),
            SouthWestDown => IVec3::new(-1, -1, -1),
            SouthDown => IVec3::new(0, -1, -1),
            SouthEastDown => IVec3::new(1, -1, -1),

            NorthWest => IVec3::new(-1, 1, 0),
            North => IVec3::new(0, 1, 0),
            NorthEast => IVec3::new(1, 1, 0),
            West => IVec3::new(-1, 0, 0),
            East => IVec3::new(1, 0, 0),
            SouthWest => IVec3::new(-1, -1, 0),
            South => IVec3::new(0, -1, 0),
            SouthEast => IVec3::new(1, -1, 0),

            NorthWestUp => IVec3::new(-1, 1, 1),
            NorthUp => IVec3::new(0, 1, 1),
            NorthEastUp => IVec3::new(1, 1, 1),
            WestUp => IVec3::new(-1, 0, 1),
            Up => IVec3::new(0, 0, 1),
            EastUp => IVec3::new(1, 0, 1),
            SouthWestUp => IVec3::new(-1, -1, 1),
            SouthUp => IVec3::new(0, -1, 1),
            SouthEastUp => IVec3::new(1, -1, 1),
        }
    }

    /// Return the direction variant from an offset vector.
    pub fn from_offset(offset: IVec3) -> Option<Self> {
        use Dir::*;
        match offset {
            IVec3 { x: -1, y: 1, z: -1 } => Some(NorthWestDown),
            IVec3 { x: 0, y: 1, z: -1 } => Some(NorthDown),
            IVec3 { x: 1, y: 1, z: -1 } => Some(NorthEastDown),
            IVec3 { x: -1, y: 0, z: -1 } => Some(WestDown),
            IVec3 { x: 0, y: 0, z: -1 } => Some(Down),
            IVec3 { x: 1, y: 0, z: -1 } => Some(EastDown),
            IVec3 {
                x: -1,
                y: -1,
                z: -1,
            } => Some(SouthWestDown),
            IVec3 { x: 0, y: -1, z: -1 } => Some(SouthDown),
            IVec3 { x: 1, y: -1, z: -1 } => Some(SouthEastDown),

            IVec3 { x: -1, y: 1, z: 0 } => Some(NorthWest),
            IVec3 { x: 0, y: 1, z: 0 } => Some(North),
            IVec3 { x: 1, y: 1, z: 0 } => Some(NorthEast),
            IVec3 { x: -1, y: 0, z: 0 } => Some(West),
            IVec3 { x: 1, y: 0, z: 0 } => Some(East),
            IVec3 { x: -1, y: -1, z: 0 } => Some(SouthWest),
            IVec3 { x: 0, y: -1, z: 0 } => Some(South),
            IVec3 { x: 1, y: -1, z: 0 } => Some(SouthEast),

            IVec3 { x: -1, y: 1, z: 1 } => Some(NorthWestUp),
            IVec3 { x: 0, y: 1, z: 1 } => Some(NorthUp),
            IVec3 { x: 1, y: 1, z: 1 } => Some(NorthEastUp),
            IVec3 { x: -1, y: 0, z: 1 } => Some(WestUp),
            IVec3 { x: 0, y: 0, z: 1 } => Some(Up),
            IVec3 { x: 1, y: 0, z: 1 } => Some(EastUp),
            IVec3 { x: -1, y: -1, z: 1 } => Some(SouthWestUp),
            IVec3 { x: 0, y: -1, z: 1 } => Some(SouthUp),
            IVec3 { x: 1, y: -1, z: 1 } => Some(SouthEastUp),
            _ => None,
        }
    }

    /// All direction variants as an iterator.
    pub fn all() -> impl Iterator<Item = Dir> {
        (0..26).filter_map(Dir::from_bit_index)
    }

    /// Returns an iterator of only the cardinal faces in a 3d cube.
    pub fn cardinal_faces() -> impl Iterator<Item = Dir> {
        [North, East, South, West, Up, Down].into_iter()
    }

    /// Returns an iterator of only the cardinal edges in a 3d cube.
    pub fn cardinal_edges() -> impl Iterator<Item = Dir> {
        [
            NorthUp, EastUp, SouthUp, WestUp, NorthDown, EastDown, SouthDown, WestDown,
        ]
        .into_iter()
    }

    /// Returns an iterator of all the cardinal directions in a 3d cube.
    pub fn cardinal() -> impl Iterator<Item = Dir> {
        [
            North, East, South, West, Up, Down, NorthUp, NorthDown, SouthUp, SouthDown, WestUp,
            WestDown, EastUp, EastDown,
        ]
        .into_iter()
    }

    /// Returns an iterator of all the diagonal directions in a 3d cube.
    pub fn ordinal() -> impl Iterator<Item = Dir> {
        [
            NorthEast,
            SouthEast,
            SouthWest,
            NorthWest,
            NorthEastUp,
            SouthEastUp,
            SouthWestUp,
            NorthWestUp,
            NorthEastDown,
            SouthEastDown,
            SouthWestDown,
            NorthWestDown,
        ]
        .into_iter()
    }

    /// Enum bit index -> `Dir` mapping.
    pub(crate) fn from_bit_index(index: usize) -> Option<Self> {
        use Dir::*;
        match index {
            0 => Some(NorthWestDown),
            1 => Some(NorthDown),
            2 => Some(NorthEastDown),
            3 => Some(WestDown),
            4 => Some(Down),
            5 => Some(EastDown),
            6 => Some(SouthWestDown),
            7 => Some(SouthDown),
            8 => Some(SouthEastDown),
            9 => Some(NorthWest),
            10 => Some(North),
            11 => Some(NorthEast),
            12 => Some(West),
            13 => Some(East),
            14 => Some(SouthWest),
            15 => Some(South),
            16 => Some(SouthEast),
            17 => Some(NorthWestUp),
            18 => Some(NorthUp),
            19 => Some(NorthEastUp),
            20 => Some(WestUp),
            21 => Some(Up),
            22 => Some(EastUp),
            23 => Some(SouthWestUp),
            24 => Some(SouthUp),
            25 => Some(SouthEastUp),
            _ => None,
        }
    }

    pub(crate) fn from_bits(bits: u32) -> Vec<Self> {
        (0..26)
            .filter(|&i| (bits & (1 << i)) != 0)
            .filter_map(Dir::from_bit_index)
            .collect()
    }

    /// Returns the direction from a start position to an end position.
    pub fn dir_to(start: &Vec3, end: &Vec3) -> Self {
        let delta = (*end - *start).normalize_or_zero();
        let vec = IVec3::new(
            delta.x.round() as i32,
            delta.y.round() as i32,
            delta.z.round() as i32,
        );
        Dir::from_offset(vec).expect("Not a valid direction")
    }

    /// Returns the opposite direction.
    pub fn opposite(self) -> Dir {
        self.offset()
            .neg()
            .try_into()
            .expect("Invalid opposite direction")
    }

    /// Whether the dirction changes the z coordinate.
    pub fn is_vertical(self) -> bool {
        self.offset().z != 0
    }
}

impl TryFrom<IVec3> for Dir {
    type Error = (i32, i32, i32);

    fn try_from(value: IVec3) -> Result<Self, Self::Error> {
        Dir::from_offset(value).ok_or((value.x, value.y, value.z))
    }
}
