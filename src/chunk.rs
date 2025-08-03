//! This module defines the `Chunk` struct, which represents a 3D region of the grid.
use bevy::math::UVec3;
use ndarray::{s, Array3, ArrayView1, ArrayView2, ArrayView3};

use crate::{dir::Dir, nav::NavCell};

/// A chunk is a 3D region of the grid.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Chunk {
    /// The index of the chunk in the grid.
    index: (usize, usize, usize),
    /// The minimum coordinates of the chunk.
    min: UVec3,
    /// The maximum coordinates of the chunk.
    max: UVec3,
    /// Flags which indicate which edges are dirty.
    dirty_edges: [bool; 26],
}

impl PartialEq for Chunk {
    fn eq(&self, other: &Self) -> bool {
        self.min == other.min && self.max == other.max
    }
}

impl Eq for Chunk {}

impl Chunk {
    /// Creates a new chunk from minimum and maximum coordinates.
    pub(crate) fn new(index: (usize, usize, usize), min: UVec3, max: UVec3) -> Self {
        Chunk {
            index,
            min,
            max,
            dirty_edges: [true; 26],
        }
    }

    pub(crate) fn index(&self) -> (usize, usize, usize) {
        self.index
    }

    pub(crate) fn min(&self) -> UVec3 {
        self.min
    }

    pub(crate) fn max(&self) -> UVec3 {
        self.max
    }

    pub(crate) fn bounds(&self) -> impl Iterator<Item = UVec3> {
        let min = self.min();
        let max = self.max();

        (min.x..max.x).flat_map(move |x| {
            (min.y..max.y).flat_map(move |y| (min.z..max.z).map(move |z| UVec3::new(x, y, z)))
        })
    }

    pub(crate) fn has_dirty_edges(&self) -> bool {
        self.dirty_edges.iter().any(|&edge| edge)
    }

    pub(crate) fn is_edge_dirty(&self, dir: Dir) -> bool {
        self.dirty_edges[dir as usize]
    }

    pub(crate) fn set_dirty_edge(&mut self, dir: Dir, dirty: bool) {
        self.dirty_edges[dir as usize] = dirty;
    }

    #[allow(dead_code)]
    pub(crate) fn set_all_edges_dirty(&mut self, dirty: bool) {
        for edge in self.dirty_edges.iter_mut() {
            *edge = dirty;
        }
    }

    pub(crate) fn clean(&mut self) {
        self.dirty_edges.fill(false);
    }

    // Adjusts a position to the local coordinates of the chunk.
    pub(crate) fn to_local(&self, pos: &UVec3) -> UVec3 {
        UVec3::new(
            pos.x.saturating_sub(self.min().x),
            pos.y.saturating_sub(self.min().y),
            pos.z.saturating_sub(self.min().z),
        )
    }

    /// Returns a 3D `ArrayView3`` of `NavCell`s of the chunk from the grid.
    pub(crate) fn view<'a>(&self, grid: &'a Array3<NavCell>) -> ArrayView3<'a, NavCell> {
        grid.slice(s![
            self.min.x as usize..self.max.x as usize,
            self.min.y as usize..self.max.y as usize,
            self.min.z as usize..self.max.z as usize
        ])
    }

    pub(crate) fn face<'a>(&self, grid: &'a Array3<NavCell>, dir: Dir) -> ArrayView2<'a, NavCell> {
        match dir {
            Dir::North => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::East => grid.slice(s![
                self.max.x as usize - 1,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::South => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::West => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::Up => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.max.z as usize - 1,
            ]),
            Dir::Down => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize,
            ]),
            _ => panic!("{dir:?} does not correspond to a face."),
        }
    }

    pub(crate) fn edge<'a>(&self, grid: &'a Array3<NavCell>, dir: Dir) -> ArrayView1<'a, NavCell> {
        match dir {
            Dir::NorthUp => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize - 1,
                self.max.z as usize - 1
            ]),
            Dir::EastUp => grid.slice(s![
                self.max.x as usize - 1,
                self.min.y as usize..self.max.y as usize,
                self.max.z as usize - 1
            ]),
            Dir::SouthUp => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize,
                self.max.z as usize - 1
            ]),
            Dir::WestUp => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.max.z as usize - 1
            ]),
            Dir::NorthDown => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize
            ]),
            Dir::EastDown => grid.slice(s![
                self.max.x as usize - 1,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize
            ]),
            Dir::SouthDown => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize,
                self.min.z as usize
            ]),
            Dir::WestDown => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize
            ]),
            _ => panic!("{dir:?} does not correspond to an edge."),
        }
    }

    /// Returns the chunk corner `NavCell` for the given ordinal direction.
    pub(crate) fn corner(&self, grid: &Array3<NavCell>, dir: Dir) -> NavCell {
        match dir {
            Dir::NorthEast => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthEast => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthWest => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NorthWest => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NorthEastUp => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::SouthEastUp => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::SouthWestUp => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::NorthWestUp => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::NorthEastDown => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthEastDown => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthWestDown => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NorthWestDown => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            _ => panic!("{dir:?} does not correspond to a corner."),
        }
    }

    pub(crate) fn touching_edges(&self, pos: UVec3) -> impl Iterator<Item = Dir> + '_ {
        Dir::all().filter(move |&dir| {
            let offset = dir.offset();
            let (dx, dy, dz) = (offset.x, offset.y, offset.z);

            let x_match = match dx {
                -1 => pos.x == self.min().x,
                1 => pos.x + 1 == self.max().x,
                _ => true, // direction doesn't check x
            };

            let y_match = match dy {
                -1 => pos.y == self.min().y,
                1 => pos.y + 1 == self.max().y,
                _ => true,
            };

            let z_match = match dz {
                -1 => pos.z == self.min().z,
                1 => pos.z + 1 == self.max().z,
                _ => true,
            };

            x_match && y_match && z_match
        })
    }

    pub(crate) fn boundary_pos_to_global(&self, pos: UVec3, dir: Dir) -> UVec3 {
        match dir {
            Dir::North => UVec3::new(pos.x + self.min().x, self.max().y - 1, pos.y + self.min().z),
            Dir::South => UVec3::new(pos.x + self.min().x, self.min().y, pos.y + self.min().z),
            Dir::East => UVec3::new(self.max().x - 1, pos.x + self.min().y, pos.y + self.min().z),
            Dir::West => UVec3::new(self.min().x, pos.x + self.min().y, pos.y + self.min().z),
            Dir::Up => UVec3::new(pos.x + self.min().x, pos.y + self.min().y, self.max().z - 1),
            Dir::Down => UVec3::new(pos.x + self.min().x, pos.y + self.min().y, self.min().z),
            Dir::NorthUp => UVec3::new(pos.x + self.min().x, self.max().y - 1, self.max().z - 1),
            Dir::EastUp => UVec3::new(self.max().x - 1, pos.x + self.min().y, self.max().z - 1),
            Dir::SouthUp => UVec3::new(pos.x + self.min().x, self.min().y, self.max().z - 1),
            Dir::WestUp => UVec3::new(self.min().x, pos.x + self.min().y, self.max().z - 1),
            Dir::NorthDown => UVec3::new(pos.x + self.min().x, self.max().y - 1, self.min().z),
            Dir::EastDown => UVec3::new(self.max().x - 1, pos.x + self.min().y, self.min().z),
            Dir::SouthDown => UVec3::new(pos.x + self.min().x, self.min().y, self.min().z),
            Dir::WestDown => UVec3::new(self.min().x, pos.x + self.min().y, self.min().z),
            _ => panic!("{dir:?} is a corner. This function only supports edges and faces."),
        }
    }

    pub fn corner_pos(&self, dir: Dir) -> UVec3 {
        match dir {
            Dir::NorthEast => UVec3::new(self.max().x - 1, self.max().y - 1, self.min().z),
            Dir::SouthEast => UVec3::new(self.max().x - 1, self.min().y, self.min().z),
            Dir::SouthWest => UVec3::new(self.min().x, self.min().y, self.min().z),
            Dir::NorthWest => UVec3::new(self.min().x, self.max().y - 1, self.min().z),
            Dir::NorthEastUp => UVec3::new(self.max().x - 1, self.max().y - 1, self.max().z - 1),
            Dir::SouthEastUp => UVec3::new(self.max().x - 1, self.min().y, self.max().z - 1),
            Dir::SouthWestUp => UVec3::new(self.min().x, self.min().y, self.max().z - 1),
            Dir::NorthWestUp => UVec3::new(self.min().x, self.max().y - 1, self.max().z - 1),
            Dir::NorthEastDown => UVec3::new(self.max().x - 1, self.max().y - 1, self.min().z),
            Dir::SouthEastDown => UVec3::new(self.max().x - 1, self.min().y, self.min().z),
            Dir::SouthWestDown => UVec3::new(self.min().x, self.min().y, self.min().z),
            Dir::NorthWestDown => UVec3::new(self.min().x, self.max().y - 1, self.min().z),
            _ => {
                panic!("{dir:?} is not a corner. This function only supports corner directions.");
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_chunk_view() {
        let grid = Array3::from_elem((10, 10, 10), NavCell::default());
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let view = chunk.view(&grid);

        assert_eq!(view.shape(), [4, 4, 4]);
    }

    #[test]
    fn test_chunk_edge() {
        let grid = Array3::from_elem((10, 10, 10), NavCell::default());
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let edge = chunk.face(&grid, Dir::North);

        assert_eq!(edge.shape(), [4, 4]);
    }

    #[test]
    fn test_touching_edges() {
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let pos = UVec3::new(3, 2, 0);

        let edges: Vec<Dir> = chunk.touching_edges(pos).collect();
        assert!(!edges.contains(&Dir::West));
        assert!(!edges.contains(&Dir::North));
        assert!(edges.contains(&Dir::East));
        assert!(!edges.contains(&Dir::South));
    }
}
