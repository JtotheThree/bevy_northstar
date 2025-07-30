//! Navigation Masks for 
use bevy::{math::UVec3, platform::collections::HashMap};
use ndarray::Array3;

use crate::nav::{Nav, NavCell};

pub trait NavMask {
    fn region(&self) -> Option<Region3D>;
    fn mask(&self, pos: UVec3, prev_pos: UVec3) -> Option<NavCell>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region3D {
    pub min: UVec3,
    pub max: UVec3,
}

#[derive(Debug, Clone)]
pub struct HashMapNavMask {
    overrides: HashMap<UVec3, NavCell>,
}

impl HashMapNavMask {
    pub fn new() -> Self {
        HashMapNavMask {
            overrides: HashMap::new(),
        }
    }

    pub fn insert(&mut self, pos: UVec3, cell: NavCell) {
        self.overrides.insert(pos, cell);
    }

    pub fn remove(&mut self, pos: UVec3) {
        self.overrides.remove(&pos);
    }
}

impl NavMask for HashMapNavMask {
    fn region(&self) -

    fn mask(&self, pos: UVec3, _prev_pos: UVec3) -> Option<NavCell> {
        self.overrides.get(&pos).cloned()
    }
}

#[derive(Debug, Clone)]
pub struct RegionNavMask {
    region: Region3D,
    mask: Array3<NavCell>,
}

impl RegionNavMask {
    pub fn new(region: Region3D) -> Self {
        let size = region.max - region.min + UVec3::ONE;
        let mask = Array3::default((size.x as usize, size.y as usize, size.z as usize));
        RegionNavMask { region, mask }
    }

    pub fn set_cell(&mut self, pos: UVec3, cell: NavCell) {
        let idx = pos - self.region.min;
        self.mask[[idx.x as usize, idx.y as usize, idx.z as usize]] = cell;
    }

    pub fn get_cell(&self, pos: UVec3) -> Option<&NavCell> {
        let idx = pos - self.region.min;
        if idx.x < self.mask.shape()[0] as u32 && idx.y < self.mask.shape()[1] as u32 && idx.z < self.mask.shape()[2] as u32 {
            Some(&self.mask[[idx.x as usize, idx.y as usize, idx.z as usize]])
        } else {
            None
        }
    }
}

impl NavMask for RegionNavMask {
    fn mask(&self, pos: UVec3, _prev_pos: UVec3) -> Option<NavCell> {
        self.get_cell(pos).cloned()
    }
}