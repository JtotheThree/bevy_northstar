//! Navigation Masks for 
use std::sync::Arc;

use bevy::{math::UVec3, platform::collections::HashMap};
use ndarray::ArrayView3;

use crate::{nav::{Nav, NavCell, Portal}, MovementCost};


#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum NavCellMask {
    ImpassableOverride,
    PassableOverride(MovementCost),
    PortalOverride(Portal),
    ModifyCost(i32),
}

pub trait NavMask: Send + Sync + 'static {
    fn get(&self, pos: UVec3) -> Option<&NavCellMask>;
}

fn process_mask(
    cell: &mut NavCell,
    mask: &NavCellMask,
) {
    match mask {
        NavCellMask::ImpassableOverride => {
            cell.nav = Nav::Impassable;
        }
        NavCellMask::PassableOverride(cost) => {
            cell.nav = Nav::Passable(*cost);
            cell.cost = *cost;
        }
        NavCellMask::PortalOverride(portal) => {
            cell.nav = Nav::Portal(*portal);
        }
        NavCellMask::ModifyCost(delta) => {
            if let Nav::Passable(cost) = cell.nav {
                let new_cost = cost.saturating_add(*delta as MovementCost);
                cell.nav = Nav::Passable(new_cost);
                cell.cost = new_cost;
            }
        }
    }
}

#[derive(Debug)]
pub struct RegionNavMask {
    region: Region3d,
    masks: HashMap<UVec3, NavCellMask>,
}

impl RegionNavMask {
    pub fn new(region: Region3d) -> Self {
        Self {
            region,
            masks: HashMap::new(),
        }
    }

    pub fn apply_mask(&mut self, pos: UVec3, mask: NavCellMask) {
        if self.region.iter().any(|p| p == pos) {
            self.masks.insert(pos, mask);
        }
    }

    pub fn clear(&mut self) {
        self.masks.clear();
    }

    pub fn in_bounds(&self, pos: UVec3) -> bool {
        self.region.min.x <= pos.x && pos.x < self.region.max.x &&
        self.region.min.y <= pos.y && pos.y < self.region.max.y &&
        self.region.min.z <= pos.z && pos.z < self.region.max.z
    }
}

impl NavMask for RegionNavMask {
    fn get(&self, pos: UVec3) -> Option<&NavCellMask> {
        if !self.in_bounds(pos) {
            return None;
        }

        self.masks.get(&pos)
    }
}

#[derive(Debug, Default)]
pub struct HashMapNavMask {
    masks: HashMap<UVec3, NavCellMask>,
}

impl HashMapNavMask {
    pub fn new() -> Self {
        Self {
            masks: HashMap::new(),
        }
    }

    pub fn insert_mask(&mut self, pos: UVec3, mask: NavCellMask) {
        self.masks.insert(pos, mask);
    }

    pub fn clear(&mut self) {
        self.masks.clear();
    }
}

impl NavMask for HashMapNavMask {
    fn get(&self, pos: UVec3) -> Option<&NavCellMask> {
        self.masks.get(&pos)
    }
}

pub struct NavMasks {
    masks: Vec<Arc<dyn NavMask + Send + Sync>>
}

impl NavMasks {
    pub fn new() -> Self {
        Self { masks: Vec::new() }
    }

    pub fn from_mask<M: NavMask + 'static>(mask: M) -> Self {
        Self {
            masks: vec![Arc::new(mask)],
        }
    }

    pub fn from_masks<M: NavMask + 'static>(masks: Vec<M>) -> Self {
        Self {
            masks: masks.into_iter().map(|m| Arc::new(m) as Arc<dyn NavMask + Send + Sync>).collect(),
        }
    }

    pub fn with_other<M: NavMask + 'static>(masks: &Self, mask: M) -> Self {
        let mut new_masks = masks.masks.clone();
        new_masks.push(Arc::new(mask));
        Self { masks: new_masks }
    }

    pub fn add_mask<M: NavMask + 'static>(&mut self, mask: M) {
        self.masks.push(Arc::new(mask));
    }

    pub fn clear(&mut self) {
        self.masks.clear();
    }

    pub fn nav(&self, grid: &ArrayView3<NavCell>, pos: UVec3) -> Option<NavCell> {
        let mut cell = grid[(pos.x as usize, pos.y as usize, pos.z as usize)].clone();

        for mask in &self.masks {
            if let Some(mask_cell) = mask.get(pos) {
                process_mask(&mut cell, mask_cell);
            }
        }

        Some(cell)
    }
}



#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Region3d {
    pub min: UVec3,
    pub max: UVec3,
}

impl Region3d {
    pub fn iter(&self) -> Region3dIter {
        Region3dIter {
            region: *self,
            current: self.min,
        }
    }
}

pub struct Region3dIter {
    region: Region3d,
    current: UVec3,
}

impl Iterator for Region3dIter {
    type Item = UVec3;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current.z >= self.region.max.z {
            return None;
        }

        let result = self.current;

        self.current.x += 1;
        if self.current.x >= self.region.max.x {
            self.current.x = self.region.min.x;
            self.current.y += 1;

            if self.current.y >= self.region.max.y {
                self.current.y = self.region.min.y;
                self.current.z += 1;
            }
        }

        Some(result)
    }
}