//! Navigation Masks for 
use bevy::{math::{IVec3, UVec3}, platform::collections::{HashMap, HashSet}};
use ndarray::ArrayView3;

use crate::{nav::{Nav, NavCell, Portal}, MovementCost};

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

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum NavCellMask {
    ImpassableOverride,
    PassableOverride(MovementCost),
    PortalOverride(Portal),
    ModifyCost(i32),
}

#[derive(Debug, Default, Clone)]
pub struct NavMask {
    masks: HashMap<UVec3, NavCellMask>,
}

impl NavMask {
    pub fn new() -> Self {
        Self { masks: HashMap::new() }
    }

    pub fn from_nav_mask(other: &NavMask) -> Self {
        Self {
            masks: other.masks.clone(),
        }
    }

    pub fn apply_region(
        &mut self,
        region: &Region3d,
        mask: NavCellMask,
    ) {
        for pos in region.iter() {
            self.masks.insert(pos, mask.clone());
        }
    }

    pub fn clear(&mut self) {
        self.masks.clear();
    }

    pub fn clear_region(&mut self, region: &Region3d) {
        for pos in region.iter() {
            self.masks.remove(&pos);
        }
    }

    pub fn apply_hashset(
        &mut self,
        positions: &HashSet<UVec3>,
        mask: NavCellMask,
    ) {
        for pos in positions {
            self.masks.insert(*pos, mask.clone());
        }
    }

    pub fn get(&self, pos: &UVec3) -> Option<&NavCellMask> {
        self.masks.get(pos)
    }
}

pub trait NavMaskView {
    fn nav(&self, grid: &ArrayView3<NavCell>, pos: UVec3) -> Option<NavCell>;
}

#[derive(Debug, Default, Clone)]
pub struct CompositeNavMask<'a> {
    masks: Vec<&'a NavMask>,
}

impl<'a> CompositeNavMask<'a> {
    pub fn new() -> Self {
        Self { masks: Vec::new() }
    }

    pub fn with(mut self, mask: &'a NavMask) -> Self {
        self.masks.push(mask);
        self
    }
}

impl NavMaskView for CompositeNavMask<'_> {
    fn nav(&self, grid: &ArrayView3<NavCell>, pos: UVec3) -> Option<NavCell> {
        let mut grid_cell = grid[(pos.x as usize, pos.y as usize, pos.z as usize)].clone();

        for mask in self.masks.iter() {
            if let Some(cell_mask) = mask.masks.get(&pos) {
                match cell_mask {
                    NavCellMask::ImpassableOverride => {
                        grid_cell.nav = Nav::Impassable;
                        break;
                    }
                    NavCellMask::PassableOverride(cost) => {
                        grid_cell.nav = Nav::Passable(*cost);
                        grid_cell.cost = *cost;
                        break;
                    }
                    NavCellMask::PortalOverride(portal) => {
                        grid_cell.nav = Nav::Portal(*portal);
                        break;
                    }
                    NavCellMask::ModifyCost(delta) => {
                        if let Nav::Passable(cost) = grid_cell.nav {
                            let new_cost = cost.saturating_add(*delta as MovementCost);
                            grid_cell.nav = Nav::Passable(new_cost);
                            grid_cell.cost = new_cost;
                        }
                    }
                }
            }
        }

        Some(grid_cell)
    }
}

pub struct CompositeNavMaskView<'a> {
    mask: &'a CompositeNavMask<'a>,
    offset: IVec3,
    bounds: (IVec3, IVec3),
}

impl<'a> CompositeNavMaskView<'a> {
    pub fn new(mask: &'a CompositeNavMask, offset: IVec3, bounds: (IVec3, IVec3)) -> Self {
        Self { mask, offset, bounds }
    }
}

impl NavMaskView for CompositeNavMaskView<'_> {
    fn nav(&self, grid: &ArrayView3<NavCell>, pos: UVec3) -> Option<NavCell> {
        let world_pos = self.offset + pos.as_ivec3();
        if world_pos.cmplt(self.bounds.0).any() || world_pos.cmpge(self.bounds.1).any() {
            return None;
        }

        self.mask.nav(grid, world_pos.as_uvec3())
    }
}
