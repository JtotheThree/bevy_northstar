//! Navigation Masks for 
use std::sync::Arc;

use bevy::{math::{IVec3, UVec3}, platform::collections::HashMap};
use ndarray::ArrayView3;

use crate::{nav::{Nav, NavCell, Portal}, MovementCost};


#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum NavCellMask {
    ImpassableOverride,
    PassableOverride(MovementCost),
    PortalOverride(Portal),
    ModifyCost(i32),
}

#[derive(Clone)]
pub struct NavMask {
    layers: Vec<Arc<NavMaskLayer>>,
    translation: IVec3,
}

impl NavMask {
    pub fn new() -> Self {
        let translation = IVec3::ZERO;

        Self { layers: Vec::new(), translation }
    }

    pub fn from_layer(layer: Arc<NavMaskLayer>) -> Self {
        let translation = IVec3::ZERO;

        Self {
            layers: vec![layer],
            translation,
        }
    }

    pub fn with_additional_layer(&self, layer: Arc<NavMaskLayer>) -> Self {
        let mut cloned = self.clone();
        cloned.add_layer(layer);
        cloned
    }

    pub fn add_layer(&mut self, layer: Arc<NavMaskLayer>) -> &mut Self {
        self.layers.push(layer);
        self
    }

    pub fn clear(&mut self) {
        self.layers.clear();
    }

    pub fn translate_by(&mut self, offset: IVec3) -> &mut Self {
        self.translation += offset;
        self
    }

    pub fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        let pos = pos.as_ivec3() - self.translation;

        if pos.x < 0 || pos.y < 0 || pos.z < 0 {
            return prev; // Out of bounds
        }

        let pos = pos.as_uvec3();

        self.layers.iter().fold(prev, |cell, layer| layer.get(cell, pos))
    }
}

fn process_mask(
    mut cell: NavCell,
    mask: &NavCellMask,
) -> NavCell {
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
    cell
}

#[derive(Debug, Default)]
pub struct NavMaskLayer {
    mask: HashMap<UVec3, NavCellMask>,
}

impl NavMaskLayer {
    pub fn new() -> Self {
        Self {
            mask: HashMap::new(),
        }
    }

    pub fn insert_mask(&mut self, pos: UVec3, mask: NavCellMask) {
        self.mask.insert(pos, mask);
    }

    pub fn insert_region(
        &mut self,
        region: Region3d,
        mask: NavCellMask,
    ) {
        for pos in region.iter() {
            self.insert_mask(pos, mask.clone());
        }
    }

    pub fn insert_hashmap(
        &mut self,
        masks: &HashMap<UVec3, NavCellMask>,
    ) {
        for (pos, mask) in masks {
            self.insert_mask(*pos, mask.clone());
        }
    }

    pub fn clear_mask(&mut self, pos: UVec3) {
        self.mask.remove(&pos);
    }

    pub fn clear_region(&mut self, region: Region3d) {
        for pos in region.iter() {
            self.clear_mask(pos);
        }
    }

    pub fn clear_hashmap(&mut self, masks: &HashMap<UVec3, NavCellMask>) {
        for pos in masks.keys() {
            self.clear_mask(*pos);
        }
    }

    pub fn clear(&mut self) {
        self.mask.clear();
    }

    fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        let mut cell = prev;

        if let Some(mask) = self.mask.get(&pos) {
            cell = process_mask(cell, mask);
        }
        
        cell
    }
}
/*
pub struct TranslatedNavMask<'a> {
    mask: &'a dyn NavMaskView,
    offset: IVec3,
    bounds: (IVec3, IVec3),
}

impl<'a> TranslatedNavMask<'a> {
    /// Creates a new `TranslatedNavMask` that translates positions by the given offset.
    ///
    /// The `region` defines the bounds of the mask in world coordinates.
    pub fn new(region: Region3d, mask: &'a dyn NavMaskView) -> Self {
        let offset = region.min.as_ivec3();
        let max = region.max.as_ivec3();
        Self {
            mask,
            offset,
            bounds: (offset, max),
        }
    }
}

impl<'a> NavMaskView for TranslatedNavMask<'a> {
    fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        let world = pos.as_ivec3();
        // Bounds check
        if world.cmplt(self.bounds.0).any() || world.cmpge(self.bounds.1).any() {
            return prev;
        }

        // Translate world → local
        let local = (world - self.offset).as_uvec3();
        self.mask.get(prev, local)
    }

    fn as_view(self) -> Option<Arc<dyn NavMaskView>> {
        None
    }
}

*/

/*pub struct TranslatedNavMask {
    inner: Arc<dyn NavMaskView>,
    offset: IVec3,
    bounds: (IVec3, IVec3),
}

impl TranslatedNavMask {
    pub fn new(region: Region3d, inner: Arc<dyn NavMaskView>) -> Self {
        let offset = region.min.as_ivec3();
        let max = region.max.as_ivec3();
        Self {
            inner,
            offset,
            bounds: (offset, max),
        }
    }
}

impl NavMaskView for TranslatedNavMask {
    fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        let world = pos.as_ivec3();
        // Bounds check
        if world.cmplt(self.bounds.0).any() || world.cmpge(self.bounds.1).any() {
            return prev;
        }

        // Translate world → local
        let local = (world - self.offset).as_uvec3();
        self.inner.get(prev, local)
    }

    fn as_view(self) -> Arc<dyn NavMaskView> {
        Arc::new(self)
    }
}*/

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Region3d {
    pub min: UVec3,
    pub max: UVec3,
}

impl Region3d {
    pub fn new(min: UVec3, max: UVec3) -> Self {
        assert!(min.x <= max.x && min.y <= max.y && min.z <= max.z, "Invalid region bounds");
        Self { min, max }
    }

    pub fn from_grid(grid: &ArrayView3<NavCell>) -> Self {
        let shape = grid.shape();
        Self {
            min: UVec3::new(0, 0, 0),
            max: UVec3::new(shape[0] as u32, shape[1] as u32, shape[2] as u32),
        }
    }

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
