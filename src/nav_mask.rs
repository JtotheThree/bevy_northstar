//! Navigation Masks for
use std::sync::{Arc, Mutex};

use bevy::{
    math::{IVec3, UVec3},
    platform::collections::{HashMap, HashSet},
};
use ndarray::ArrayView3;

use crate::{
    nav::{Nav, NavCell, Portal},
    MovementCost,
};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum NavCellMask {
    /// Overrides anything below this as impassable
    ImpassableOverride,
    /// Overrides anything below this as passable.
    /// This might have odd interactions if you override a grid NavCell that is
    /// already impassable because it won't change the surrounding neighbors.
    PassableOverride(MovementCost),
    /// Overrides anything below this as a portal.
    PortalOverride(Portal),
    /// Modifies the cost of the cell.
    /// If the cell is impassable, this will not change it.
    /// If the cell is passable, this will add the cost to the existing cost. In this way you can layer cost.
    ModifyCost(i32),
}

fn process_mask(mut cell: NavCell, mask: &NavCellMask) -> NavCell {
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
                // Fix: Handle negative deltas properly
                let new_cost = if *delta < 0 {
                    cost.saturating_sub((-*delta) as MovementCost)
                } else {
                    cost.saturating_add(*delta as MovementCost)
                };
                cell.nav = Nav::Passable(new_cost);
                cell.cost = new_cost;
            }
        }
    }
    cell
}

/// Arc Mutex wrapper for NavMaskData.
/// I probably need to rename this so it's obvious. SharedNavMask?
#[derive(Clone, Debug, Default)]
pub struct NavMask {
    pub(crate) data: Arc<Mutex<NavMaskData>>,
}

impl NavMask {
    pub fn new() -> Self {
        Self {
            data: Arc::new(Mutex::new(NavMaskData {
                layers: Vec::new(),
                translation: IVec3::ZERO,
            })),
        }
    }

    pub fn add_layer(&self, layer: NavMaskLayer) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.add_layer(layer);
        Ok(())
    }

    pub fn with_additional_layer(&self, layer: NavMaskLayer) -> Self {
        let new_mask = NavMask::new();

        // Copy all existing layers
        {
            let original_data = self.data.lock().unwrap();
            let mut new_data = new_mask.data.lock().unwrap();

            new_data.layers = original_data.layers.clone();
            new_data.translation = original_data.translation;
        }

        new_mask.add_layer(layer).unwrap();

        new_mask
    }

    pub fn get(&self, prev: NavCell, pos: UVec3) -> Result<NavCell, String> {
        let data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        Ok(data.get(prev, pos))
    }

    pub fn translate_by(&self, offset: IVec3) -> Self {
        let original_data = self.data.lock().unwrap();
        let new_data = original_data.translate_by(offset);

        Self {
            data: Arc::new(Mutex::new(new_data)),
        }
    }

    pub fn translate_by_mut(&self, offset: IVec3) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.translate_by_mut(offset);
        Ok(())
    }

    pub fn clear(&self) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.clear();
        Ok(())
    }

    /// Merges all the layers in the NavMask into a single layer if you don't need to change them often.
    /// Useful for performance in theory, but in benches it's slower for some reason? I doubt having less layers would decrease perforamnce so there must be another issue.
    pub fn flatten(&mut self) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;

        let mut merged_layer_data = NavMaskLayerData::new();

        for layer in &data.layers {
            let layer_data = layer.data.lock().unwrap();
            for (pos, mask) in &layer_data.mask {
                merged_layer_data.mask.insert(*pos, mask.clone());
            }
        }

        data.layers.clear();
        data.layers.push(NavMaskLayer::from(merged_layer_data));

        Ok(())
    }
}

impl From<NavMask> for NavMaskData {
    fn from(mask: NavMask) -> Self {
        match Arc::try_unwrap(mask.data) {
            Ok(mutex) => mutex.into_inner().unwrap(),
            Err(arc) => {
                let data = arc.lock().unwrap();
                data.clone()
            }
        }
    }
}

impl From<&NavMask> for NavMaskData {
    fn from(mask: &NavMask) -> Self {
        let data = mask.data.lock().unwrap();
        data.clone()
    }
}

/// The underlying data structure for NavMask
#[derive(Clone, Debug, Default)]
pub(crate) struct NavMaskData {
    pub(crate) layers: Vec<NavMaskLayer>,
    translation: IVec3,
}

impl NavMaskData {
    pub(crate) fn new() -> Self {
        Self {
            layers: Vec::new(),
            translation: IVec3::ZERO,
        }
    }

    pub(crate) fn add_layer(&mut self, layer: NavMaskLayer) {
        self.layers.push(layer);
    }

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        let translated_pos = pos.as_ivec3() - self.translation;

        if translated_pos.x < 0 || translated_pos.y < 0 || translated_pos.z < 0 {
            return prev;
        }

        let translated_pos = translated_pos.as_uvec3();

        self.layers
            .iter()
            .fold(prev, |cell, layer| layer.get(cell, translated_pos))
    }

    pub(crate) fn translate_by(&self, offset: IVec3) -> Self {
        let mut cloned = self.clone();
        cloned.translation += offset;
        cloned
    }

    pub(crate) fn translate_by_mut(&mut self, offset: IVec3) {
        self.translation += offset;
    }

    pub(crate) fn clear(&mut self) {
        self.layers.clear();
    }
}

/// Arc Mutex wrapper for NavMaskLayerData.
/// I probably need to rename this so it's obvious. SharedNavMaskLayer?
#[derive(Clone, Default, Debug)]
pub struct NavMaskLayer {
    data: Arc<Mutex<NavMaskLayerData>>,
}

impl NavMaskLayer {
    pub fn new() -> Self {
        Self {
            data: Arc::new(Mutex::new(NavMaskLayerData::new())),
        }
    }

    pub fn insert_mask(&self, pos: UVec3, mask: NavCellMask) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_mask(pos, mask);
        Ok(())
    }

    pub fn remove_mask(&self, pos: UVec3) -> Result<Option<NavCellMask>, String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.remove(&pos))
    }

    pub fn insert_region(&self, region: Region3d, mask: NavCellMask) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_region(region, mask);
        Ok(())
    }

    pub fn insert_hashmap(&self, masks: &HashMap<UVec3, NavCellMask>) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_hashmap(masks);
        Ok(())
    }

    pub fn insert_hashset(&self, cells: &HashSet<UVec3>, mask: NavCellMask) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_hashset(cells, mask);
        Ok(())
    }

    pub fn clear(&self) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.clear();
        Ok(())
    }

    pub fn batch_update<F, R>(&self, f: F) -> Result<R, String>
    where
        F: FnOnce(&mut NavMaskLayerData) -> R,
    {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(f(&mut data))
    }

    pub fn get_mask(&self, pos: UVec3) -> Result<Option<NavCellMask>, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.get(&pos).cloned())
    }

    pub fn contains(&self, pos: UVec3) -> Result<bool, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.contains_key(&pos))
    }

    pub fn len(&self) -> Result<usize, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.len())
    }

    pub fn is_empty(&self) -> Result<bool, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.is_empty())
    }

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        if let Ok(data) = self.data.lock() {
            data.get(prev, pos)
        } else {
            panic!("NavMaskLayer lock poisoned")
        }
    }

    pub fn into_data(self) -> NavMaskLayerData {
        self.into()
    }

    pub fn to_data(&self) -> NavMaskLayerData {
        self.into()
    }
}

impl From<NavMaskLayerData> for NavMaskLayer {
    fn from(data: NavMaskLayerData) -> Self {
        Self {
            data: Arc::new(Mutex::new(data)),
        }
    }
}

/// The underlying data structure for NavMaskLayer
#[derive(Clone, Debug, Default)]
pub struct NavMaskLayerData {
    pub mask: HashMap<UVec3, NavCellMask>,
}

impl NavMaskLayerData {
    pub fn new() -> Self {
        Self {
            mask: HashMap::new(),
        }
    }

    pub fn insert_mask(&mut self, pos: UVec3, mask: NavCellMask) {
        self.mask.insert(pos, mask);
    }

    pub fn insert_region(&mut self, region: Region3d, mask: NavCellMask) {
        for pos in region.iter() {
            self.mask.insert(pos, mask.clone());
        }
    }

    pub fn insert_hashmap(&mut self, masks: &HashMap<UVec3, NavCellMask>) {
        for (pos, mask) in masks {
            self.mask.insert(*pos, mask.clone());
        }
    }

    pub fn insert_hashset(&mut self, cells: &HashSet<UVec3>, mask: NavCellMask) {
        for pos in cells {
            self.mask.insert(*pos, mask.clone());
        }
    }

    pub fn clear(&mut self) {
        self.mask.clear();
    }

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        let mut cell = prev;
        if let Some(mask) = self.mask.get(&pos) {
            cell = process_mask(cell, mask);
        }
        cell
    }
}

impl From<NavMaskLayer> for NavMaskLayerData {
    fn from(layer: NavMaskLayer) -> Self {
        match Arc::try_unwrap(layer.data) {
            Ok(mutex) => mutex.into_inner().unwrap(),
            Err(arc) => {
                let data = arc.lock().unwrap();
                data.clone()
            }
        }
    }
}

impl From<&NavMaskLayer> for NavMaskLayerData {
    fn from(layer: &NavMaskLayer) -> Self {
        let data = layer.data.lock().unwrap();
        data.clone()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Region3d {
    pub min: UVec3,
    pub max: UVec3,
}

impl Region3d {
    pub fn new(min: UVec3, max: UVec3) -> Self {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "Invalid region bounds"
        );
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nav_layer() {
        let mask = NavMask::new();

        // Create layers directly
        let layer1 = NavMaskLayer::new();
        layer1
            .insert_region(
                Region3d::new(UVec3::new(0, 0, 0), UVec3::new(4, 4, 4)),
                NavCellMask::ImpassableOverride,
            )
            .unwrap();

        let layer2 = NavMaskLayer::new();
        layer2
            .insert_region(
                Region3d::new(UVec3::new(4, 4, 4), UVec3::new(8, 8, 8)),
                NavCellMask::PassableOverride(2),
            )
            .unwrap();

        let layer3 = NavMaskLayer::new();
        layer3
            .insert_region(
                Region3d::new(UVec3::new(4, 4, 4), UVec3::new(8, 8, 8)),
                NavCellMask::ModifyCost(3),
            )
            .unwrap();

        mask.add_layer(layer1).unwrap();
        mask.add_layer(layer2).unwrap();
        mask.add_layer(layer3).unwrap();

        let layer4 = NavMaskLayer::new();

        let mut layer4_data: NavMaskLayerData = layer4.into_data();
        layer4_data.insert_mask(UVec3::new(5, 5, 5), NavCellMask::ImpassableOverride);

        let updated_layer4: NavMaskLayer = layer4_data.into();

        mask.add_layer(updated_layer4).unwrap();

        assert_eq!(
            mask.get(NavCell::default(), UVec3::new(1, 1, 1))
                .unwrap()
                .nav,
            Nav::Impassable
        );
        assert_eq!(
            mask.get(NavCell::default(), UVec3::new(5, 5, 5))
                .unwrap()
                .nav,
            Nav::Impassable
        );
        assert_eq!(
            mask.get(NavCell::default(), UVec3::new(6, 6, 6))
                .unwrap()
                .nav,
            Nav::Passable(5)
        );
        assert_eq!(
            mask.get(NavCell::default(), UVec3::new(40, 40, 40))
                .unwrap()
                .nav,
            Nav::Passable(1)
        );
    }

    #[test]
    fn test_process_mask() {
        let cell = NavCell::default();
        let mut modified_cell = cell.clone();

        // Test ImpassableOverride
        modified_cell = process_mask(modified_cell, &NavCellMask::ImpassableOverride);
        assert_eq!(modified_cell.nav, Nav::Impassable);

        // Test PassableOverride
        modified_cell = cell.clone();
        modified_cell = process_mask(modified_cell, &NavCellMask::PassableOverride(5));
        assert_eq!(modified_cell.nav, Nav::Passable(5));
        assert_eq!(modified_cell.cost, 5);

        // Test ModifyCost
        modified_cell = cell.clone();
        modified_cell.nav = Nav::Passable(10);
        modified_cell = process_mask(modified_cell, &NavCellMask::ModifyCost(-3));
        assert_eq!(modified_cell.nav, Nav::Passable(7));
    }
}
