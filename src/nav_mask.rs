//! Navigation Mask for overriding cell navigation properties for a specific pathfinding request.
use std::{hash::Hash, sync::{Arc, Mutex}};

use bevy::{
    math::{IVec3, UVec3},
    platform::collections::{HashMap, HashSet},
};
use ndarray::ArrayView3;

use crate::{
    grid::Grid, nav::{Nav, NavCell, Portal}, path::Path, prelude::Neighborhood, MovementCost
};

/// Mask for a single cell over [`NavCell`].
/// You can use this to override or modify the cost of the underlying [`NavCell`] in the [`crate::grid::Grid`].
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

/// Holds a collection of layers that can be used to override the navigation properties of a grid.
/// Each layer can have its own set of masks that apply to specific cells in the grid.
/// This is useful for creating complex navigation scenarios where different agents might have different navigation properties.
///
/// This is an Arc Mutex wrapper around the internal data structure to allow for shared access across threads.
#[derive(Clone, Debug, Default)]
pub struct NavMask {
    pub(crate) data: Arc<Mutex<NavMaskData>>,
}

impl NavMask {
    /// Creates a new empty NavMask.
    pub fn new() -> Self {
        Self {
            data: Arc::new(Mutex::new(NavMaskData::new())),
        }
    }

    /// Adds a new new [`NavMaskLayer`] to the NavMask.
    /// The most recent layer added will be the last one applied.
    pub fn add_layer(&self, layer: NavMaskLayer) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.add_layer(layer);
        Ok(())
    }

    /// Clones this NavMask and adds an additional layer to it.
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

    /// Gets the masked [`NavCell`] for the given position.
    /// # Arguments
    /// * `prev` - Provide the previous [`NavCell`] so the it can be returned if the mask layers do not contain a mask for that position.
    ///   This is mostly to avoid double cell lookup and having to potentially unwrap millions of cells.
    /// * `pos` - The position in the grid to get the masked [`NavCell`].
    /// # Returns
    /// A [`Result`] containing the masked [`NavCell`] or an error message if the lock is poisoned.
    pub fn get(&self, prev: NavCell, pos: UVec3) -> Result<Option<NavCell>, String> {
        let data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        Ok(data.get(prev, pos))
    }

    /// Returns a new NavMask with the translation applied.
    pub fn translate_by(&self, offset: IVec3) -> Self {
        let original_data = self.data.lock().unwrap();
        let new_data = original_data.translate_by(offset);

        Self {
            data: Arc::new(Mutex::new(new_data)),
        }
    }

    /// Applies a translation to the NavMask in place.
    pub fn translate_by_mut(&self, offset: IVec3) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.translate_by_mut(offset);
        Ok(())
    }

    /// Clears all layers in the NavMask.
    pub fn clear(&self) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.clear();
        Ok(())
    }

    /// Flattens the NavMask by merging all layers into a single layer. This can help with performance.
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
    pub(crate) cached_paths: HashMap<(UVec3, UVec3), Path>,
    translation: IVec3,
}

impl NavMaskData {
    pub(crate) fn new() -> Self {
        Self {
            layers: Vec::new(),
            cached_paths: HashMap::new(),
            translation: IVec3::ZERO,
        }
    }

    pub(crate) fn add_layer(&mut self, layer: NavMaskLayer) {
        self.layers.push(layer);
    }

    pub(crate) fn add_cached_path(&mut self, start: UVec3, end: UVec3, path: Path) {
        self.cached_paths.insert((start, end), path);
    }

    pub(crate) fn get_cached_path(&self, start: UVec3, end: UVec3) -> Option<&Path> {
        self.cached_paths.get(&(start, end))
    }

    /*pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> Option<NavCell> {
        let translated_pos = pos.as_ivec3() - self.translation;

        if translated_pos.x < 0 || translated_pos.y < 0 || translated_pos.z < 0 {
            return None;
        }

        let translated_pos = translated_pos.as_uvec3();

        self.layers
            .iter()
            .fold(prev, |cell, layer| layer.get(cell, translated_pos))
    }*/

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> Option<NavCell> {
        let translated_pos = pos.as_ivec3() - self.translation;

        if translated_pos.x < 0 || translated_pos.y < 0 || translated_pos.z < 0 {
            return None;
        }

        let translated_pos = translated_pos.as_uvec3();

        let mut result = prev;
        let mut has_any_mask = false;

        for layer in &self.layers {
            let data = layer.data.lock().unwrap();
            if let Some(mask) = data.mask.get(&translated_pos) {
                result = process_mask(result, mask);
                has_any_mask = true;
            }
        }

        if has_any_mask {
            Some(result)
        } else {
            None
        }
    }

    pub(crate) fn chunk_in_mask(&self, chunk_index: (usize, usize, usize)) -> bool {
        self.layers.iter().any(|layer| {
            let data = layer.data.lock().unwrap();
            data.chunks.contains(&chunk_index)
        })
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

/// A single navigation mask layer than can be added to a [`NavMask`].
/// This is an Arc Mutex wrapper around the internal data structure to allow for shared access across threads.
#[derive(Clone, Default, Debug)]
pub struct NavMaskLayer {
    data: Arc<Mutex<NavMaskLayerData>>,
}

impl NavMaskLayer {
    /// Creates a new empty NavMaskLayer.
    pub fn new() -> Self {
        Self {
            data: Arc::new(Mutex::new(NavMaskLayerData::new())),
        }
    }

    /// Inserts a single position [`NavCellMask`] into the layer.
    /// # Arguments
    /// * `pos` - The position in the grid to insert the mask.
    /// * `mask` - The [`NavCellMask`] to insert at the position.
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    pub fn insert_mask<N: Neighborhood>(&self, grid: &Grid<N>, pos: UVec3, mask: NavCellMask) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_mask(grid, pos, mask);
        Ok(())
    }

    /// Clears the mask at a single position.
    /// # Arguments
    /// * `pos` - The position in the grid to clear the mask.
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    //pub fn remove_mask<N: Neighborhood>(&self, grid: &Grid<N>, pos: UVec3) -> Result<Option<NavCellMask>, String> {
    //    let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
    //    Ok(data.mask.remove(&pos))
    //}

    /// Inserts a [`NavCellMask`] over an entire region for the layer.
    /// # Arguments
    /// * `region` - The [`Region3d`] in the grid to insert the mask. You can also use this in 2d with no z range.
    /// * `mask` - The [`NavCellMask`] to insert in the region.
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    pub fn insert_region<N: Neighborhood>(&self, grid: &Grid<N>, region: Region3d, mask: NavCellMask) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_region(grid, region, mask);
        Ok(())
    }

    /// Inserts a HashMap of `Uvec3` positions and their corresponding [`NavCellMask`] into the layer.
    /// # Arguments
    /// * `masks` - A HashMap where keys are positions and values are [`NavCellMask`].
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    pub fn insert_hashmap<N: Neighborhood>(&self, grid: &Grid<N>, masks: &HashMap<UVec3, NavCellMask>) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_hashmap(grid, masks);
        Ok(())
    }

    /// Inserts a HashSet of `Uvec3` positions with the same [`NavCellMask`] into the layer.
    /// # Arguments
    /// * `cells` - A HashSet of positions to insert the same [`NavCellMask`].
    /// * `mask` - The [`NavCellMask`] to apply to all positions in the HashSet.
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    pub fn insert_hashset<N: Neighborhood>(&self, grid: &Grid<N>, cells: &HashSet<UVec3>, mask: NavCellMask) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_hashset(grid, cells, mask);
        Ok(())
    }

    /// Clears all of the masks in the layer.
    pub fn clear(&self) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.clear();
        Ok(())
    }

    // pub fn batch_update<F, R>(&self, f: F) -> Result<R, String>
    // where
    //     F: FnOnce(&mut NavMaskLayerData) -> R,
    // {
    //     let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
    //     Ok(f(&mut data))
    // }

    /// Gets the [`NavCellMask`] for a specific position in the layer.
    pub fn get_mask(&self, pos: UVec3) -> Result<Option<NavCellMask>, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.get(&pos).cloned())
    }

    /// Checks if the layer contains a mask for a specific position.
    pub fn contains(&self, pos: UVec3) -> Result<bool, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.contains_key(&pos))
    }

    /// Returns the number of masks in the layer.
    pub fn len(&self) -> Result<usize, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.len())
    }

    /// Checks if the layer is empty.
    pub fn is_empty(&self) -> Result<bool, String> {
        let data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        Ok(data.mask.is_empty())
    }

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> Option<NavCell> {
        if let Ok(data) = self.data.lock() {
            data.get(prev, pos)
        } else {
            panic!("NavMaskLayer lock poisoned")
        }
    }

    #[allow(dead_code)]
    fn into_data(self) -> NavMaskLayerData {
        self.into()
    }

    #[allow(dead_code)]
    fn to_data(&self) -> NavMaskLayerData {
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
pub(crate) struct NavMaskLayerData {
    pub mask: HashMap<UVec3, NavCellMask>,
    pub chunks: HashSet<(usize, usize, usize)>,
}

impl NavMaskLayerData {
    pub fn new() -> Self {
        Self {
            mask: HashMap::new(),
            chunks: HashSet::new(),
        }
    }

    pub fn insert_mask<N: Neighborhood>(&mut self, grid: &Grid<N>, pos: UVec3, mask: NavCellMask) {
        self.mask.insert(pos, mask);
        let chunk = grid.chunk_at_position(pos).unwrap();
        self.chunks.insert(chunk.index());
    }

    pub fn insert_region<N: Neighborhood>(&mut self, grid: &Grid<N>, region: Region3d, mask: NavCellMask) {
        for pos in region.iter() {
            self.mask.insert(pos, mask.clone());
            let chunk = grid.chunk_at_position(pos).unwrap();
            self.chunks.insert(chunk.index());
        }
    }

    pub fn insert_hashmap<N: Neighborhood>(&mut self, grid: &Grid<N>, masks: &HashMap<UVec3, NavCellMask>) {
        for (pos, mask) in masks {
            self.mask.insert(*pos, mask.clone());
            let chunk = grid.chunk_at_position(*pos).unwrap();
            self.chunks.insert(chunk.index());
        }
    }

    pub fn insert_hashset<N: Neighborhood>(&mut self, grid: &Grid<N>, cells: &HashSet<UVec3>, mask: NavCellMask) {
        for pos in cells {
            self.mask.insert(*pos, mask.clone());
            let chunk = grid.chunk_at_position(*pos).unwrap();
            self.chunks.insert(chunk.index());
        }
    }

    pub fn clear(&mut self) {
        self.mask.clear();
        self.chunks.clear();
    }

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> Option<NavCell> {
        if let Some(mask) = self.mask.get(&pos) {
            Some(process_mask(prev, mask));
        }
        
        None
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

/// A Region3d with an iter method to iterate over all positions in the region.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Region3d {
    /// The minimum position of the region.
    pub min: UVec3,
    /// The maximum position of the region (exclusive).
    pub max: UVec3,
}

impl Region3d {
    /// Creates a new Region3d with the given minimum and maximum positions.
    pub fn new(min: UVec3, max: UVec3) -> Self {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "Invalid region bounds"
        );
        Self { min, max }
    }

    /// Create a new Region3d from a grid's shape.
    pub fn from_grid(grid: &ArrayView3<NavCell>) -> Self {
        let shape = grid.shape();
        Self {
            min: UVec3::new(0, 0, 0),
            max: UVec3::new(shape[0] as u32, shape[1] as u32, shape[2] as u32),
        }
    }

    /// Returns an iterator over all positions in the region.
    pub fn iter(&self) -> Region3dIter {
        Region3dIter {
            region: *self,
            current: self.min,
        }
    }
}

/// An iterator over all positions in a Region3d.
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

/*#[cfg(test)]
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
}*/
