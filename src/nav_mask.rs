//! Navigation Mask for overriding cell navigation properties for a specific pathfinding request.
use std::{
    hash::Hash,
    sync::{Arc, Mutex},
};

use bevy::{
    log,
    math::{IVec3, UVec3},
    platform::collections::{HashMap, HashSet},
};

use crate::{
    grid::Grid,
    nav::{Nav, NavCell},
    path::Path,
    prelude::Neighborhood,
    MovementCost, NavRegion,
};

/// Result of a navigation mask query.
#[derive(Debug)]
pub enum NavMaskResult {
    /// The mask contains the cell, returning the modified cell.
    Masked(NavCell),
    /// The mask does not contain the cell
    NotMasked,
    /// The mask is locked and cannot be accessed.
    Locked,
}

/// Mask for a single cell over [`NavCell`].
/// You can use this to override or modify the cost of the underlying [`NavCell`] in the [`crate::grid::Grid`].
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum NavCellMask {
    /// Overrides anything below this as impassable
    ImpassableOverride,
    /// Modifies the cost of the cell.
    /// If the cell is impassable, this will not change it.
    /// If the cell is passable, this will add the cost to the existing cost. In this way you can layer cost.
    ModifyCost(i32),
}

fn process_mask(mut cell: NavCell, mask: &NavCellMask) -> NavCell {
    match mask {
        NavCellMask::ImpassableOverride => {
            cell.nav = Nav::Impassable;
            cell.cost = MovementCost::MAX;
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
///
/// Example Usage:
/// ```rust
/// use bevy::prelude::*;
/// use bevy::platform::collections::HashMap;
/// use bevy_northstar::prelude::*;
///
/// #[derive(Resource, Default)]
/// struct NavMaskLayers(HashMap<String, NavMaskLayer>);
///
/// #[derive(Resource, Default)]
/// struct NavMasks(HashMap<String, NavMask>);
///
/// fn setup_masks(mut masks: ResMut<NavMasks>, layers: Res<NavMaskLayers>) {
///    let mask = NavMask::new();
///    if let Some(layer) = layers.0.get("water") {
///        mask.add_layer(layer.clone());
///    }
///
///    masks.0.insert("infantry".to_string(), mask);
/// }
/// ```
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
    /// * `original` - Provide the original [`NavCell`] so it can be masked if required.
    ///   This is mostly to avoid double cell lookup and having to potentially unwrap millions of cells.
    /// * `pos` - The position in the grid to get the masked [`NavCell`].
    /// # Returns
    /// A [`Result`] containing the masked [`NavCell`] or an error message if the lock is poisoned.
    pub fn get(&self, original: NavCell, pos: UVec3) -> NavMaskResult {
        match self.data.lock() {
            Ok(data) => {
                if let Some(masked) = data.get(original, pos) {
                    NavMaskResult::Masked(masked)
                } else {
                    NavMaskResult::NotMasked
                }
            }
            Err(_) => NavMaskResult::Locked,
        }
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
        // TODO: We can maybe improve this in the future by only removing paths that are affected by the new layer
        self.cached_paths.clear();
    }

    pub(crate) fn add_cached_path(&mut self, start: UVec3, end: UVec3, path: Path) {
        self.cached_paths.insert((start, end), path);
    }

    pub(crate) fn get_cached_path(&self, start: UVec3, end: UVec3) -> Option<&Path> {
        self.cached_paths.get(&(start, end))
    }

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> Option<NavCell> {
        if self.layers.is_empty() {
            return None;
        }

        let lookup_pos = if self.translation == IVec3::ZERO {
            pos
        } else {
            let translated_pos = pos.as_ivec3() - self.translation;

            if translated_pos.x < 0 || translated_pos.y < 0 || translated_pos.z < 0 {
                return None;
            }

            translated_pos.as_uvec3()
        };

        let mut result = prev;
        let mut mask_found = false;

        // Lock all layers once
        let layer_guards: Vec<_> = self
            .layers
            .iter()
            .map(|layer| layer.data.lock().unwrap())
            .collect();

        for guard in &layer_guards {
            if let Some(mask) = guard.mask.get(&lookup_pos) {
                result = process_mask(result, mask);
                mask_found = true;
            }
        }

        if mask_found {
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

    pub(crate) fn layer_count(&self) -> usize {
        self.layers.len()
    }
}

/// A single navigation mask layer than can be added to a [`NavMask`].
/// This is an Arc Mutex wrapper around the internal data structure to allow for shared access across threads.
///
/// Example Usage:
/// ```rust
/// use bevy::prelude::*;
/// use bevy_northstar::prelude::*;
///
/// let grid_settings = GridSettingsBuilder::new_3d(16, 16, 16).build();
/// let grid = Grid::<CardinalNeighborhood>::new(&grid_settings);
///
/// let layer = NavMaskLayer::new();
/// layer.insert_region_fill(
///     &grid,
///     NavRegion::new(UVec3::new(0, 0, 0), UVec3::new(10, 10, 10)),
///     NavCellMask::ModifyCost(50),
/// ).ok();
/// ```
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
    pub fn insert_mask<N: Neighborhood>(
        &self,
        grid: &Grid<N>,
        pos: UVec3,
        mask: NavCellMask,
    ) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_mask(grid, pos, mask);
        Ok(())
    }

    /// Inserts a [`NavCellMask`] over an entire region for the layer.
    /// # Arguments
    /// * `region` - The [`NavRegion`] in the grid to insert the mask. You can also use this in 2d with no z range.
    /// * `mask` - The [`NavCellMask`] to insert in the region.
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    pub fn insert_region_fill<N: Neighborhood>(
        &self,
        grid: &Grid<N>,
        region: NavRegion,
        mask: NavCellMask,
    ) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_region_fill(grid, region, mask);
        Ok(())
    }

    /// Inserts a [`NavCellMask`] on the outlines of a region.
    /// You can use this to box in an agent to an area etc.
    /// # Arguments
    /// * `region` - The [`NavRegion`] in the grid to insert the mask. You can also use this in 2d with no z range.
    /// * `mask` - The [`NavCellMask`] to insert in the region.
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    pub fn insert_region_outline<N: Neighborhood>(
        &self,
        grid: &Grid<N>,
        region: NavRegion,
        mask: NavCellMask,
    ) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?;
        data.insert_region_outline(grid, region, mask);
        Ok(())
    }

    /// Inserts a HashMap of `Uvec3` positions and their corresponding [`NavCellMask`] into the layer.
    /// # Arguments
    /// * `masks` - A HashMap where keys are positions and values are [`NavCellMask`].
    /// # Returns
    /// [`Result`] will fail if the mutex is poisoned.
    pub fn insert_hashmap<N: Neighborhood>(
        &self,
        grid: &Grid<N>,
        masks: &HashMap<UVec3, NavCellMask>,
    ) -> Result<(), String> {
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
    pub fn insert_hashset<N: Neighborhood>(
        &self,
        grid: &Grid<N>,
        cells: &HashSet<UVec3>,
        mask: NavCellMask,
    ) -> Result<(), String> {
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
        if !grid.in_bounds(pos) {
            log::warn!(
                "Unable to insert mask position: {:?} is out of bounds!",
                pos
            );
            return;
        }

        self.mask.insert(pos, mask);
        let chunk = grid.chunk_at_position(pos).unwrap();
        self.chunks.insert(chunk.index());
    }

    pub fn insert_region_fill<N: Neighborhood>(
        &mut self,
        grid: &Grid<N>,
        region: NavRegion,
        mask: NavCellMask,
    ) {
        for pos in region.iter() {
            self.insert_mask(grid, pos, mask.clone());
        }
    }

    pub fn insert_region_outline<N: Neighborhood>(
        &mut self,
        grid: &Grid<N>,
        region: NavRegion,
        mask: NavCellMask,
    ) {
        // Insert the outline of the region
        for x in region.min.x..region.max.x {
            for y in region.min.y..region.max.y {
                for z in region.min.z..region.max.z {
                    let pos = UVec3::new(x, y, z);
                    if pos.x == region.min.x
                        || pos.x == region.max.x - 1
                        || pos.y == region.min.y
                        || pos.y == region.max.y - 1
                        || pos.z == region.min.z
                        || pos.z == region.max.z - 1
                    {
                        self.insert_mask(grid, pos, mask.clone());
                    }
                }
            }
        }
    }

    pub fn insert_hashmap<N: Neighborhood>(
        &mut self,
        grid: &Grid<N>,
        masks: &HashMap<UVec3, NavCellMask>,
    ) {
        for (pos, mask) in masks {
            self.insert_mask(grid, *pos, mask.clone());
        }
    }

    pub fn insert_hashset<N: Neighborhood>(
        &mut self,
        grid: &Grid<N>,
        cells: &HashSet<UVec3>,
        mask: NavCellMask,
    ) {
        for pos in cells {
            self.insert_mask(grid, *pos, mask.clone());
        }
    }

    pub fn clear(&mut self) {
        self.mask.clear();
        self.chunks.clear();
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

#[cfg(test)]
mod tests {
    use crate::{grid::GridSettingsBuilder, prelude::CardinalNeighborhood};

    use super::*;

    #[test]
    fn test_nav_layer() {
        let grid_settings = GridSettingsBuilder::new_3d(16, 16, 16).build();
        let grid = Grid::<CardinalNeighborhood>::new(&grid_settings);

        let mask = NavMask::new();

        // Create layers directly
        let layer1 = NavMaskLayer::new();
        layer1
            .insert_region_fill(
                &grid,
                NavRegion::new(UVec3::new(0, 0, 0), UVec3::new(4, 4, 4)),
                NavCellMask::ImpassableOverride,
            )
            .ok();

        let layer2 = NavMaskLayer::new();
        layer2
            .insert_region_fill(
                &grid,
                NavRegion::new(UVec3::new(4, 4, 4), UVec3::new(8, 8, 8)),
                NavCellMask::ModifyCost(2),
            )
            .ok();

        let layer3 = NavMaskLayer::new();
        layer3
            .insert_region_fill(
                &grid,
                NavRegion::new(UVec3::new(4, 4, 4), UVec3::new(8, 8, 8)),
                NavCellMask::ModifyCost(3),
            )
            .unwrap();

        mask.add_layer(layer1).unwrap();
        mask.add_layer(layer2).unwrap();
        mask.add_layer(layer3).unwrap();

        let layer4 = NavMaskLayer::new();

        let mut layer4_data: NavMaskLayerData = layer4.into_data();
        layer4_data.insert_mask(&grid, UVec3::new(5, 5, 5), NavCellMask::ImpassableOverride);

        let updated_layer4: NavMaskLayer = layer4_data.into();

        mask.add_layer(updated_layer4).unwrap();

        if let NavMaskResult::Masked(cell) = mask.get(NavCell::default(), UVec3::new(1, 1, 1)) {
            assert_eq!(cell.nav, Nav::Impassable);
        } else {
            panic!("Expected masked result");
        }

        if let NavMaskResult::Masked(cell) = mask.get(NavCell::default(), UVec3::new(5, 5, 5)) {
            assert_eq!(cell.nav, Nav::Impassable);
        } else {
            panic!("Expected masked result");
        }

        if let NavMaskResult::Masked(cell) = mask.get(NavCell::default(), UVec3::new(6, 6, 6)) {
            assert_eq!(cell.nav, Nav::Passable(6));
        } else {
            panic!("Expected masked result");
        }

        if let NavMaskResult::NotMasked = mask.get(NavCell::default(), UVec3::new(40, 40, 40)) {
            // This position is not masked, so we should get NotMasked
        } else {
            panic!("Expected not masked result");
        }
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
        modified_cell = process_mask(modified_cell, &NavCellMask::ModifyCost(5));
        assert_eq!(modified_cell.nav, Nav::Passable(6));
        assert_eq!(modified_cell.cost, 6);

        // Test ModifyCost
        modified_cell = cell.clone();
        modified_cell.nav = Nav::Passable(10);
        modified_cell = process_mask(modified_cell, &NavCellMask::ModifyCost(-3));
        assert_eq!(modified_cell.nav, Nav::Passable(7));
    }
}
