//! Navigation Masks for 
use std::sync::{Arc, Mutex};

use bevy::{math::{IVec3, UVec3}, platform::collections::{HashMap, HashSet}};
use ndarray::ArrayView3;

use crate::{nav::{Nav, NavCell, Portal}, MovementCost};


#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Region3d {
    pub min: UVec3,
    pub max: UVec3,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum NavCellMask {
    ImpassableOverride,
    PassableOverride(MovementCost),
    PortalOverride(Portal),
    ModifyCost(i32),
}

// Private implementation details - users never see these
#[derive(Clone)]
pub(crate) struct NavMaskData {
    layers: Vec<Arc<NavMaskLayerData>>,
    translation: IVec3,
}

impl NavMaskData {
    pub(crate) fn new() -> Self {
        Self {
            layers: Vec::new(),
            translation: IVec3::ZERO,
        }
    }

    pub(crate) fn add_layer(&mut self, layer: Arc<NavMaskLayerData>) {
        self.layers.push(layer);
    }

    pub(crate) fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        // Apply translation first
        let translated_pos = pos.as_ivec3() - self.translation;

        // Check bounds after translation
        if translated_pos.x < 0 || translated_pos.y < 0 || translated_pos.z < 0 {
            return prev;
        }

        let translated_pos = translated_pos.as_uvec3();

        // Apply all layers in sequence
        self.layers.iter().fold(prev, |cell, layer_data| {
            layer_data.get(cell, translated_pos)
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

#[derive(Clone)]
pub struct NavMask {
    data: Arc<Mutex<NavMaskData>>,
}

impl NavMask {
    pub fn new() -> Self {
        Self {
            data: Arc::new(Mutex::new(NavMaskData {
                layers: Vec::new(),
                translation: IVec3::ZERO,
            }))
        }
    }

    pub fn add_layer(&self, layer: NavMaskLayer) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        // Extract the inner NavMaskLayerData from the Arc<Mutex<NavMaskLayerData>>
        let layer_data = Arc::new(
            layer.data.lock().map_err(|_| "NavMaskLayer lock poisoned")?.clone()
        );
        data.add_layer(layer_data);
        Ok(())
    }

    pub fn with_additional_layer(&self, layer: NavMaskLayer) -> Self {
        let original_data = self.data.lock().unwrap();
        let mut new_data = original_data.clone(); // Deep clone NavMaskData
        // Extract the inner NavMaskLayerData from the Arc<Mutex<NavMaskLayerData>>
        let layer_data = Arc::new(
            layer.data.lock().unwrap().clone()
        );
        new_data.add_layer(layer_data);

        Self {
            data: Arc::new(Mutex::new(new_data))
        }
    }


    pub fn get(&self, prev: NavCell, pos: UVec3) -> Result<NavCell, String> {
        let data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        Ok(data.get(prev, pos))
    }

    pub fn translate_by(&self, offset: IVec3) -> Self {
        let original_data = self.data.lock().unwrap();
        let new_data = original_data.translate_by(offset); // Delegate to NavMaskData
        
        Self {
            data: Arc::new(Mutex::new(new_data))
        }
    }

    // Use NavMaskData::translate_by_mut for mutable translation
    pub fn translate_by_mut(&self, offset: IVec3) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.translate_by_mut(offset); // Delegate to NavMaskData
        Ok(())
    }

    pub fn clear(&self) -> Result<(), String> {
        let mut data = self.data.lock().map_err(|_| "NavMask lock poisoned")?;
        data.clear();
        Ok(())
    }
}

impl From<NavMask> for NavMaskData {
    fn from(mask: NavMask) -> Self {
        // Try to unwrap the Arc<Mutex<NavMaskData>> and extract the data
        match Arc::try_unwrap(mask.data) {
            Ok(mutex) => mutex.into_inner().unwrap(),
            Err(arc) => {
                // If there are multiple references, we need to clone
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


#[derive(Clone)]
pub struct NavMaskLayer {
    data: Arc<Mutex<NavMaskLayerData>>,
}

impl NavMaskLayer {
    pub fn new() -> Self {
        Self {
            data: Arc::new(Mutex::new(NavMaskLayerData {
                mask: HashMap::new(),
            }))
        }
    }
}


struct NavMaskLayerData {
    mask: HashMap<UVec3, NavCellMask>,
}

impl NavMaskLayerData {
    fn new() -> Self {
        Self {
            mask: HashMap::new(),
        }
    }

    fn insert_mask(&mut self, pos: UVec3, mask: NavCellMask) {
        self.mask.insert(pos, mask);
    }

    fn insert_region(&mut self, region: Region3d, mask: NavCellMask) {
        for pos in region.iter() {
            self.insert_mask(pos, mask.clone());
        }
    }

    fn insert_hashmap(&mut self, masks: &HashMap<UVec3, NavCellMask>) {
        for (pos, mask) in masks {
            self.insert_mask(*pos, mask.clone());
        }
    }

    fn insert_hashset(&mut self, cells: &HashSet<UVec3>, mask: NavCellMask) {
        for pos in cells {
            self.insert_mask(*pos, mask.clone());
        }
    }

    fn get(&self, prev: NavCell, pos: UVec3) -> NavCell {
        let mut cell = prev;
        if let Some(mask) = self.mask.get(&pos) {
            cell = process_mask(cell, mask);
        }
        cell
    }

    fn clear(&mut self) {
        self.mask.clear();
    }
}

/// ***NavMaskLayerBuilder&&&
pub struct NavMaskLayerBuilder {
    layer_data: NavMaskLayerData,
}

impl NavMaskLayerBuilder {
    pub fn new() -> Self {
        Self {
            layer_data: NavMaskLayerData {
                mask: HashMap::new(),
            }
        }
    }

    pub fn add_mask(&mut self, pos: UVec3, mask: NavCellMask) -> &mut Self {
        self.layer_data.mask.insert(pos, mask);
        self
    }

    pub fn add_region(&mut self, region: Region3d, mask: NavCellMask) -> &mut Self {
        for pos in region.iter() {
            self.layer_data.mask.insert(pos, mask.clone());
        }
        self
    }

    pub fn add_hashmap(&mut self, masks: &HashMap<UVec3, NavCellMask>) -> &mut Self {
        for (pos, mask) in masks {
            self.layer_data.mask.insert(*pos, mask.clone());
        }
        self
    }

    pub fn add_hashset(&mut self, cells: &HashSet<UVec3>, mask: NavCellMask) -> &mut Self {
        for pos in cells {
            self.layer_data.mask.insert(*pos, mask.clone());
        }
        self
    }

    pub fn build(self) -> NavMaskLayer {
        NavMaskLayer {
            data: Arc::new(self.layer_data)
        }
    }
}


/*impl NavMaskData {
    pub fn new() -> Self {
        let translation = IVec3::ZERO;

        Self { layers: Vec::new(), translation }
    }

    pub fn from_layer(layer: Arc<NavMaskLayerData>) -> Self {
        let translation = IVec3::ZERO;

        Self {
            layers: vec![layer],
            translation,
        }
    }

    pub fn with_additional_layer(&self, layer: Arc<NavMaskLayerData>) -> Self {
        let mut cloned = self.clone();
        cloned.add_layer(layer);
        cloned
    }

    pub fn add_layer(&mut self, layer: Arc<NavMaskLayerData>) -> &mut Self {
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
}*/

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


/*impl NavMaskLayerData {
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
} */
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

mod tests {
    use super::*;

    #[test]
    fn test_nav_layer() {
        let mask = NavMask::new();

        let mut builder1 = NavMaskLayerBuilder::new();
        builder1.add_region(
            Region3d::new(UVec3::new(1, 1, 1), UVec3::new(2, 2, 2)),
            NavCellMask::ImpassableOverride
        );
        let layer1 = builder1.build();

        let mut builder2 = NavMaskLayerBuilder::new();
        builder2.add_region(
            Region3d::new(UVec3::new(2, 2, 2), UVec3::new(3, 3, 3)),
            NavCellMask::PassableOverride(1)
        );
        let layer2 = builder2.build();

        let mut builder3 = NavMaskLayerBuilder::new();
        builder3.add_region(
            Region3d::new(UVec3::new(0, 0, 0), UVec3::new(3, 3, 3)), 
            NavCellMask::ModifyCost(3)
        );
        let layer3 = builder3.build();

        mask.add_layer(layer1).unwrap();
        mask.add_layer(layer2).unwrap();
        mask.add_layer(layer3).unwrap();
    }
}
