//! Navigation Masks for 
use bevy::{math::UVec3, platform::collections::HashMap};

use crate::nav::{Nav, NavCell};

pub trait NavMask {
    fn mask(&self, global_pos: UVec3) -> Option<Nav>;
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
}

impl NavMask for HashMapNavMask {
    fn mask(&self, global_pos: UVec3) -> Option<Nav> {
        self.overrides.get(&global_pos).map(|cell| cell.nav())
    }
}