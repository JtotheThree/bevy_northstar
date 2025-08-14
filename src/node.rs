//! This module defines the `Node` struct, which represents a node in the graph.
use bevy::{math::UVec3, platform::collections::HashMap};
use std::hash::{Hash, Hasher};

use crate::{chunk::Chunk, dir::Dir, path::Path};

/// A `Node` for use in `Graph`.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Node {
    /// The position of the node in space.
    pub(crate) pos: UVec3,
    /// The chunk that this node belongs to.
    pub(crate) chunk_index: (usize, usize, usize),
    /// Edges are the other nodes that this node is connected to and the path to them.
    pub(crate) edges: HashMap<UVec3, Path>,
    /// The direction of the edge relative to the chunk.
    pub(crate) dir: Option<Dir>,
    /// If this is a portal
    pub(crate) portal: bool,
}

impl Node {
    pub(crate) fn new(pos: UVec3, chunk: Chunk, dir: Option<Dir>) -> Self {
        Node {
            pos,
            chunk_index: chunk.index(),
            dir,
            edges: HashMap::new(),
            portal: false,
        }
    }

    /// Returns all positions that are connected to this `Node`.
    pub(crate) fn edges(&self) -> Vec<UVec3> {
        self.edges.keys().cloned().collect()
    }

    pub(crate) fn remove_edges_to_positions(&mut self, positions: &[UVec3]) {
        for pos in positions {
            self.edges.remove(pos);
        }
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.pos == other.pos
    }
}

impl Eq for Node {}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.pos.hash(state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_eq() {
        let node1 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );
        let node2 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );

        assert_eq!(node1, node2);
    }

    #[test]
    fn test_node_hash() {
        let node1 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );
        let node2 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );

        let mut hasher1 = std::collections::hash_map::DefaultHasher::new();
        let mut hasher2 = std::collections::hash_map::DefaultHasher::new();

        node1.hash(&mut hasher1);
        node2.hash(&mut hasher2);

        assert_eq!(hasher1.finish(), hasher2.finish());
    }
}
