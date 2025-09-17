//! Graph module for managing nodes and edges in relative space.
use bevy::{log, math::UVec3, platform::collections::HashMap};

use crate::{chunk::Chunk, dir::Dir, node::Node, path::Path, NodeId};

/// A graph structure that holds nodes and their connections (edges).
pub(crate) struct Graph {
    /// `Node` storage.
    nodes: slab::Slab<Node>,
    /// A mapping from node `UVec3` positions to their IDs in the slab.
    node_ids: HashMap<UVec3, NodeId>,
}

impl Graph {
    pub(crate) fn new() -> Self {
        Graph {
            nodes: slab::Slab::new(),
            node_ids: HashMap::new(),
        }
    }

    /// Returns the `Node` at the given position, if it exists.
    pub(crate) fn node_at(&self, pos: UVec3) -> Option<&Node> {
        self.node_ids.get(&pos).and_then(|&id| self.nodes.get(id))
    }

    /// Returns all `Node`s in the `Graph`.
    pub(crate) fn nodes(&self) -> Vec<&Node> {
        self.nodes.iter().map(|(_, node)| node).collect()
    }

    /// Returns all `Node`s that exist in the given `Chunk`.
    pub(crate) fn nodes_in_chunk(&self, chunk: &Chunk) -> Vec<&Node> {
        let nodes = self
            .nodes
            .iter()
            .filter(|(_, node)| node.chunk_index == chunk.index())
            .map(|(_, node)| node)
            .collect();

        nodes
    }

    /// Add a new `Node` to the graph at the given position with the specified `Chunk`
    /// and optional direction.
    pub(crate) fn add_node(&mut self, node: Node) -> NodeId {
        if let Some(old_id) = self.node_ids.insert(node.pos, usize::MAX) {
            self.nodes.remove(old_id);
        }

        let id = self.nodes.insert(node.clone());
        self.node_ids.insert(node.pos, id);
        id
    }

    /// Add a list of `Node`s to the graph.
    /// This will insert the nodes into the graph and update the `node_ids` mapping.
    /// If a node with the same position already exists, it will be replaced.
    /// This is useful for bulk adding nodes to the graph.
    /// The `nodes` parameter is a vector of `Node`s to be added.
    pub(crate) fn add_nodes(&mut self, nodes: &Vec<Node>) {
        for node in nodes {
            if let Some(old_id) = self.node_ids.insert(node.pos, usize::MAX) {
                self.nodes.remove(old_id);
            }

            let id = self.nodes.insert(node.clone());
            self.node_ids.insert(node.pos, id);
        }
    }

    /// Remove a `Node` from the graph at the given position.
    #[allow(dead_code)]
    pub(crate) fn remove_node(&mut self, pos: UVec3) {
        if let Some(id) = self.node_ids.remove(&pos) {
            self.nodes.remove(id);
        }
    }

    /// Remove all nodes for edge directions in the Chunk.
    /// This is used to clean up nodes that are no longer needed
    /// when edges are rebuilt.
    pub(crate) fn remove_nodes_for_edges(&mut self, chunk: &Chunk, dirs: &[Dir]) {
        // Collect all nodes for all specified edges
        let edge_nodes: Vec<(UVec3, NodeId)> = self
            .nodes_in_chunk(chunk)
            .iter()
            .filter(|node| node.dir.is_some() && dirs.contains(&node.dir.unwrap()))
            .filter_map(|node| self.node_ids.get(&node.pos).map(|&id| (node.pos, id)))
            .collect();

        // Remove nodes from slab and mapping
        for (pos, id) in &edge_nodes {
            if self.nodes.contains(*id) {
                self.nodes.remove(*id);
                self.node_ids.remove(pos);
            } else {
                log::warn!(
                    "Tried to remove node {:?} with id {:?} from Chunk {:?}, but it was already gone",
                    pos, id, chunk
                );
            }
        }

        // Remove edges from all nodes pointing to removed positions
        let removed_positions: Vec<UVec3> = edge_nodes.iter().map(|(pos, _)| *pos).collect();
        for node in self.nodes.iter_mut() {
            node.1.remove_edges_to_positions(&removed_positions);
        }
    }

    /*pub(crate) fn remove_nodes_at_edge(&mut self, chunk: &Chunk, dir: Dir) {
        // Collect all nodes in the edge and their IDs
        let edge_nodes: Vec<(UVec3, NodeId)> = self
            .nodes_in_chunk(chunk)
            .iter()
            .filter(|node| node.dir == Some(dir))
            .filter_map(|node| self.node_ids.get(&node.pos).map(|&id| (node.pos, id)))
            .collect();

        // Remove all nodes from the slab and node_ids mapping
        for (pos, id) in &edge_nodes {
            if self.nodes.contains(*id) {
                self.nodes.remove(*id);
                self.node_ids.remove(pos);
            } else {
                log::warn!("Tried to remove node {:?} with id {:?} from Chunk {:?}, but it was already gone", pos, id, chunk);
            }
        }

        // Remove any edges in other nodes that point to the removed positions
        let positions: Vec<UVec3> = edge_nodes.iter().map(|(pos, _)| *pos).collect();
        for node in self.nodes.iter_mut() {
            node.1.remove_edges_to_positions(&positions);
        }
    }*/

    pub(crate) fn remove_edges_for_chunk(&mut self, chunk: &Chunk) {
        // Get all the ndoes in the chunk
        let nodes_in_chunk = self.nodes_in_chunk(chunk);
        // Collect all positions in the chunk
        let positions: Vec<UVec3> = nodes_in_chunk.iter().map(|node| node.pos).collect();
        // Now iterate over all nodes in the chunk and remove edges to those positions
        for node in self.nodes.iter_mut() {
            node.1.remove_edges_to_positions(&positions);
        }
    }

    /// Connect two nodes in the graph with a provided `Path`.
    /// The path will be used as a cached path between the two nodes.
    pub(crate) fn connect_node(&mut self, from: UVec3, to: UVec3, path: Path) {
        if let Some(&from_id) = self.node_ids.get(&from) {
            self.nodes.get_mut(from_id).unwrap().edges.insert(to, path);
        }
    }

    /// Returns the cost of the edge between two node positions.
    pub(crate) fn edge_cost(&self, from: UVec3, to: UVec3) -> Option<u32> {
        self.node_ids
            .get(&from)
            .and_then(|&from_id| self.nodes.get(from_id))
            .and_then(|node| node.edges.get(&to).map(|path| path.cost()))
    }

    /// Returns the cached edge path
    pub(crate) fn edge_path(&self, from: UVec3, to: UVec3) -> Option<&Path> {
        self.node_ids
            .get(&from)
            .and_then(|&from_id| self.nodes.get(from_id))
            .and_then(|node| node.edges.get(&to))
    }

    /// Returns all cached `Path`s in the graph.
    pub(crate) fn all_paths(&self) -> Vec<Path> {
        let mut paths = Vec::new();
        for node in self.nodes.iter().map(|(_, node)| node) {
            for path in node.edges.values() {
                paths.push(path.clone());
            }
        }

        paths
    }

    #[allow(dead_code)]
    pub(crate) fn validate_slab(&self) {
        for (pos, id) in &self.node_ids {
            if !self.nodes.contains(*id) {
                log::error!(
                    "validate: node_ids maps pos {:?} to id {:?}, but slab has no entry!",
                    pos,
                    id
                );
            } else {
                let node = &self.nodes[*id];
                if node.pos != *pos {
                    log::error!(
                        "validate: node_ids maps pos {:?} to id {:?}, but slab node has pos {:?}!",
                        pos,
                        id,
                        node.pos
                    );
                }
            }
        }

        for (id, node) in self.nodes.iter() {
            if let Some(mapped_id) = self.node_ids.get(&node.pos) {
                if *mapped_id != id {
                    log::error!(
                        "validate: slab node at id {:?} has pos {:?}, but node_ids maps to different id {:?}",
                        id,
                        node.pos,
                        mapped_id
                    );
                }
            } else {
                log::error!(
                    "validate: slab has node at pos {:?}, but node_ids has no entry",
                    node.pos
                );
            }
        }
    }

    /// Returns the number of nodes in the graph.
    #[allow(dead_code)]
    pub(crate) fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the number of edges in the graph.
    #[allow(dead_code)]
    pub(crate) fn edge_count(&self) -> usize {
        self.nodes.iter().map(|(_, node)| node.edges.len()).sum()
    }

    /// Ensure that there are no `Node`s with duplicate positions in the graph.
    #[allow(dead_code)]
    pub(crate) fn clear_duplicates(&mut self) {
        let mut to_remove = Vec::new();
        for (id, node) in self.nodes.iter() {
            if let Some(&other_id) = self.node_ids.get(&node.pos) {
                if id != other_id {
                    to_remove.push(id);
                }
            }
        }

        for id in to_remove {
            self.nodes.remove(id);
        }
    }

    /// Returns the closest `Node` to the given position in the specified `Chunk`.
    #[allow(dead_code)]
    pub(crate) fn closest_node_in_chunk(&self, pos: UVec3, chunk: &Chunk) -> Option<&Node> {
        let node = self
            .nodes_in_chunk(chunk)
            .iter()
            .min_by_key(|node| {
                let dx = (node.pos.x as i32 - pos.x as i32).abs();
                let dy = (node.pos.y as i32 - pos.y as i32).abs();
                let dz = (node.pos.z as i32 - pos.z as i32).abs();
                (dx + dy + dz) as u32
            })
            .cloned();

        node
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_node() {
        let mut graph = Graph::new();
        let pos = UVec3::new(0, 0, 0);
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16));
        let id = graph.add_node(Node::new(pos, chunk.clone(), None));

        let node = graph.node_at(pos).unwrap();
        assert_eq!(node.pos, pos);
        assert_eq!(node.chunk_index, chunk.index());
        assert_eq!(node.dir, None);
        assert_eq!(id, 0);
    }

    #[test]
    fn test_all_nodes_in_chunk() {
        let mut graph = Graph::new();
        let pos1 = UVec3::new(0, 0, 0);
        let pos2 = UVec3::new(1, 1, 1);
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16));
        graph.add_node(Node::new(pos1, chunk.clone(), None));
        graph.add_node(Node::new(pos2, chunk.clone(), None));

        let nodes = graph.nodes_in_chunk(&chunk);
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes[0].pos, pos1);
        assert_eq!(nodes[1].pos, pos2);
    }

    #[test]
    fn test_closest_node_in_chunk() {
        let mut graph = Graph::new();
        let pos1 = UVec3::new(0, 0, 0);
        let pos2 = UVec3::new(1, 1, 1);
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(16, 16, 16));
        graph.add_node(Node::new(pos1, chunk.clone(), None));
        graph.add_node(Node::new(pos2, chunk.clone(), None));

        let pos = UVec3::new(0, 1, 0);
        let node = graph.closest_node_in_chunk(pos, &chunk).unwrap();
        assert_eq!(node.pos, pos1);
    }
}
