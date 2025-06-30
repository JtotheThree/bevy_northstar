//! This module contains the `Grid` component which is the main component for the crate.
use std::sync::Arc;

use bevy::{
    log,
    math::{IVec3, UVec3},
    platform::collections::HashMap,
    prelude::{Component, Entity},
};
use ndarray::{s, Array3, ArrayView2, ArrayView3};

use crate::{
    chunk::Chunk,
    dijkstra::*,
    dir::*,
    filter::NeighborFilter,
    graph::Graph,
    nav::{Nav, NavCell},
    neighbor::Neighborhood,
    node::Node,
    path::Path,
    pathfind::{pathfind, pathfind_astar, reroute_path},
    position_in_cubic_window, MovementCost,
};

/// Settings for how the grid is divided into chunks.
#[derive(Copy, Clone, Debug)]
pub struct ChunkSettings {
    /// The square size of each chunk in the grid.
    /// Needs to be at least 3.
    pub size: u32,
    /// The depth of each chunk in the grid when using 3D grids.
    pub depth: u32,
    /// If true, allows corner connections between chunks.
    /// This will increase the time it takes to build the grid.
    /// It generally isn't recommended as the path refinement step should handle corners already unless you have a noisy tilemap.
    pub diagonal_connections: bool,
}

impl Default for ChunkSettings {
    fn default() -> Self {
        ChunkSettings {
            size: 16,
            depth: 1,
            diagonal_connections: false,
        }
    }
}

/// Defaults movement cost and passability for initializing the grid cells.
/// Useful if you're generating your a large map to reduce your initialization time.
#[derive(Copy, Clone, Debug)]
pub struct NavSettings {
    /// The default cost for each cell in the grid.
    pub default_movement_cost: MovementCost,
    /// If true, the default cells will be solid and block movement.
    pub default_impassible: bool,
}

impl Default for NavSettings {
    fn default() -> Self {
        NavSettings {
            default_movement_cost: 1,
            default_impassible: false,
        }
    }
}

/// Settings for collision
#[derive(Copy, Clone, Debug)]
pub struct CollisionSettings {
    /// If true, collision avoidance is enabled.
    pub enabled: bool,
    /// The plugin systems use collision avoidance, this is the look ahead distance for the path to check for blocking entities.
    pub avoidance_distance: u32,
}

impl Default for CollisionSettings {
    fn default() -> Self {
        CollisionSettings {
            enabled: false,
            avoidance_distance: 4,
        }
    }
}

/// Settings for filtering determined neighbors.
#[derive(Clone, Default)]
pub struct NeighborhoodSettings {
    pub filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

/// Holder for internal crate settings.
pub struct GridSettings(pub(crate) GridInternalSettings);

/// Builder for `GridSettings`.
///
/// Example usage:
/// ```
/// use bevy_northstar::prelude::*;
///
/// let grid_settings = GridSettingsBuilder::new_2d(128, 128)
///     .chunk_size(16)
///     .default_impassable()
///     .add_neighbor_filter(filter::NoCornerClipping)
///     .enable_collision()
///     .avoidance_distance(5)
///     .build();
///
/// let grid: Grid<OrdinalNeighborhood> = Grid::new(&grid_settings);
/// ```
///
#[derive(Clone)]
pub struct GridSettingsBuilder {
    dimensions: UVec3,
    chunk_settings: ChunkSettings,
    cost_settings: NavSettings,
    collision_settings: CollisionSettings,
    neighborhood_settings: NeighborhoodSettings,
}

impl Default for GridSettingsBuilder {
    fn default() -> Self {
        GridSettingsBuilder {
            dimensions: UVec3::new(64, 64, 1),
            chunk_settings: ChunkSettings::default(),
            cost_settings: NavSettings::default(),
            collision_settings: CollisionSettings::default(),
            neighborhood_settings: NeighborhoodSettings::default(),
        }
    }
}

impl GridSettingsBuilder {
    /// Initalize a 2D [`Grid`] with the given width and height. Returns a [`GridSettingsBuilder`] that can be further configured.
    pub fn new_2d(width: u32, height: u32) -> Self {
        if width < 3 || height < 3 {
            panic!("Width and height must be at least 3");
        }

        let mut grid_settings = GridSettingsBuilder {
            dimensions: UVec3::new(width, height, 1),
            ..Default::default()
        };

        grid_settings.chunk_settings.depth = 1;

        grid_settings
    }

    /// Initalize a 3D [`Grid`] with the given width, height, and depth. Returns a [`GridSettingsBuilder`] that can be further configured.
    pub fn new_3d(width: u32, height: u32, depth: u32) -> Self {
        if width < 3 || height < 3 {
            panic!("Width and height must be at least 3");
        }

        if depth < 1 {
            panic!("Depth must be at least 1");
        }

        GridSettingsBuilder {
            dimensions: UVec3::new(width, height, depth),
            ..Default::default()
        }
    }

    /// Size of each square chunk the grid is divided into.
    /// Must be at least 3.
    pub fn chunk_size(mut self, chunk_size: u32) -> Self {
        if chunk_size < 3 {
            panic!("Chunk size must be at least 3");
        }

        self.chunk_settings.size = chunk_size;
        self
    }

    /// Depth (Z) of each chunk in the grid when using 3D grids.
    /// Must be at least 1 and the grid's depth must be divisible by the chunk depth.
    pub fn chunk_depth(mut self, chunk_depth: u32) -> Self {
        if chunk_depth < 1 {
            panic!("Chunk depth must be at least 1");
        }

        //FIXME: User should be able to set the chunk depth so there's no z chunking but still allow x/y chunking
        // Right now we need dimension.z to be divisible by chunk_depth
        if self.dimensions.z % chunk_depth != 0 {
            panic!("Depth must be divisible by chunk depth");
        }

        self.chunk_settings.depth = chunk_depth;
        self
    }

    /// Enabling this will create extra HPA connections in the corners of the chunks if passable.
    /// This can create better paths in some cases but at the cost of performance.
    /// If you're only using refined HPA* (default) paths then this likely isn't needed.
    pub fn enable_diagonal_connections(mut self) -> Self {
        self.chunk_settings.diagonal_connections = true;
        self
    }

    /// Default movement cost for each cell in the grid.
    pub fn default_movement_cost(mut self, default_movement_cost: MovementCost) -> Self {
        self.cost_settings.default_movement_cost = default_movement_cost;
        self
    }

    /// Sets the default cells in the grid to be impassable.
    pub fn default_impassable(mut self) -> Self {
        self.cost_settings.default_impassible = true;
        self
    }

    /// Enables collision avoidance in the [`crate::plugin::NorthstarPlugin`] pathfinding systems.
    pub fn enable_collision(mut self) -> Self {
        self.collision_settings.enabled = true;
        self
    }

    /// The look ahead distance for collision avoidance.
    /// No need to set this if collision is not enabled.
    pub fn avoidance_distance(mut self, distance: u32) -> Self {
        self.collision_settings.avoidance_distance = distance;
        self
    }

    /// Adds a [`NeighborFilter`] to the grid settings to be applied when computing neighbors.
    /// This lets you apply custom movement rules based on grid cell state.
    /// Multiple filters can be added and will be applied in the order they are added.
    pub fn add_neighbor_filter<F>(mut self, filter: F) -> Self
    where
        F: NeighborFilter + Send + Sync + 'static,
    {
        self.neighborhood_settings.filters.push(Arc::new(filter));
        self
    }

    /// Pass in a [`ChunkSettings`] to configure the grid's chunking behavior.
    /// Or use the individual methods [`GridSettingsBuilder::chunk_size()`] and [`GridSettingsBuilder::chunk_depth()`] to set the chunk size and depth individually.
    pub fn chunk_settings(mut self, chunk_settings: ChunkSettings) -> Self {
        self = self.chunk_size(chunk_settings.size);
        self = self.chunk_depth(chunk_settings.depth);

        if chunk_settings.diagonal_connections {
            self = self.enable_diagonal_connections();
        }

        self
    }

    /// Pass in [`NavSettings`] to configure the grid's default cell navigation data.
    /// You can also use the individual methods [`GridSettingsBuilder::default_movement_cost()`] and [`GridSettingsBuilder::default_impassable()`].
    pub fn nav_settings(mut self, nav_settings: NavSettings) -> Self {
        self = self.default_movement_cost(nav_settings.default_movement_cost);

        if nav_settings.default_impassible {
            self = self.default_impassable();
        }

        self
    }

    /// Pass in a [`CollisionSettings`] to configure the grid's collision avoidance behavior.
    /// You can also use the individual methods [`GridSettingsBuilder::enable_collision()`] and [`GridSettingsBuilder::avoidance_distance()`].
    pub fn collision_settings(mut self, collison_settings: CollisionSettings) -> Self {
        self.collision_settings = collison_settings;
        self
    }

    /// Pass in a [`NeighborhoodSettings`] to configure the grid's neighborhood behavior.
    /// You can also use the individual method [`GridSettingsBuilder::add_neighbor_filter()`] to add a filter.
    pub fn neighborhood_settings(mut self, neighborhood_settings: NeighborhoodSettings) -> Self {
        self.neighborhood_settings = neighborhood_settings;
        self
    }

    /// Builds the [`GridSettings`] from the current builder state.
    /// Call this after you've configured the builder to your liking
    /// and then pass the resulting [`GridSettings`] to the [`Grid::new()`] method.
    pub fn build(self) -> GridSettings {
        GridSettings(GridInternalSettings {
            dimensions: self.dimensions,
            chunk_settings: self.chunk_settings,
            cost_settings: self.cost_settings,
            collision_settings: self.collision_settings,
            neighborhood_settings: self.neighborhood_settings,
        })
    }
}

#[derive(Clone)]
pub(crate) struct GridInternalSettings {
    pub(crate) dimensions: UVec3,
    pub(crate) chunk_settings: ChunkSettings,
    pub(crate) cost_settings: NavSettings,
    pub(crate) collision_settings: CollisionSettings,
    pub(crate) neighborhood_settings: NeighborhoodSettings,
}

impl Default for GridInternalSettings {
    fn default() -> Self {
        GridSettingsBuilder::default().build().0
    }
}

/// `Grid` is the main `Resource` struct for the crate.
///
/// # Example
/// ```
/// use bevy::prelude::*;
/// use bevy_northstar::prelude::*;
///
/// fn main() {
///     App::new()
///        .add_plugins(DefaultPlugins)
///        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
///        .add_systems(Startup, startup);
/// }
///
/// fn startup(mut commands: Commands) {
///    // Populate the grid with, you'd usually do this after you load your tilemap
///    // and then set the cells in the grid to match the tilemap.
///
///    let grid_settings = GridSettingsBuilder::new_2d(64, 64)
///       .chunk_size(16)
///       .enable_collision()
///       .build();
///
///    let mut grid: Grid<CardinalNeighborhood> = Grid::new(&grid_settings);
///
///    // Set the cell at (0, 0, 0) to be passable with a cost of 1
///    grid.set_nav(UVec3::new(0, 0, 0), Nav::Passable(1));
///    // Set the cell at (1, 0, 0) to be impassable
///    grid.set_nav(UVec3::new(1, 0, 0), Nav::Impassable);
///    // Initialize the grid
///    grid.build();
///
///    // Add the grid to the world
///    commands.spawn(grid);
/// }
/// ```
#[derive(Component)]
pub struct Grid<N: Neighborhood> {
    pub(crate) neighborhood: N,

    dimensions: UVec3,
    chunk_settings: ChunkSettings,
    collision_settings: CollisionSettings,

    grid: Array3<NavCell>,
    chunks: Array3<Chunk>,

    graph: Graph,

    dirty: bool,
}

impl<N: Neighborhood + Default> Grid<N> {
    /// Creates a new [`Grid`] instance with the given [`GridSettings`].
    /// Use the [`GridSettingsBuilder`] to generate the settings.
    pub fn new(settings: &GridSettings) -> Self {
        let GridInternalSettings {
            dimensions,
            chunk_settings,
            cost_settings,
            collision_settings,
            neighborhood_settings: _,
        } = settings.0;

        let UVec3 { x, y, z } = dimensions;

        let x_chunks = x / chunk_settings.size;
        let y_chunks = y / chunk_settings.size;
        let z_chunks = z / chunk_settings.depth;

        let default_navcell = if cost_settings.default_impassible {
            NavCell::new(Nav::Impassable)
        } else {
            NavCell::new(Nav::Passable(cost_settings.default_movement_cost))
        };

        let grid = Array3::from_elem((x as usize, y as usize, z as usize), default_navcell);

        let chunks = Array3::from_shape_fn(
            (x_chunks as usize, y_chunks as usize, z_chunks as usize),
            |(x, y, z)| {
                let min_x = x as u32 * chunk_settings.size;
                let max_x = min_x + chunk_settings.size - 1;
                let min_y = y as u32 * chunk_settings.size;
                let max_y = min_y + chunk_settings.size - 1;
                let min_z = z as u32 * chunk_settings.depth;
                let max_z = min_z + chunk_settings.depth - 1;

                Chunk::new(
                    UVec3::new(min_x, min_y, min_z),
                    UVec3::new(max_x, max_y, max_z),
                )
            },
        );

        Self {
            neighborhood: N::from_settings(&settings.0.neighborhood_settings),
            dimensions,
            chunk_settings,
            collision_settings,

            grid,
            chunks,

            graph: Graph::new(),

            dirty: true,
        }
    }

    /// Returns the neighborhood used by this grid.
    /// Useful if you want to call [`Neighborhood::is_ordinal()`] or similar methods.
    pub fn neighborhood(&self) -> &N {
        &self.neighborhood
    }

    /// Returns an [`ndarray::ArrayView3<NavCell>`] for read-only access to the grid data.
    pub fn view(&self) -> ArrayView3<NavCell> {
        self.grid.view()
    }

    /// Returns an [`ndarray::ArrayView3<NavCell>`] for read-only access to the data within a given [`Chunk`].
    pub(crate) fn chunk_view(&self, chunk: &Chunk) -> ArrayView3<NavCell> {
        chunk.view(&self.grid)
    }

    /// Returns the [`Graph`] associated with this grid.
    pub(crate) fn graph(&self) -> &Graph {
        &self.graph
    }

    /// Test if a grid cell is passable at a given [`bevy::math::UVec3`] position.
    pub fn is_passable(&self, pos: UVec3) -> bool {
        if !self.in_bounds(pos) {
            return false;
        }

        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].is_passable()
    }

    /// Test if a grid cell is a ramp at a given [`bevy::math::UVec3`] position.
    pub fn is_ramp(&self, pos: UVec3) -> bool {
        if !self.in_bounds(pos) {
            return false;
        }

        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].is_ramp()
    }

    /// Set the [`Nav`] settings at a given [`bevy::math::UVec3`] position in the grid.
    pub fn set_nav(&mut self, pos: UVec3, nav: Nav) {
        if !self.in_bounds(pos) {
            panic!("Attempted to set nav at out-of-bounds position");
        }

        if !self.dirty {
            log::warn!("The Grid state is dirty, you either forgot to call `build()` on grid or you need to call `build()` after modifying the grid. In the future partial rebuilding will be supported.");
        }

        let navcell = NavCell::new(nav);
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]] = navcell;
    }

    /// Gets the [`Nav`] settings at a given [`bevy::math::UVec3`] position in the grid.
    pub fn nav(&self, pos: UVec3) -> Option<Nav> {
        if self.in_bounds(pos) {
            Some(self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].nav())
        } else {
            None
        }
    }

    /// Gets the [`NavCell`] at a given [`bevy::math::UVec3`] position in the grid.
    pub(crate) fn navcell(&self, pos: UVec3) -> &NavCell {
        &self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]]
    }

    /// Returns the dimensions of the grid.
    pub fn dimensions(&self) -> UVec3 {
        self.dimensions
    }

    /// Returns the width of the grid.
    pub fn width(&self) -> u32 {
        self.dimensions.x
    }

    /// Returns the height of the grid.
    pub fn height(&self) -> u32 {
        self.dimensions.y
    }

    /// Returns the depth of the grid.
    pub fn depth(&self) -> u32 {
        self.dimensions.z
    }

    /// Returns the square width/height dimensions of the chunks in the grid.
    pub fn chunk_size(&self) -> u32 {
        self.chunk_settings.size
    }

    /// Returns the chunk settings of the grid.
    pub fn chunk_settings(&self) -> ChunkSettings {
        self.chunk_settings
    }

    /// Test if collision is enabled.
    pub fn collision(&self) -> bool {
        self.collision_settings.enabled
    }

    /// Set the collision settings for the grid.
    pub fn set_collision(&mut self, collision: bool) {
        self.collision_settings.enabled = collision;
    }

    /// Returns the avoidance distance for collision avoidance
    pub fn avoidance_distance(&self) -> u32 {
        self.collision_settings.avoidance_distance
    }

    /// Set the avoidance distance for collision avoidance
    pub fn set_avoidance_distance(&mut self, distance: u32) {
        self.collision_settings.avoidance_distance = distance;
    }

    /// Checks if a position is within the bounds of the grid.
    pub fn in_bounds(&self, pos: UVec3) -> bool {
        pos.x < self.dimensions.x && pos.y < self.dimensions.y && pos.z < self.dimensions.z
    }

    /// Builds the entire grid. This includes precomputing neighbors, creating nodes for each edge of each chunk,
    /// caching paths between internal nodes within each chunk, and connecting adjacent nodes between chunks.
    /// This method needs to be called after the grid has been initialized.
    pub fn build(&mut self) {
        self.precompute_neighbors();
        self.build_nodes();
        self.connect_internal_chunk_nodes();
        self.connect_adjacent_chunk_nodes();
        self.dirty = false;
    }

    pub(crate) fn precompute_neighbors(&mut self) {
        let grid_view = self.grid.view();

        // Collect positions and neighbor bits first to avoid borrow checker issues
        let mut neighbor_bits_vec = Vec::with_capacity(self.grid.len());
        for ((x, y, z), _cell) in self.grid.indexed_iter() {
            let pos = UVec3::new(x as u32, y as u32, z as u32);
            let bits = self.neighborhood.neighbors(&grid_view, pos);
            neighbor_bits_vec.push(((x, y, z), bits));
        }

        // Now apply the neighbor bits with mutable access
        for &((x, y, z), bits) in &neighbor_bits_vec {
            self.grid[[x, y, z]].neighbor_bits = bits;
        }
    }

    // Populates the graph with nodes for each edge of each chunk.
    fn build_nodes(&mut self) {
        let chunk_size = self.chunk_settings.size as usize;
        let chunk_depth = self.chunk_settings.depth as usize;

        let x_chunks = self.dimensions.x as usize / chunk_size;
        let y_chunks = self.dimensions.y as usize / chunk_size;
        let z_chunks = self.dimensions.z as usize / chunk_depth;

        for x in 0..x_chunks {
            for y in 0..y_chunks {
                for z in 0..z_chunks {
                    let current_chunk = &self.chunks[[x, y, z]];

                    let directions = Dir::cardinal();

                    for dir in directions {
                        let dir_vec = dir.vector();

                        let nx = x as i32 + dir_vec.0;
                        let ny = y as i32 + dir_vec.1;
                        let nz = z as i32 + dir_vec.2;

                        // Ensure neighbor is within bounds
                        if nx >= 0
                            && nx < x_chunks as i32
                            && ny >= 0
                            && ny < y_chunks as i32
                            && nz >= 0
                            && nz < z_chunks as i32
                        {
                            let neighbor_chunk =
                                &self.chunks[[nx as usize, ny as usize, nz as usize]];

                            let current_edge = current_chunk.edge(&self.grid, dir);
                            let neighbor_edge = neighbor_chunk.edge(&self.grid, dir.opposite());

                            let mut nodes = self.calculate_edge_nodes(
                                current_edge,
                                neighbor_edge,
                                current_chunk.clone(),
                                dir,
                            );

                            // Position the nodes in world space using the fixed axis to realign the nodes,
                            // they also need to be adjusted by the position of the chunk in the grid
                            for node in nodes.iter_mut() {
                                node.pos = match dir {
                                    Dir::NORTH => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.max().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    Dir::EAST => UVec3::new(
                                        node.pos.y + current_chunk.max().x,
                                        node.pos.x + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    Dir::SOUTH => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    Dir::WEST => UVec3::new(
                                        node.pos.y + current_chunk.min().x,
                                        node.pos.x + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    // TODO: WE NEED TO FIX UP AND DOWN NODE STUFFS
                                    Dir::UP => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.min().y,
                                        node.pos.z + current_chunk.max().z,
                                    ),
                                    Dir::DOWN => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    _ => panic!("Invalid direction"),
                                }
                            }

                            self.graph.add_nodes(&nodes);
                        }
                    }

                    // Handle ordinal connections if enabled
                    if self.chunk_settings.diagonal_connections {
                        let ordinal_directions = Dir::ordinal();

                        for dir in ordinal_directions {
                            let dir_vec = dir.vector();

                            let nx = x as i32 + dir_vec.0;
                            let ny = y as i32 + dir_vec.1;
                            let nz = z as i32 + dir_vec.2;

                            // Ensure neighbor is within bounds
                            if nx >= 0
                                && nx < x_chunks as i32
                                && ny >= 0
                                && ny < y_chunks as i32
                                && nz >= 0
                                && nz < z_chunks as i32
                            {
                                let neighbor_chunk =
                                    &self.chunks[[nx as usize, ny as usize, nz as usize]];

                                let current_corner = current_chunk.corner(&self.grid, dir);
                                let neighbor_corner =
                                    neighbor_chunk.corner(&self.grid, dir.opposite());

                                if current_corner.is_impassable() || neighbor_corner.is_impassable()
                                {
                                    continue;
                                }

                                let pos = match dir {
                                    Dir::NORTHEAST => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHEAST => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHWEST => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::NORTHWEST => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::NORTHEASTUP => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.max().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::SOUTHEASTUP => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.min().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::SOUTHWESTUP => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.min().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::NORTHWESTUP => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.max().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::NORTHEASTDOWN => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHEASTDOWN => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHWESTDOWN => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::NORTHWESTDOWN => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    _ => panic!("Invalid direction"),
                                };

                                self.graph.add_node(pos, current_chunk.clone(), Some(dir));
                            }
                        }
                    }
                }
            }
        }
    }

    // Calculates the edge nodes for a given edge in the grid.
    fn calculate_edge_nodes(
        &self,
        start_edge: ArrayView2<NavCell>,
        end_edge: ArrayView2<NavCell>,
        chunk: Chunk,
        dir: Dir,
    ) -> Vec<Node> {
        let mut nodes = Vec::new();

        // Iterate over the start edge and find connections that are walkable
        for ((start_x, start_y), start_cell) in start_edge.indexed_iter() {
            if start_cell.is_impassable() {
                continue;
            }

            let end_x = start_x;
            let mut end_y = start_y;

            if start_cell.is_ramp() {
                end_y = start_y + 1;
            }

            let end_cell = end_edge[[end_x, end_y]].clone();

            // If the end cell is impassable, we can't connect the nodes
            if !end_cell.is_impassable() {
                let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                let node = Node::new(pos, chunk.clone(), Some(dir));
                nodes.push(node);
            }
        }

        // Split nodes into groups of continous nodes
        let continous_with = |start: UVec3, end: UVec3| {
            let x_diff = (start.x as i32 - end.x as i32).abs();
            let y_diff = (start.y as i32 - end.y as i32).abs();

            x_diff <= 1 && y_diff <= 1
        };

        let mut split_nodes = Vec::new();
        let mut current_group: Vec<Node> = Vec::new();

        for node in nodes {
            if let Some(last_node) = current_group.last() {
                if !continous_with(last_node.pos, node.pos) {
                    split_nodes.push(current_group);
                    current_group = Vec::new();
                }
            }
            current_group.push(node);
        }

        if !current_group.is_empty() {
            split_nodes.push(current_group);
        }

        // Find the center of the split nodes
        let mut final_nodes = Vec::new();

        for group in &split_nodes {
            /*if group.len() > 8 {
                final_nodes.push(group.first().unwrap().clone());
                final_nodes.push(group.last().unwrap().clone());
            } else {*/

            let middle = group.len() / 2;
            final_nodes.push(group[middle].clone());
            //}
        }

        if !final_nodes.is_empty() {
            return final_nodes;
        }

        if !self.neighborhood.is_ordinal() {
            return final_nodes;
        }

        let mut ordinal_nodes = Vec::new();

        // If we made it here that means no connection nodes were found, but we allow ordinal connections so let's build those if possible
        for ((start_x, start_y), start_cell) in start_edge.indexed_iter() {
            if start_cell.is_impassable() {
                continue;
            }

            let end_x = start_x;
            let mut end_y = start_y;

            if start_cell.is_ramp() {
                end_y = start_y + 1;
            }

            if end_x > 0 {
                let left = end_edge[[end_x - 1, end_y]].clone();
                if !left.is_impassable() {
                    let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                    let node = Node::new(pos, chunk.clone(), Some(dir));
                    ordinal_nodes.push(node);
                }
            }

            if end_x < end_edge.shape()[0] - 1 {
                let right = end_edge[[end_x + 1, end_y]].clone();
                if !right.is_impassable() {
                    let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                    let node = Node::new(pos, chunk.clone(), Some(dir));
                    ordinal_nodes.push(node);
                }
            }
        }

        ordinal_nodes
    }

    // Connects the internal nodes of each chunk to each other.
    fn connect_internal_chunk_nodes(&mut self) {
        let chunk_size = self.chunk_settings.size as usize;
        let chunk_depth = self.chunk_settings.depth as usize;

        let x_chunks = self.dimensions.x as usize / chunk_size;
        let y_chunks = self.dimensions.y as usize / chunk_size;
        let z_chunks = self.dimensions.z as usize / chunk_depth;

        for x in 0..x_chunks {
            for y in 0..y_chunks {
                for z in 0..z_chunks {
                    // Connect internal nodes

                    let chunk_grid = self.chunks[[x, y, z]].view(&self.grid);
                    let chunk = &self.chunks[[x, y, z]];

                    let nodes = self.graph.nodes_in_chunk(chunk.clone());

                    let mut connections = Vec::new();

                    for node in nodes.iter() {
                        // Collect other nodes positions into an array
                        let other_nodes = nodes
                            .iter()
                            .filter(|other_node| other_node.pos != node.pos)
                            .map(|other_node| other_node.pos)
                            .collect::<Vec<_>>();

                        // Adjust node.pos by the chunk position
                        let start_pos = node.pos - chunk.min();

                        let goals = other_nodes
                            .iter()
                            .map(|pos| *pos - chunk.min())
                            .collect::<Vec<_>>();

                        let paths = dijkstra_grid(
                            &chunk_grid,
                            start_pos,
                            &goals,
                            false,
                            100,
                            &HashMap::new(),
                        );

                        // Readjust position to world space and then connect the nodes
                        for (goal_pos, path) in paths {
                            let start = node.pos;
                            let goal = goal_pos + chunk.min();

                            // Readjust path to world space
                            let path_vec = path
                                .path()
                                .iter()
                                .map(|pos| *pos + chunk.min())
                                .collect::<Vec<_>>();

                            connections.push((
                                start,
                                goal,
                                Path::new(path_vec.clone(), path_vec.len() as u32),
                            ));
                        }
                    }

                    for (node_pos, other_node_pos, path) in connections {
                        self.graph.connect_node(node_pos, other_node_pos, path);
                    }
                }
            }
        }
    }

    // Connects the nodes of adjacent chunks to each other.
    fn connect_adjacent_chunk_nodes(&mut self) {
        let nodes = self.graph.nodes();

        let mut connections = Vec::new();

        for node in nodes {
            // Check all the adjacent positions of the node, taking into account cardinal/ordinal settings
            let directions = if self.chunk_settings.diagonal_connections {
                Dir::all()
            } else {
                Dir::cardinal()
            };

            for dir in directions {
                let dir_vec = dir.vector();

                let nx = node.pos.x as i32 + dir_vec.0;
                let ny = node.pos.y as i32 + dir_vec.1;
                let nz = node.pos.z as i32 + dir_vec.2;

                if let Some(neighbor) = self
                    .graph
                    .node_at(UVec3::new(nx as u32, ny as u32, nz as u32))
                {
                    // Check if neighbor is in a different chunk
                    if node.chunk != neighbor.chunk {
                        let path = Path::from_slice(&[node.pos, neighbor.pos], 1);

                        connections.push((node.pos, neighbor.pos, path));
                    }
                }
            }
        }

        for (node_pos, neighbor_pos, path) in connections {
            self.graph.connect_node(node_pos, neighbor_pos, path);
        }
    }

    /// Returns the `Chunk` for the given position `UVec3` in the grid.
    pub(crate) fn chunk_at_position(&self, pos: UVec3) -> Option<&Chunk> {
        self.chunks.iter().find(|&chunk| {
            pos.x >= chunk.min().x
                && pos.x <= chunk.max().x
                && pos.y >= chunk.min().y
                && pos.y <= chunk.max().y
                && pos.z >= chunk.min().z
                && pos.z <= chunk.max().z
        })
    }

    /// Recursively reroutes a path using astar pathing to further away chunks until a path can be found.
    ///
    /// Useful if local collision avoidance is failing.
    ///
    /// If you're using the plugin pathing systems, you shouldn't need to call this directly.
    ///
    /// # Arguments
    /// * `path` - The [`Path`] to reroute.
    /// * `start` - The start position of the path.
    /// * `goal` - The goal position of the path.
    /// * `blocking` - A map of positions to entities that are blocking the path. Pass `&HashMap::new()` if you're not concerned with collision.
    ///   Pass `&HasMap::new()` if you're not concerned with collision. If using [`crate::plugin::NorthstarPlugin`] you can pass it the [`crate::plugin::BlockingMap`] resource.
    ///   If not, build a [`HashMap<UVec3, Entity>`] with the positions of entities that should be blocking paths.
    /// * `refined` - Whether to use refined pathing or not.
    ///
    /// # Returns
    /// A new rerouted [`Path`] if successful, or `None` if no viable path could be found.
    ///
    pub fn reroute_path(
        &self,
        path: &Path,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        refined: bool,
    ) -> Option<Path> {
        reroute_path(self, path, start, goal, blocking, refined)
    }

    /// Checks if a path exists from `start` to `goal` using the fastest algorithm.
    /// Ignores any blocking entities.
    ///
    /// # Arguments
    /// * `start` - The starting position in the grid.
    /// * `goal` - The goal position in the grid.
    /// # Returns
    /// `true` if a path exists, `false` otherwise.
    ///
    pub fn is_path_viable(&self, start: UVec3, goal: UVec3) -> bool {
        pathfind(self, start, goal, &HashMap::new(), false, false).is_some()
    }

    /// Generate an HPA* path from `start` to `goal`.
    ///
    /// # Arguments
    /// * `start` - The starting position in the grid.
    /// * `goal` - The goal position in the grid.
    /// * `blocking` - A map of positions to entities that are blocking the path. Pass `&HashMap::new()` if you're not concerned with collision.
    ///   Pass `&HasMap::new()` if you're not concerned with collision. If using [`crate::plugin::NorthstarPlugin`] you can pass it the [`crate::plugin::BlockingMap`] resource.
    ///   If not, build a [`HashMap<UVec3, Entity>`] with the positions of entities that should be blocking paths.
    /// * `partial` - Whether to allow partial paths (i.e., if the goal is unreachable, return the closest reachable point).
    /// # Returns
    /// A [`Path`] if successful, or `None` if no viable path could be found.
    ///
    pub fn pathfind(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        partial: bool,
    ) -> Option<Path> {
        pathfind(self, start, goal, blocking, partial, true)
    }

    /// Generate a coarse (unrefined) HPA* path from `start` to `goal`.
    ///
    /// This method is useful for generating paths when the shortest viable path is not required.
    ///
    /// # Arguments
    /// * `start` - The starting position in the grid.
    /// * `goal` - The goal position in the grid.
    /// * `blocking` - A map of positions to entities that are blocking the path. Pass `&HashMap::new()` if you're not concerned with collision.
    ///   Pass `&HasMap::new()` if you're not concerned with collision. If using [`crate::plugin::NorthstarPlugin`] you can pass it the [`crate::plugin::BlockingMap`] resource.
    ///   If not, build a [`HashMap<UVec3, Entity>`] with the positions of entities that should be blocking paths.
    /// * `partial` - Whether to allow partial paths (i.e., if the goal is unreachable, return the closest reachable point).
    /// # Returns
    /// A [`Path`] if successful, or `None` if no viable path could be found.
    ///
    pub fn pathfind_coarse(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        partial: bool,
    ) -> Option<Path> {
        pathfind(self, start, goal, blocking, partial, false)
    }

    /// Generate a traditional A* path from `start` to `goal`.
    /// This method is useful for generating paths that require precise navigation and CPU cost isn't a concern.
    /// Great for a turn based game where movment cost is important.
    ///
    /// # Arguments
    /// * `start` - The starting position in the grid.
    /// * `goal` - The goal position in the grid.
    /// * `blocking` - A map of positions to entities that are blocking the path. Pass `&HashMap::new()` if you're not concerned with collision.
    ///   Pass `&HasMap::new()` if you're not concerned with collision. If using [`crate::plugin::NorthstarPlugin`] you can pass it the [`crate::plugin::BlockingMap`] resource.
    ///   If not, build a [`HashMap<UVec3, Entity>`] with the positions of entities that should be blocking paths.
    /// * `partial` - Whether to allow partial paths (i.e., if the goal is unreachable, return the closest reachable point).
    /// # Returns
    /// A [`Path`] if successful, or `None` if no viable path could be found.
    ///
    pub fn pathfind_astar(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        partial: bool,
    ) -> Option<Path> {
        pathfind_astar(
            &self.neighborhood,
            &self.grid.view(),
            start,
            goal,
            blocking,
            partial,
        )
    }

    /// Generate an A* path within a cubic radius around the `start` position.
    /// This can be used to limit an A* search to a confined search area.
    /// You'll want to ensure your radius at least covers the distance to the goal.
    ///
    /// # Arguments
    /// * `start` - The starting position in the grid.
    /// * `goal` - The goal position in the grid.
    /// * `radius` - The radius around the start position to search for a path.
    /// * `blocking` - A map of positions to entities that are blocking the path.
    ///   Pass `&HasMap::new()` if you're not concerned with collision. If using [`crate::plugin::NorthstarPlugin`] you can pass it the [`crate::plugin::BlockingMap`] resource.
    ///   If not, build a [`HashMap<UVec3, Entity>`] with the positions of entities that should be blocking paths.
    /// * `partial` - Whether to allow partial paths (i.e., if the goal is unreachable, return the closest reachable point).
    /// # Returns
    /// A [`Path`] if successful, or `None` if no viable path could be found.
    ///
    pub fn pathfind_astar_radius(
        &self,
        start: UVec3,
        goal: UVec3,
        radius: u32,
        blocking: &HashMap<UVec3, Entity>,
        partial: bool,
    ) -> Option<Path> {
        let min = start.as_ivec3().saturating_sub(IVec3::splat(radius as i32));
        let max = start
            .as_ivec3()
            .saturating_add(IVec3::splat(radius as i32) + IVec3::ONE);

        let grid_shape = self.grid.shape();
        let min = min.max(IVec3::ZERO);
        let max = max.min(IVec3::new(
            grid_shape[0] as i32,
            grid_shape[1] as i32,
            grid_shape[2] as i32,
        ));

        let grid_shape_vec = IVec3::new(
            grid_shape[0] as i32,
            grid_shape[1] as i32,
            grid_shape[2] as i32,
        );
        if !position_in_cubic_window(goal, start.as_ivec3(), radius as i32, grid_shape_vec) {
            return None;
        }

        // Create a subview of the grid
        let view = self.grid.slice(s![
            min.x as usize..max.x as usize,
            min.y as usize..max.y as usize,
            min.z as usize..max.z as usize
        ]);

        // Remap start/goal into local view
        let start_local = (start.as_ivec3() - min).as_uvec3();
        let goal_local = (goal.as_ivec3() - min).as_uvec3();

        // Remap blocking positions into local view
        let blocking_local: HashMap<UVec3, Entity> = blocking
            .iter()
            .filter_map(|(pos, &ent)| {
                let pos_i = pos.as_ivec3();
                if pos_i.cmplt(min).any() || pos_i.cmpge(max).any() {
                    return None;
                }
                Some(((pos_i - min).as_uvec3(), ent))
            })
            .collect();

        // Run pathfinding on the subview
        let result = pathfind_astar(
            &self.neighborhood,
            &view,
            start_local,
            goal_local,
            &blocking_local,
            partial,
        );

        // Convert path result back to global positions
        result.map(|mut path| {
            path.translate_by(min.as_uvec3());
            path
        })
    }
}

#[cfg(test)]
mod tests {
    use bevy::{math::UVec3, platform::collections::HashMap};

    use crate::{
        dir::Dir,
        grid::{
            ChunkSettings, CollisionSettings, Grid, GridInternalSettings, GridSettings,
            GridSettingsBuilder, NavCell, NavSettings, NeighborhoodSettings,
        },
        nav::Nav,
        neighbor::OrdinalNeighborhood3d,
    };

    const GRID_SETTINGS: GridSettings = GridSettings(GridInternalSettings {
        dimensions: UVec3::new(12, 12, 1),
        chunk_settings: ChunkSettings {
            size: 4,
            depth: 1,
            diagonal_connections: false,
        },
        cost_settings: NavSettings {
            default_movement_cost: 1,
            default_impassible: false,
        },
        collision_settings: CollisionSettings {
            enabled: false,
            avoidance_distance: 4,
        },
        neighborhood_settings: NeighborhoodSettings {
            filters: Vec::new(),
        },
    });

    #[test]
    pub fn test_new() {
        let grid = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS);
        assert_eq!(grid.grid.shape(), [12, 12, 1]);
    }

    #[test]
    pub fn test_edges() {
        let grid_settings = GridSettingsBuilder::new_2d(4, 4)
            .chunk_size(4)
            .enable_diagonal_connections()
            .build();

        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);

        // Fill grid edges with walls
        for x in 0..4 {
            for y in 0..4 {
                if x == 0 || x == 3 || y == 0 || y == 3 {
                    grid.grid[[x, y, 0]] = NavCell::new(Nav::Impassable);
                }
            }
        }

        let chunk = grid.chunks[[0, 0, 0]].clone();

        let mut edges = Vec::new();

        edges.push(chunk.edge(&grid.grid, Dir::NORTH));
        edges.push(chunk.edge(&grid.grid, Dir::EAST));
        edges.push(chunk.edge(&grid.grid, Dir::SOUTH));
        edges.push(chunk.edge(&grid.grid, Dir::WEST));

        for edge in edges {
            for cell in edge.iter() {
                assert!(cell.is_impassable());
            }
        }
    }

    #[test]
    pub fn test_calculate_edge_nodes() {
        let grid = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS);

        let chunk = grid.chunks.iter().next().unwrap().clone();
        let neighbor_chunk = grid.chunks.iter().nth(1).unwrap().clone();

        let edges = grid.calculate_edge_nodes(
            chunk.edge(&grid.grid, Dir::NORTH),
            neighbor_chunk.edge(&grid.grid, Dir::SOUTH),
            chunk.clone(),
            Dir::NORTH,
        );

        assert_eq!(edges.len(), 1);
    }

    #[test]
    pub fn test_calculate_edge_nodes_3d() {
        let grid_settings = GridSettingsBuilder::new_3d(8, 8, 8)
            .chunk_size(4)
            .chunk_depth(4)
            .enable_diagonal_connections()
            .build();

        let grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);

        let chunk = grid.chunks.iter().next().unwrap().clone();
        let neighbor_chunk = grid.chunks.iter().nth(1).unwrap().clone();

        let edges = grid.calculate_edge_nodes(
            chunk.edge(&grid.grid, Dir::NORTH),
            neighbor_chunk.edge(&grid.grid, Dir::SOUTH),
            chunk.clone(),
            Dir::NORTH,
        );

        assert_eq!(edges.len(), 4);
    }

    #[test]
    pub fn test_build_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build_nodes();

        let nodes = grid.graph.nodes();

        assert_eq!(nodes[0].pos, UVec3::new(2, 3, 0));
        assert_eq!(nodes[1].pos, UVec3::new(3, 2, 0));

        assert_eq!(nodes[2].pos, UVec3::new(2, 7, 0));
        assert_eq!(nodes[3].pos, UVec3::new(3, 6, 0));
        assert_eq!(nodes[4].pos, UVec3::new(2, 4, 0));

        assert_eq!(nodes[5].pos, UVec3::new(3, 10, 0));
        assert_eq!(nodes[6].pos, UVec3::new(2, 8, 0));

        assert_eq!(grid.graph.node_count(), 24);

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        let chunk_size = GRID_SETTINGS.0.chunk_settings.size as usize;
        let half_chunk_size = chunk_size / 2;

        for x in 0..(GRID_SETTINGS.0.dimensions.x as usize) {
            for y in 0..(GRID_SETTINGS.0.dimensions.y as usize) {
                if x % chunk_size == 0 && y % half_chunk_size == 0 {
                    grid.grid[[x, y, 0]] = NavCell::new(Nav::Impassable);
                } else {
                    grid.grid[[x, y, 0]] = NavCell::new(Nav::Passable(1));
                }
            }
        }

        grid.build_nodes();

        assert_eq!(grid.graph.node_count(), 36);

        let grid_settings = GridSettingsBuilder::new_2d(12, 12)
            .chunk_size(4)
            .enable_diagonal_connections()
            .build();

        // Test ordinal
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        grid.build_nodes();

        assert_eq!(grid.graph.node_count(), 40);
    }

    #[test]
    pub fn test_connect_internal_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.precompute_neighbors();
        grid.build_nodes();
        grid.connect_internal_chunk_nodes();

        assert_eq!(grid.graph.node_count(), 24);
        assert_eq!(grid.graph.edge_count(), 44);
    }

    #[test]
    pub fn test_connect_adjacent_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.precompute_neighbors();
        grid.build_nodes();
        grid.connect_adjacent_chunk_nodes();

        let edges = grid
            .graph
            .node_at(UVec3::new(2, 3, 0))
            .unwrap()
            .edges
            .clone();

        assert!(edges.contains_key(&UVec3::new(2, 4, 0)));

        assert_eq!(edges[&UVec3::new(2, 4, 0)].path().len(), 2);
        assert_eq!(edges[&UVec3::new(2, 4, 0)].cost(), 1);

        assert_eq!(grid.graph.edge_count(), 24);
    }

    #[test]
    pub fn test_get_chunk_for_position() {
        let grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);
        let chunk = grid.chunk_at_position(UVec3::new(0, 0, 0)).unwrap();

        assert_eq!(chunk.min(), UVec3::new(0, 0, 0));
        assert_eq!(
            chunk.max(),
            UVec3::new(
                GRID_SETTINGS.0.chunk_settings.size - 1,
                GRID_SETTINGS.0.chunk_settings.size - 1,
                GRID_SETTINGS.0.chunk_settings.depth - 1
            )
        );
    }

    #[test]
    pub fn test_get_all_nodes_in_chunk() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);
        grid.build_nodes();

        let nodes = grid.graph.nodes_in_chunk(grid.chunks[[0, 0, 0]].clone());

        assert_eq!(nodes.len(), 2);
    }

    #[test]
    pub fn test_get_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.pathfind(
            UVec3::new(10, 10, 0),
            UVec3::new(4, 4, 0),
            &HashMap::new(),
            false,
        );
        let raw_path = grid.pathfind_astar(
            UVec3::new(10, 10, 0),
            UVec3::new(4, 4, 0),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
        // Ensure start cell is the first cell in the path
        assert_ne!(path.clone().unwrap().path()[0], UVec3::new(10, 10, 0));
        // Ensure end cell is the last cell in the path
        assert_eq!(
            path.clone().unwrap().path().last().unwrap(),
            &UVec3::new(4, 4, 0)
        );

        assert_eq!(path.clone().unwrap().len(), raw_path.unwrap().len());
        assert_eq!(path.unwrap().len(), 6);
    }

    #[test]
    fn test_calculate_edge_nodes_returns_center() {
        let grid_settings = GridSettingsBuilder::new_2d(64, 64).chunk_size(32).build();

        let grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        let start_edge = grid.chunks[[0, 0, 0]].edge(&grid.grid, Dir::NORTH);
        let end_edge = grid.chunks[[0, 1, 0]].edge(&grid.grid, Dir::SOUTH);

        let nodes = grid.calculate_edge_nodes(
            start_edge,
            end_edge,
            grid.chunks[[0, 0, 0]].clone(),
            Dir::NORTH,
        );

        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].pos, UVec3::new(16, 0, 0));
    }

    #[test]
    fn test_random_grid_path() {
        let width = 128;
        let height = 128;
        let chunk_size = 32;

        // Test a grid with randomized walls, the grid must be solvable
        let grid_settings = GridSettingsBuilder::new_2d(width, height)
            .chunk_size(chunk_size)
            .enable_diagonal_connections()
            .build();

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        for x in 0..width {
            for y in 0..height {
                if (x % 4 == 0)
                    && (y % chunk_size == 0 || y / (chunk_size + chunk_size) == chunk_size)
                {
                    grid.grid[[x as usize, y as usize, 0]] = NavCell::new(Nav::Impassable);
                }

                if (y % 4 == 0)
                    && (x % chunk_size == 0 || x & (chunk_size + chunk_size) == chunk_size)
                {
                    grid.grid[[x as usize, y as usize, 0]] = NavCell::new(Nav::Impassable);
                }
            }
        }

        grid.build();

        let path = grid.pathfind(
            UVec3::new(7, 7, 0),
            UVec3::new(121, 121, 0),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
        assert!(!path.unwrap().is_empty());
    }

    #[test]
    fn test_large_3d_path() {
        let grid_settings = GridSettingsBuilder::new_3d(128, 128, 4)
            .chunk_size(16)
            .chunk_depth(2)
            .build();

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        grid.build();
        let path = grid.pathfind(
            UVec3::new(0, 0, 0),
            UVec3::new(31, 31, 3),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
    }

    #[test]
    pub fn test_get_astar_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.pathfind_astar(
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 10);
    }

    #[test]
    pub fn test_is_path_viable() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        // Block off a section of the grid to make sure the path is not viable
        for x in 0..12 {
            grid.set_nav(
                UVec3::new(x, 5, 0),
                Nav::Impassable, // Set as wall
            );
        }

        grid.build();

        let viable = grid.is_path_viable(UVec3::new(0, 0, 0), UVec3::new(10, 0, 0));
        assert!(viable);

        let not_viable = grid.is_path_viable(UVec3::new(0, 0, 0), UVec3::new(8, 8, 0));
        assert!(!not_viable);

        let out_of_bounds_not_viable =
            grid.is_path_viable(UVec3::new(100, 100, 0), UVec3::new(200, 200, 0));
        assert!(!out_of_bounds_not_viable);
    }
}
