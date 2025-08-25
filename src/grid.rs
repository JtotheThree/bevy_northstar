//! This module contains the `Grid` component which is the main component for the crate.
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use std::sync::Arc;

use bevy::{
    ecs::entity::Entity, log, math::{IVec3, UVec3}, platform::collections::{HashMap, HashSet}, prelude::Component, transform::components::Transform
};
use ndarray::{s, Array2, Array3, ArrayView1, ArrayView2, ArrayView3, Zip};

use crate::{
    chunk::Chunk,
    dijkstra::*,
    dir::*,
    filter::NeighborFilter,
    flood_fill::flood_fill_bool_mask,
    graph::Graph,
    nav::{Nav, NavCell, Portal},
    nav_mask::NavMaskData,
    neighbor::Neighborhood,
    node::Node,
    path::Path,
    pathfind::{pathfind, pathfind_astar, pathfind_new, pathfind_thetastar, reroute_path, PathfindRequest},
    position_in_cubic_window,
    prelude::{NavMask, PathfindMode},
    timed, MovementCost,
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
    /// Provide a `Vec` of [`NeighborFilter`]s to apply custom filtering logic.
    /// The filters will be chained in order added.
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
/// ```rust,no_run
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
// Transform doesn't actually do anything for the grid
// Requiring it just surpresses the Bevy warning if the user doesn't add the grid 
// as a child to anything.
#[require(Transform)]
pub struct Grid<N: Neighborhood> {
    pub(crate) neighborhood: N,

    dimensions: UVec3,
    chunk_settings: ChunkSettings,
    collision_settings: CollisionSettings,

    grid: Array3<NavCell>,
    chunks: Array3<Chunk>,

    graph: Graph,

    dirty: bool,
    built: bool,
    dirty_chunks: HashSet<(usize, usize, usize)>,
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
                let max_x = min_x + chunk_settings.size;
                let min_y = y as u32 * chunk_settings.size;
                let max_y = min_y + chunk_settings.size;
                let min_z = z as u32 * chunk_settings.depth;
                let max_z = min_z + chunk_settings.depth;

                Chunk::new(
                    (x, y, z),
                    UVec3::new(min_x, min_y, min_z),
                    UVec3::new(max_x, max_y, max_z),
                )
            },
        );

        // Put all the chunks into dirty_chunks so they can be built later.
        let dirty_chunks = (0..x_chunks as usize)
            .flat_map(|x| {
                (0..y_chunks as usize)
                    .flat_map(move |y| (0..z_chunks as usize).map(move |z| (x, y, z)))
            })
            .collect::<HashSet<_>>();

        Self {
            neighborhood: N::from_settings(&settings.0.neighborhood_settings),
            dimensions,
            chunk_settings,
            collision_settings,

            grid,
            chunks,

            graph: Graph::new(),

            dirty: true,
            built: false,
            dirty_chunks,
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

    /// Test if a grid cell is a portal to a target [`bevy::math::UVec3`] cell.
    pub fn is_portal(&self, pos: UVec3) -> bool {
        if !self.in_bounds(pos) {
            return false;
        }

        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].is_portal()
    }

    /// Set the [`Nav`] settings at a given [`bevy::math::UVec3`] position in the grid.
    pub fn set_nav(&mut self, pos: UVec3, nav: Nav) {
        if !self.in_bounds(pos) {
            panic!("Attempted to set nav at out-of-bounds position at {pos}");
        }

        // If the grid not dirty, we need to flag every chunk, edge, and nodes that needs to be rebuilt.
        if self.built {
            self.dirty = true;
            self.mark_dirty_for_pos(pos);
        }

        // Handle portals
        if let Nav::Portal(portal) = nav {
            let target = portal.target;

            // Check if the target is in bounds as well
            if !self.in_bounds(target) {
                panic!("Portal {target} is out of bounds");
            }

            if !portal.one_way {
                // Mark the target chunk as dirty
                self.mark_dirty_for_pos(target);

                // Create a reverse portal at the target position.
                let reverse_portal = Portal::to(pos, portal.cost, true);
                self.set_nav(target, Nav::Portal(reverse_portal));
            }
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

    /// Returns the depth of the chunks in the grid.
    pub fn chunk_depth(&self) -> u32 {
        self.chunk_settings.depth
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

    /// Returns the neighbors of a given position in the grid.
    pub fn neighbors(&self, pos: &UVec3) -> Vec<Dir> {
        if !self.in_bounds(*pos) {
            return Vec::new();
        }

        let neighbor_bits =
            self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].neighbor_bits;
        Dir::from_bits(neighbor_bits)
    }

    pub(crate) fn chunk_in_bounds(&self, chunk_x: isize, chunk_y: isize, chunk_z: isize) -> bool {
        let chunk_size = self.chunk_settings.size as usize;
        let chunk_depth = self.chunk_settings.depth as usize;
        let x_chunks = self.dimensions.x as usize / chunk_size;
        let y_chunks = self.dimensions.y as usize / chunk_size;
        let z_chunks = self.dimensions.z as usize / chunk_depth;

        chunk_x >= 0
            && chunk_x < x_chunks as isize
            && chunk_y >= 0
            && chunk_y < y_chunks as isize
            && chunk_z >= 0
            && chunk_z < z_chunks as isize
    }

    pub(crate) fn needs_build(&self) -> bool {
        if self.dirty {
            log::error!("Grid is dirty! You must call `build()` after modifying the grid.");
            return true;
        }

        if !self.built {
            log::error!("Grid is not built! You must call `build()` after setting up the grid.");
            return true;
        }

        false
    }

    /// Marks the chunk containing the given position as dirty, marks all its edges as dirty,
    /// and marks the relevant edges and cells of adjacent chunks as dirty as well.
    fn mark_dirty_for_pos(&mut self, pos: UVec3) {
        let chunk_size = self.chunk_settings.size as usize;
        let chunk_depth = self.chunk_settings.depth as usize;

        let chunk_x = pos.x as usize / chunk_size;
        let chunk_y = pos.y as usize / chunk_size;
        let chunk_z = pos.z as usize / chunk_depth;

        // Get a reference to the chunk
        let Some(chunk) = self.chunks.get((chunk_x, chunk_y, chunk_z)) else {
            return;
        };

        // Check which edges (if any) this position touches
        let touched_dirs: Vec<Dir> = chunk.touching_edges(pos).collect();

        // If diagonal or ordinal connectivity is enabled, we always dirty all directions
        let mark_all_dirs =
            self.chunk_settings.diagonal_connections || self.neighborhood.is_ordinal();

        let dirs_to_dirty: Vec<Dir> = if touched_dirs.is_empty() && !mark_all_dirs {
            // Only dirty the local chunk (no neighbor edge concerns)
            self.dirty_chunks.insert((chunk_x, chunk_y, chunk_z));
            return;
        } else if mark_all_dirs {
            Dir::all().collect()
        } else {
            touched_dirs
        };

        // Dirty this chunk and mark affected edges
        self.dirty_chunks.insert((chunk_x, chunk_y, chunk_z));
        if let Some(mut_chunk) = self.chunks.get_mut((chunk_x, chunk_y, chunk_z)) {
            for dir in dirs_to_dirty.iter() {
                mut_chunk.set_dirty_edge(*dir, true);
            }
        }

        // Dirty adjacent chunks and mark their opposite edges
        for dir in dirs_to_dirty {
            let offset = dir.offset();
            let (dx, dy, dz) = (offset.x, offset.y, offset.z);
            let nx = chunk_x as isize + dx as isize;
            let ny = chunk_y as isize + dy as isize;
            let nz = chunk_z as isize + dz as isize;

            if self.chunk_in_bounds(nx, ny, nz) {
                let n_coords = (nx as usize, ny as usize, nz as usize);
                self.dirty_chunks.insert(n_coords);

                if let Some(neighbor_chunk) = self.chunks.get_mut(n_coords) {
                    neighbor_chunk.set_dirty_edge(dir.opposite(), true);
                }
            }
        }
    }

    /// Builds the entire grid. This includes precomputing neighbors, creating nodes for each edge of each chunk,
    /// caching paths between internal nodes within each chunk, and connecting adjacent nodes between chunks.
    /// This method needs to be called after the grid has been initialized.
    pub fn build(&mut self) {
        #[cfg(feature = "stats")]
        let num_dirty_chunks = self.dirty_chunks.len();
        #[cfg(feature = "stats")]
        let build_start = std::time::Instant::now();

        timed!("Precomputed neighbors", { self.precompute_neighbors() });
        timed!("Built nodes", { self.build_nodes() });
        timed!("Create portal nodes", {
            self.create_portal_nodes();
        });
        timed!("Connected internal chunk nodes", {
            self.connect_internal_chunk_nodes()
        });
        timed!("Connected adjacent chunk nodes", {
            self.connect_adjacent_chunk_nodes()
        });

        for (_, chunk) in self.chunks.indexed_iter_mut() {
            chunk.clean();
        }

        #[cfg(feature = "stats")]
        {
            if num_dirty_chunks == 0 {
                log::debug!("No dirty chunks to build.");
                return;
            }
            let elapsed = build_start.elapsed();
            log::debug!(
                "Built grid with {} dirty chunks in {:?}: Average {}µs per chunk",
                num_dirty_chunks,
                elapsed,
                elapsed.as_micros() / num_dirty_chunks as u128
            );
        }

        self.dirty = false;
        self.dirty_chunks.clear();
        self.built = true;
    }

    fn precompute_neighbors(&mut self) {
        #[cfg(feature = "parallel")]
        {
            self.precompute_neighbors_parallel();
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.precompute_neighbors_single();
        }
    }

    /// Precomputes the neighbors for each cell in the grid.
    #[cfg(not(feature = "parallel"))]
    fn precompute_neighbors_single(&mut self) {
        let mut updates = Vec::new();
        let grid_view = self.grid.view();
        let neighborhood = &self.neighborhood;

        for (_, chunk) in self.chunks.indexed_iter_mut() {
            if !self.dirty_chunks.contains(&chunk.index()) {
                continue;
            }

            for pos in chunk.bounds() {
                let (pos, bits, special) = compute_cell_neighbors(neighborhood, &grid_view, pos);
                updates.push((pos, bits, special));
            }
        }

        // Apply updates after view is dropped
        for (pos, bits, special) in updates {
            self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].neighbor_bits = bits;
            self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].special_neighbors = special;
        }
    }

    /// Precomputes the neighbors for each cell in the grid in parallel.
    #[cfg(feature = "parallel")]
    fn precompute_neighbors_parallel(&mut self) {
        let grid_view = self.grid.view();
        let neighborhood = &self.neighborhood;

        let updates: Vec<(UVec3, u32, Vec<UVec3>)> = self
            .chunks
            .indexed_iter()
            .par_bridge() // rayon parallel iterator over non-par types
            .filter_map(|(_, chunk)| {
                if !self.dirty_chunks.contains(&chunk.index()) {
                    return None;
                }

                let updates = chunk
                    .bounds()
                    .map(|pos| {
                        let (pos, bits, special) =
                            compute_cell_neighbors(neighborhood, &grid_view, pos);
                        (pos, bits, special)
                    })
                    .collect::<Vec<_>>();

                // Handle special neighbors?

                Some(updates)
            })
            .flatten()
            .collect();

        // Now apply updates
        for (pos, bits, special) in updates {
            self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].neighbor_bits = bits;
            self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].special_neighbors = special;
        }
    }

    fn build_nodes(&mut self) {
        #[cfg(feature = "parallel")]
        {
            self.build_nodes_parallel();
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.build_nodes_single();
        }
    }

    /// Shared logic for building nodes for a single chunk at (x, y, z).
    /// Returns (nodes_to_add, cleaned_edges).
    fn build_nodes_for_chunk(&self, x: usize, y: usize, z: usize) -> Option<(Vec<Node>, Vec<Dir>)> {
        let chunk_size = self.chunk_settings.size as usize;
        let chunk_depth = self.chunk_settings.depth as usize;
        let x_chunks = self.dimensions.x as usize / chunk_size;
        let y_chunks = self.dimensions.y as usize / chunk_size;
        let z_chunks = self.dimensions.z as usize / chunk_depth;

        let chunk = &self.chunks[[x, y, z]];

        if !chunk.has_dirty_edges() {
            return None;
        }

        let mut cleaned_edges = Vec::new();
        let mut nodes_to_add = Vec::new();

        // Helper closure to check neighbor in bounds
        let in_bounds = |nx: i32, ny: i32, nz: i32| {
            nx >= 0
                && nx < x_chunks as i32
                && ny >= 0
                && ny < y_chunks as i32
                && nz >= 0
                && nz < z_chunks as i32
        };

        // Cardinal Chunk Faces
        for dir in Dir::cardinal_faces() {
            if !chunk.is_edge_dirty(dir) {
                continue;
            }

            let dir_vec = dir.offset();
            let nx = x as i32 + dir_vec.x;
            let ny = y as i32 + dir_vec.y;
            let nz = z as i32 + dir_vec.z;

            if !in_bounds(nx, ny, nz) {
                continue;
            }

            let neighbor_chunk = &self.chunks[[nx as usize, ny as usize, nz as usize]];

            let current_edge = chunk.face(&self.grid, dir);
            let neighbor_edge = neighbor_chunk.face(&self.grid, dir.opposite());

            let mut new_nodes =
                self.calculate_face_nodes(current_edge, neighbor_edge, chunk.clone(), dir);

            // Adjust node positions according to direction
            for node in new_nodes.iter_mut() {
                node.pos = chunk.boundary_pos_to_global(node.pos, dir);
            }

            nodes_to_add.extend(new_nodes);
            cleaned_edges.push(dir);
        }

        // Cardinal Chunk Edges
        if self.depth() > 1 {
            for dir in Dir::cardinal_edges() {
                if !chunk.is_edge_dirty(dir) {
                    continue;
                }

                let dir_vec = dir.offset();
                let nx = x as i32 + dir_vec.x;
                let ny = y as i32 + dir_vec.y;
                let nz = z as i32 + dir_vec.z;

                if !in_bounds(nx, ny, nz) {
                    continue;
                }

                let neighbor_chunk = &self.chunks[[nx as usize, ny as usize, nz as usize]];

                let current_edge = chunk.edge(&self.grid, dir);
                let neighbor_edge = neighbor_chunk.edge(&self.grid, dir.opposite());

                let mut new_nodes =
                    self.calculate_edge_nodes(current_edge, neighbor_edge, chunk.clone(), dir);

                // Adjust node positions according to direction
                for node in new_nodes.iter_mut() {
                    node.pos = chunk.boundary_pos_to_global(node.pos, dir);
                }

                nodes_to_add.extend(new_nodes);
                cleaned_edges.push(dir);
            }
        }

        // Diagonal directions if enabled
        if self.chunk_settings.diagonal_connections {
            // Iterate over chunk dir corners
            for dir in Dir::ordinal() {
                if !chunk.is_edge_dirty(dir) {
                    continue;
                }

                let dir_vec = dir.offset();
                let nx = x as i32 + dir_vec.x;
                let ny = y as i32 + dir_vec.y;
                let nz = z as i32 + dir_vec.z;

                if !in_bounds(nx, ny, nz) {
                    continue;
                }

                let neighbor_chunk = &self.chunks[[nx as usize, ny as usize, nz as usize]];

                let current_corner = chunk.corner(&self.grid, dir);
                let neighbor_corner = neighbor_chunk.corner(&self.grid, dir.opposite());

                if current_corner.is_impassable() || neighbor_corner.is_impassable() {
                    continue;
                }

                let pos = chunk.corner_pos(dir);

                nodes_to_add.push(Node::new(pos, chunk.clone(), Some(dir)));
                cleaned_edges.push(dir);
            }
        }

        Some((nodes_to_add, cleaned_edges))
    }

    /// Builds the nodes for each edge of each chunk.
    /// This method will calculate nodes for each edge of the chunk and connect them to the graph in parallel.
    #[cfg(feature = "parallel")]
    fn build_nodes_parallel(&mut self) {
        use rayon::prelude::*;

        let dirty_chunks: Vec<(usize, usize, usize)> = self.dirty_chunks.iter().copied().collect();

        // Process chunks in parallel and produce per-chunk results
        let results: Vec<_> = dirty_chunks
            .into_par_iter()
            .filter_map(|(x, y, z)| {
                self.build_nodes_for_chunk(x, y, z)
                    .map(|(nodes, cleaned_edges)| (x, y, z, nodes, cleaned_edges))
            })
            .collect();

        // Apply results sequentially (mutable access to self)
        for (x, y, z, nodes, cleaned_edges) in results {
            let chunk = &mut self.chunks[[x, y, z]];
            self.graph.remove_nodes_for_edges(chunk, &cleaned_edges);
            self.graph.add_nodes(&nodes);

            for &dir in &cleaned_edges {
                chunk.set_dirty_edge(dir, false);
            }
        }
    }

    /// Builds the nodes for each edge of each chunk (single-threaded).
    #[cfg(not(feature = "parallel"))]
    fn build_nodes_single(&mut self) {
        for (x, y, z) in self.dirty_chunks.iter().copied() {
            let chunk = &self.chunks[[x, y, z]];

            if !chunk.has_dirty_edges() {
                continue;
            }

            if let Some((nodes_to_add, cleaned_edges)) = self.build_nodes_for_chunk(x, y, z) {
                let chunk = &mut self.chunks[[x, y, z]];
                self.graph.remove_nodes_for_edges(chunk, &cleaned_edges);
                self.graph.add_nodes(&nodes_to_add);

                for &dir in &cleaned_edges {
                    chunk.set_dirty_edge(dir, false);
                }
            }
        }
    }

    fn calculate_face_nodes(
        &self,
        start_face: ArrayView2<NavCell>,
        end_face: ArrayView2<NavCell>,
        chunk: Chunk,
        dir: Dir,
    ) -> Vec<Node> {
        // Create the boolean masks for passable regions
        let mut start_mask = Array2::from_elem(start_face.raw_dim(), false);
        let mut end_mask = Array2::from_elem(end_face.raw_dim(), false);

        for ((x, y), cell) in start_face.indexed_iter() {
            if !cell.is_impassable() {
                start_mask[(x, y)] = true;
            }
        }
        for ((x, y), cell) in end_face.indexed_iter() {
            if !cell.is_impassable() {
                end_mask[(x, y)] = true;
            }
        }

        // AND the masks and get the intersection
        let mut intersection = Array2::from_elem(start_face.raw_dim(), false);
        Zip::from(&mut intersection)
            .and(&start_mask)
            .and(&end_mask)
            .for_each(|out, &a, &b| *out = a && b);

        let groups = flood_fill_bool_mask(intersection.view());

        let mut nodes = Vec::new();
        for group in groups {
            let (x, y) = group
                .iter()
                .min_by_key(|&&(gx, gy)| {
                    let cx = (start_face.shape()[0] / 2) as isize;
                    let cy = (start_face.shape()[1] / 2) as isize;
                    (gx as isize - cx).abs() + (gy as isize - cy).abs()
                })
                .copied()
                .unwrap();
            let pos = UVec3::new(x as u32, y as u32, 0);
            nodes.push(Node::new(pos, chunk.clone(), Some(dir)));
        }
        // If no nodes found and ordinal movement is allowed, try checking adjacent edge cells
        if nodes.is_empty() && self.neighborhood.is_ordinal() {
            let mut ordinal_nodes = Vec::new();
            for ((x, y), start_cell) in start_face.indexed_iter() {
                if start_cell.is_impassable() {
                    continue;
                }

                if x > 0 {
                    let left = end_face[[x - 1, y]].clone();
                    if !left.is_impassable() {
                        let pos = UVec3::new(x as u32, y as u32, 0);
                        ordinal_nodes.push(Node::new(pos, chunk.clone(), Some(dir)));
                    }
                }

                if x < end_face.shape()[0] - 1 {
                    let right = end_face[[x + 1, y]].clone();
                    if !right.is_impassable() {
                        let pos = UVec3::new(x as u32, y as u32, 0);
                        ordinal_nodes.push(Node::new(pos, chunk.clone(), Some(dir)));
                    }
                }
            }
            return ordinal_nodes;
        }

        nodes
    }

    /// Calculates the 1D edge nodes for a given direction.
    fn calculate_edge_nodes(
        &self,
        start_edge: ArrayView1<NavCell>,
        end_edge: ArrayView1<NavCell>,
        chunk: Chunk,
        dir: Dir,
    ) -> Vec<Node> {
        let mut nodes = Vec::new();

        for (i, start_cell) in start_edge.iter().enumerate() {
            if start_cell.is_impassable() {
                continue;
            }

            let end_cell = end_edge[i].clone();
            if !end_cell.is_impassable() {
                let pos = UVec3::new(i as u32, 0, 0); // Y/Z will be filled in later based on direction
                nodes.push(Node::new(pos, chunk.clone(), Some(dir)));
            }
        }

        if !nodes.is_empty() {
            let middle = nodes.len() / 2;
            return vec![nodes[middle].clone()];
        }

        // If no nodes found and ordinal movement is allowed, try checking adjacent edge cells
        if !self.neighborhood.is_ordinal() {
            return nodes;
        }

        let mut ordinal_nodes = Vec::new();
        for (i, start_cell) in start_edge.iter().enumerate() {
            if start_cell.is_impassable() {
                continue;
            }

            if i > 0 {
                let left = &end_edge[i - 1];
                if !left.is_impassable() {
                    ordinal_nodes.push(Node::new(
                        UVec3::new(i as u32, 0, 0),
                        chunk.clone(),
                        Some(dir),
                    ));
                    continue;
                }
            }

            if i + 1 < end_edge.len() {
                let right = &end_edge[i + 1];
                if !right.is_impassable() {
                    ordinal_nodes.push(Node::new(
                        UVec3::new(i as u32, 0, 0),
                        chunk.clone(),
                        Some(dir),
                    ));
                }
            }
        }

        ordinal_nodes
    }

    fn create_portal_nodes(&mut self) {
        for (x, y, z) in self.dirty_chunks.iter().copied() {
            let chunk = &self.chunks[[x, y, z]];

            // Iterate over all cells in the grid and check for portals.
            for (pos, cell) in chunk.view(&self.grid).indexed_iter() {
                if cell.is_portal() {
                    if let Nav::Portal(Portal { target, .. }) = cell.nav() {
                        // If the current cell and target are in the same chunk, skip.
                        let pos_uvec3 = UVec3::new(
                            pos.0 as u32 + chunk.min().x,
                            pos.1 as u32 + chunk.min().y,
                            pos.2 as u32 + chunk.min().z,
                        );

                        if let Some(target_chunk) = self.chunk_at_position(target) {
                            if chunk == target_chunk {
                                continue;
                            }

                            let node = Node {
                                pos: pos_uvec3,
                                chunk_index: chunk.index(),
                                edges: HashMap::new(),
                                dir: None,
                                portal: true,
                            };

                            // Create node at the target position and give it a reverse path
                            let target_node = Node {
                                pos: target,
                                chunk_index: target_chunk.index(),
                                edges: HashMap::new(),
                                dir: None,
                                portal: true,
                            };

                            // Create a Node for the portal and insert it into the graph.
                            self.graph.add_node(node);
                            self.graph.add_node(target_node);
                        }
                    }
                }
            }
        }
    }

    fn connect_internal_chunk_nodes(&mut self) {
        #[cfg(feature = "parallel")]
        {
            self.connect_internal_chunk_nodes_parallel();
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.connect_internal_chunk_nodes_single();
        }
    }

    // Connects the internal nodes of each chunk to each other.
    #[cfg(not(feature = "parallel"))]
    fn connect_internal_chunk_nodes_single(&mut self) {
        for (x, y, z) in self.dirty_chunks.iter().copied() {
            // Connect internal nodes

            let chunk_grid = self.chunks[[x, y, z]].view(&self.grid);
            let chunk = &self.chunks[[x, y, z]];

            // Clear all node edges with positions in the chunk if any exist
            self.graph.remove_edges_for_chunk(chunk);

            let nodes = self.graph.nodes_in_chunk(chunk);

            // Collect all connections in a Vec
            let mut all_connections = Vec::new();

            for node in nodes.iter() {
                let start = node.pos - chunk.min();

                let goals: Vec<_> = nodes
                    .iter()
                    .filter(|other| other.pos != node.pos)
                    .map(|other| other.pos - chunk.min())
                    .collect();

                let paths = dijkstra_grid(&chunk_grid, start, &goals, false, 100, &NavMaskData::new());

                for (goal_pos, path) in paths.into_iter() {
                    let world_start = node.pos;
                    let world_goal = goal_pos + chunk.min();
                    let path_vec = path
                        .path()
                        .iter()
                        .map(|p| *p + chunk.min())
                        .collect::<Vec<_>>();

                    all_connections.push((
                        world_start,
                        world_goal,
                        Path::new(path_vec.clone(), path_vec.len() as u32),
                    ));
                }
            }

            for (node_pos, other_node_pos, path) in all_connections {
                self.graph.connect_node(node_pos, other_node_pos, path);
            }
        }
    }

    // Connects the internal nodes of each chunk to each other in parallel.
    #[cfg(feature = "parallel")]
    fn connect_internal_chunk_nodes_parallel(&mut self) {
        for (x, y, z) in self.dirty_chunks.iter().copied() {
            // Connect internal nodes

            let chunk_grid = self.chunks[[x, y, z]].view(&self.grid);
            let chunk = &self.chunks[[x, y, z]];

            // Clear all node edges with positions in the chunk if any exist
            // This is done for rebuilding the grid
            self.graph.remove_edges_for_chunk(chunk);

            let nodes = self.graph.nodes_in_chunk(chunk);

            let all_connections: Vec<_> = nodes
                .par_iter()
                .flat_map_iter(|node| {
                    let start = node.pos - chunk.min();
                    let goals = nodes
                        .iter()
                        .filter(|other| other.pos != node.pos)
                        .map(|other| other.pos - chunk.min())
                        .collect::<Vec<_>>();

                    let paths =
                        dijkstra_grid(&chunk_grid, start, &goals, false, 100, &NavMaskData::new());

                    paths.into_iter().map(move |(goal_pos, path)| {
                        let world_start = node.pos;
                        let world_goal = goal_pos + chunk.min();
                        let path_vec = path
                            .path()
                            .iter()
                            .map(|p| *p + chunk.min())
                            .collect::<Vec<_>>();
                        (
                            world_start,
                            world_goal,
                            Path::new(path_vec.clone(), path_vec.len() as u32),
                        )
                    })
                })
                .collect();

            for (node_pos, other_node_pos, path) in all_connections {
                self.graph.connect_node(node_pos, other_node_pos, path);
            }
        }
    }

    // Connects the nodes of adjacent chunks to each other.
    fn connect_adjacent_chunk_nodes(&mut self) {
        let nodes = self.graph.nodes();

        let mut connections = Vec::new();

        for node in nodes {
            if node.portal {
                // Connect the portal node to its target directly
                // Collect the portal info
                if let Nav::Portal(Portal { target, cost, .. }) =
                    self.nav(node.pos).unwrap_or(Nav::Impassable)
                {
                    // If the target is in the same chunk, skip
                    if node.chunk_index == self.chunk_at_position(target).unwrap().index() {
                        continue;
                    }

                    // Create a path from the portal node to its target
                    let path = Path::from_slice(&[node.pos, target], cost);

                    connections.push((node.pos, target, path));
                }
            }

            // Check all the adjacent positions of the node, taking into account cardinal/ordinal settings
            let directions: Box<dyn Iterator<Item = Dir>> =
                if self.chunk_settings.diagonal_connections {
                    Box::new(Dir::all())
                } else {
                    Box::new(Dir::cardinal())
                };

            for dir in directions {
                let dir_vec = dir.offset();

                let nx = node.pos.x as i32 + dir_vec.x;
                let ny = node.pos.y as i32 + dir_vec.y;
                let nz = node.pos.z as i32 + dir_vec.z;

                if let Some(neighbor) = self
                    .graph
                    .node_at(UVec3::new(nx as u32, ny as u32, nz as u32))
                {
                    // Check if neighbor is in a different chunk
                    if node.chunk_index != neighbor.chunk_index {
                        let path = Path::from_slice(&[node.pos, neighbor.pos], 1);

                        connections.push((node.pos, neighbor.pos, path));
                    }
                }
            }
        }

        for (node_pos, neighbor_pos, path) in connections {
            self.graph.connect_node(node_pos, neighbor_pos, path);
        }

        /*for node in self.graph.nodes() {
            if node.portal {
                let chunk = self.chunk_at_position(node.pos).unwrap();
                println!(
                    "Portal node: {:?} at Chunk {:?}:{:?}",
                    node,
                    chunk.min(),
                    chunk.max()
                );
            }
        }*/
    }

    /// Returns the `Chunk` for the given position `UVec3` in the grid.
    pub(crate) fn chunk_at_position(&self, pos: UVec3) -> Option<&Chunk> {
        self.chunks.iter().find(|&chunk| {
            pos.x >= chunk.min().x
                && pos.x < chunk.max().x
                && pos.y >= chunk.min().y
                && pos.y < chunk.max().y
                && pos.z >= chunk.min().z
                && pos.z < chunk.max().z
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
        mask: Option<&NavMask>,
        refined: bool,
    ) -> Option<Path> {
        if self.needs_build() {
            return None;
        }

        let mask: NavMaskData = match mask {
            Some(nav_mask) => nav_mask.clone().into(),
            None => NavMaskData::new(),
        };

        reroute_path(self, path, start, goal, blocking, &mask, refined)
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
        if self.needs_build() {
            return false;
        }

        pathfind(self, start, goal, &HashMap::new(), &NavMaskData::new(), false, false, false).is_some()
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
    pub fn pathfind_old(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        mask: Option<&mut NavMask>,
        partial: bool,
    ) -> Option<Path> {
        if self.needs_build() {
            return None;
        }

        match mask {
            Some(nav_mask) => {
                if let Ok(mut mask_data) = nav_mask.data.lock() {
                    pathfind_new(self, start, goal, blocking, &mut mask_data, partial, true, false)
                } else {
                    None
                }
            },
            None => {
                let mut empty_mask = NavMaskData::new();
                pathfind_new(self, start, goal, blocking, &mut empty_mask, partial, true, false)
            }
        }
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
        mask: Option<&mut NavMask>,
        partial: bool,
    ) -> Option<Path> {
        if self.needs_build() {
            return None;
        }

        match mask {
            Some(nav_mask) => {
                if let Ok(mut mask_data) = nav_mask.data.lock() {
                    pathfind_new(self, start, goal, blocking, &mut mask_data, partial, false, false)
                } else {
                    None
                }
            },
            None => {
                let mut empty_mask = NavMaskData::new();
                pathfind_new(self, start, goal, blocking, &mut empty_mask, partial, false, false)
            }
        }
    }

    /// Generate a path using HPA* pathfinding that ONLY returns waypoints needed to avoid walls.
    /// Games with continous real-time movement can use this to generate paths where the movement only needs the key positions to avoid running into objects and walls.
    ///
    /// # Arguuments
    /// * `start` - The starting position in the grid.
    /// * `goal` - The goal position in the grid.
    /// * `mask` - An optional [`NavMask`] which can dynamically alter navigation cells only for this request.
    /// * `partial` - Whether to allow partial paths (i.e., if the goal is unreachable, return the closest reachable point).
    /// # Returns
    /// A waypoint only [`Path`] if successful, or `None` if no viable path could be found.
    ///
    pub fn pathfind_waypoints(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        mask: Option<&mut NavMask>,
        partial: bool,
    ) -> Option<Path> {
        if self.needs_build() {
            return None;
        }

        match mask {
            Some(nav_mask) => {
                if let Ok(mut mask_data) = nav_mask.data.lock() {
                    pathfind_new(self, start, goal, blocking, &mut mask_data, partial, false, true)
                } else {
                    None
                }
            },
            None => {
                let mut empty_mask = NavMaskData::new();
                pathfind_new(self, start, goal, blocking, &mut empty_mask, partial, false, true)
            }
        }
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
        mask: Option<&NavMask>,
        partial: bool,
    ) -> Option<Path> {
        if self.needs_build() {
            return None;
        }

        // Lock and get the underlying data from the NavMask
        let mask: NavMaskData = match mask {
            Some(nav_mask) => nav_mask.clone().into(),
            None => NavMaskData::new(),
        };

        pathfind_astar(
            &self.neighborhood,
            &self.grid.view(),
            start,
            goal,
            blocking,
            &mask,
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
        mask: Option<&NavMask>,
        partial: bool,
    ) -> Option<Path> {
        if self.needs_build() {
            return None;
        }

        // Lock and get the underlying data from the NavMask
        let mask: NavMaskData = match mask {
            Some(nav_mask) => nav_mask.clone().into(),
            None => NavMaskData::new(),
        };

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

        let mask_local = mask.translate_by(-min);

        // Run pathfinding on the subview
        let result = pathfind_astar(
            &self.neighborhood,
            &view,
            start_local,
            goal_local,
            &blocking_local,
            &mask_local,
            partial,
        );

        // Convert path result back to global positions
        result.map(|mut path| {
            path.translate_by(min.as_uvec3());
            path
        })
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
    pub fn pathfind_thetastar(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        mask: Option<&NavMask>,
        partial: bool,
    ) -> Option<Path> {
        if self.needs_build() {
            return None;
        }

        // Lock and get the underlying data from the NavMask
        let mask: NavMaskData = match mask {
            Some(nav_mask) => nav_mask.clone().into(),
            None => NavMaskData::new(),
        };

        pathfind_thetastar(
            &self.neighborhood,
            &self.grid.view(),
            start,
            goal,
            blocking,
            &mask,
            partial,
        )
    }

    /// Manually get a path from the grid.
    /// Arguments:
    /// * `request` - Provide a [`PathfindRequest`] containing the customization options for the pathfinding.
    /// # Returns 
    /// A [`Path`] if successful, or `None` if no viable path could be found.
    pub fn pathfind(&self, request: &mut PathfindRequest) -> Option<Path> {
        if self.needs_build() {
            log::error!("Grid is dirty and needs to be built/rebuilt before pathfinding can be performed.");
            return None;
        }

        let start_time = std::time::Instant::now();

        let empty_blocking = HashMap::new();
        let blocking = request.blocking.unwrap_or(&empty_blocking);

        let path = match request.algorithm {
            PathfindMode::Refined => {
                match request.mask.as_mut() {
                    Some(nav_mask) => {
                        if let Ok(mut data) = nav_mask.data.lock() {
                            pathfind_new(
                                self, request.start, request.goal, blocking,
                                &mut *data, request.partial, true, false,
                            )
                        } else {
                            None // Return None if lock is contended, don't wait
                        }
                    },
                    None => {
                        let mut empty_mask = NavMaskData::new();
                        pathfind_new(
                            self, request.start, request.goal, blocking,
                            &mut empty_mask, request.partial, true, false,
                        )
                    }
                }
            },
            PathfindMode::Coarse => {
                match request.mask.take() {
                    Some(nav_mask) => {
                        let mut data = nav_mask.data.lock().expect("Failed to lock NavMask data");
                        pathfind_new(
                            self,
                            request.start,
                            request.goal,
                            blocking,
                            &mut *data,
                            request.partial,
                            false,
                            false,
                        )
                    },
                    None => {
                        let mut empty_mask = NavMaskData::new();
                        pathfind_new(
                            self,
                            request.start,
                            request.goal,
                            blocking,
                            &mut empty_mask,
                            request.partial,
                            false,
                            false,
                        )
                    }
                }
            },
            PathfindMode::AStar => {
                match &request.mask {
                    Some(nav_mask) => {
                        let data = nav_mask.data.lock().expect("Failed to lock NavMask data");
                        pathfind_astar(
                            &self.neighborhood,
                            &self.grid.view(),
                            request.start,
                            request.goal,
                            blocking,
                            &*data,
                            request.partial,
                        )
                    },
                    None => {
                        let empty_mask = NavMaskData::new();
                        pathfind_astar(
                            &self.neighborhood,
                            &self.grid.view(),
                            request.start,
                            request.goal,
                            blocking,
                            &empty_mask,
                            request.partial,
                        )
                    }
                }
            },
            PathfindMode::Waypoints => {
                match request.mask.take() {
                    Some(nav_mask) => {
                        let mut data = nav_mask.data.lock().expect("Failed to lock NavMask data");
                        pathfind_new(
                            self,
                            request.start,
                            request.goal,
                            blocking,
                            &mut *data,
                            request.partial,
                            false,
                            true,
                        )
                    },
                    None => {
                        let mut empty_mask = NavMaskData::new();
                        pathfind_new(
                            self,
                            request.start,
                            request.goal,
                            blocking,
                            &mut empty_mask,
                            request.partial,
                            false,
                            true,
                        )
                    }
                }
            },
            PathfindMode::ThetaStar => {
                match &request.mask {
                    Some(nav_mask) => {
                        let data = nav_mask.data.lock().expect("Failed to lock NavMask data");
                        pathfind_thetastar(
                            &self.neighborhood,
                            &self.grid.view(),
                            request.start,
                            request.goal,
                            blocking,
                            &*data,
                            request.partial,
                        )
                    },
                    None => {
                        let empty_mask = NavMaskData::new();
                        pathfind_thetastar(
                            &self.neighborhood,
                            &self.grid.view(),
                            request.start,
                            request.goal,
                            blocking,
                            &empty_mask,
                            request.partial,
                        )
                    }
                }
            },
        };

        log::info!("Pathfinding took {:?}", start_time.elapsed());

        path
    }
}

fn compute_cell_neighbors<N: Neighborhood>(
    neighborhood: &N,
    grid_view: &ArrayView3<NavCell>,
    pos: UVec3,
) -> (UVec3, u32, Vec<UVec3>) {
    let bits = neighborhood.neighbors(grid_view, pos);
    let nav = grid_view[[pos.x as usize, pos.y as usize, pos.z as usize]].nav();

    let special = match nav {
        Nav::Portal(p) => vec![p.target],
        _ => Vec::new(),
    };

    (pos, bits, special)
}

#[cfg(test)]
mod tests {
    use bevy::math::UVec3;

    use crate::{
        dir::Dir, grid::{
            ChunkSettings, CollisionSettings, Grid, GridInternalSettings, GridSettings,
            GridSettingsBuilder, NavCell, NavSettings, NeighborhoodSettings,
        }, nav::{Nav, Portal}, neighbor::OrdinalNeighborhood3d, pathfind::PathfindRequest, prelude::{CardinalNeighborhood, OrdinalNeighborhood}
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

    const GRID_SETTINGS_3D: GridSettings = GridSettings(GridInternalSettings {
        dimensions: UVec3::new(12, 12, 12),
        chunk_settings: ChunkSettings {
            size: 4,
            depth: 4,
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
    pub fn test_build() {
        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS);
        grid.build();

        assert!(!grid.needs_build());
        assert_eq!(grid.grid.shape(), [12, 12, 1]);
        assert_eq!(grid.chunks.len(), 9); // 3x3 chunks of size 4

        // Test 3d
        let mut grid_3d = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS_3D);
        grid_3d.build();

        assert!(!grid_3d.needs_build());
        assert_eq!(grid_3d.grid.shape(), [12, 12, 12]);
        assert_eq!(grid_3d.chunks.len(), 27); // 3x3x3 chunks of size 4
    }

    #[test]
    pub fn test_faces() {
        let grid_settings = GridSettingsBuilder::new_2d(4, 4)
            .chunk_size(4)
            .enable_diagonal_connections()
            .build();

        let mut grid = Grid::<OrdinalNeighborhood>::new(&grid_settings);

        // Fill grid edges with walls
        for x in 0..4 {
            for y in 0..4 {
                if x == 0 || x == 3 || y == 0 || y == 3 {
                    grid.grid[[x, y, 0]] = NavCell::new(Nav::Impassable);
                }
            }
        }

        let chunk = grid.chunks[[0, 0, 0]].clone();

        let mut faces = Vec::new();

        faces.push(chunk.face(&grid.grid, Dir::North));
        faces.push(chunk.face(&grid.grid, Dir::East));
        faces.push(chunk.face(&grid.grid, Dir::South));
        faces.push(chunk.face(&grid.grid, Dir::West));

        for face in faces {
            for cell in face.iter() {
                assert!(cell.is_impassable());
            }
        }
    }

    #[test]
    pub fn test_calculate_face_nodes() {
        let grid = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS);

        let chunk = grid.chunks.iter().next().unwrap().clone();
        let neighbor_chunk = grid.chunks.iter().nth(1).unwrap().clone();

        let edges = grid.calculate_face_nodes(
            chunk.face(&grid.grid, Dir::North),
            neighbor_chunk.face(&grid.grid, Dir::South),
            chunk.clone(),
            Dir::North,
        );

        assert_eq!(edges.len(), 1);

        // 3D
        let grid_settings = GridSettingsBuilder::new_3d(12, 12, 12)
            .chunk_size(4)
            .chunk_depth(4)
            .enable_diagonal_connections()
            .build();

        let grid_3d = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);
        // Center chunk
        let chunk_3d = grid_3d.chunks[[1, 1, 1]].clone();
        // Neighbor chunk
        let neighbor_chunk_3d = grid_3d.chunks[[1, 1, 2]].clone();

        let edges_3d = grid_3d.calculate_face_nodes(
            chunk_3d.face(&grid_3d.grid, Dir::North),
            neighbor_chunk_3d.face(&grid_3d.grid, Dir::South),
            chunk_3d.clone(),
            Dir::North,
        );

        assert_eq!(edges_3d.len(), 1);
        assert_eq!(edges_3d[0].pos, UVec3::new(2, 2, 0));
        assert_eq!(edges_3d[0].dir, Some(Dir::North));
    }

    #[test]
    pub fn test_build_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build_nodes();

        let nodes = grid.graph.nodes();

        let expected = vec![
            UVec3::new(2, 3, 0),
            UVec3::new(3, 2, 0),
            UVec3::new(2, 7, 0),
            UVec3::new(3, 6, 0),
            UVec3::new(2, 4, 0),
            UVec3::new(3, 10, 0),
            UVec3::new(2, 8, 0),
        ];
        for pos in expected {
            assert!(
                nodes.iter().any(|n| n.pos == pos),
                "Missing node at {pos:?}"
            );
        }

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

        // Check that there's no nodes with duplicate positions
        let mut positions = std::collections::HashSet::new();
        for node in grid.graph.nodes() {
            assert!(
                positions.insert(node.pos),
                "Duplicate node at {:?}",
                node.pos
            );
        }

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
    pub fn test_build_nodes_3d() {
        // Test 3d
        let grid_settings = GridSettingsBuilder::new_3d(48, 48, 48)
            .chunk_size(16)
            .chunk_depth(16)
            .build();

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        grid.build_nodes();

        // Get the center nodes of the center chunk
        let center_chunk = grid.chunks[[1, 1, 1]].clone();
        let center_nodes = grid.graph.nodes_in_chunk(&center_chunk);

        // Print all nodes and their directions
        for node in center_nodes.clone() {
            println!("Node at {:?} with direction {:?}", node.pos, node.dir);
        }

        assert_eq!(center_nodes.len(), 14);
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
                GRID_SETTINGS.0.chunk_settings.size,
                GRID_SETTINGS.0.chunk_settings.size,
                GRID_SETTINGS.0.chunk_settings.depth
            )
        );
    }

    #[test]
    pub fn test_get_all_nodes_in_chunk() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);
        grid.build_nodes();

        let nodes = grid.graph.nodes_in_chunk(&grid.chunks[[0, 0, 0]]);

        assert_eq!(nodes.len(), 2);
    }

    #[test]
    pub fn test_get_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(10, 10, 0), UVec3::new(4, 4, 0)));
        let raw_path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(10, 10, 0), UVec3::new(4, 4, 0)).astar());

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
    fn test_calculate_face_nodes_returns_center() {
        //2D
        let grid_settings = GridSettingsBuilder::new_2d(64, 64).chunk_size(32).build();

        let grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        let start_edge = grid.chunks[[0, 0, 0]].face(&grid.grid, Dir::North);
        let end_edge = grid.chunks[[0, 1, 0]].face(&grid.grid, Dir::South);

        let nodes = grid.calculate_face_nodes(
            start_edge,
            end_edge,
            grid.chunks[[0, 0, 0]].clone(),
            Dir::North,
        );

        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].pos, UVec3::new(16, 0, 0));

        //3D
        let grid_settings_3d = GridSettingsBuilder::new_3d(64, 64, 64)
            .chunk_size(32)
            .chunk_depth(32)
            .build();

        let grid_3d: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings_3d);

        let start_edge = grid_3d.chunks[[0, 0, 0]].face(&grid_3d.grid, Dir::North);
        let end_edge = grid_3d.chunks[[0, 1, 0]].face(&grid_3d.grid, Dir::South);
        let nodes_3d = grid_3d.calculate_face_nodes(
            start_edge,
            end_edge,
            grid_3d.chunks[[0, 0, 0]].clone(),
            Dir::North,
        );

        assert_eq!(nodes_3d.len(), 1);
        assert_eq!(nodes_3d[0].pos, UVec3::new(16, 16, 0));
        assert_eq!(nodes_3d[0].dir, Some(Dir::North));
    }

    #[test]
    fn test_calculate_edge_nodes_returns_center() {
        let grid_settings = GridSettingsBuilder::new_2d(64, 64).chunk_size(32).build();

        let grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        let start_edge = grid.chunks[[0, 0, 0]].edge(&grid.grid, Dir::NorthUp);
        let end_edge = grid.chunks[[0, 1, 0]].edge(&grid.grid, Dir::SouthDown);

        let nodes = grid.calculate_edge_nodes(
            start_edge,
            end_edge,
            grid.chunks[[0, 0, 0]].clone(),
            Dir::North,
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

        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(7, 7, 0), UVec3::new(121, 121, 0)));

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
        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(31, 31, 3)));

        assert!(path.is_some());
    }

    #[test]
    pub fn test_get_astar_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(10, 10, 0)).astar());

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

    #[test]
    fn test_mark_dirty_for_pos_marks_expected_chunks_and_edges() {
        use crate::dir::Dir;
        use bevy::math::UVec3;

        // 8x8 grid, 4x4 chunks => 2x2 chunks
        let grid_settings = GridSettingsBuilder::new_2d(8, 8).chunk_size(4).build();
        let mut grid: Grid<CardinalNeighborhood> = Grid::new(&grid_settings);

        // NOTHING should be dirty after build
        grid.build();
        assert!(
            grid.dirty_chunks.is_empty(),
            "No chunks should be dirty after initial build"
        );

        for (chunk_coords, chunk) in grid.chunks.indexed_iter() {
            assert!(
                !chunk.has_dirty_edges(),
                "Chunk {chunk_coords:?} should not have dirty edges after build",
            );
        }

        // Mark a position in the center of chunk (0, 0, 0)
        let center_pos = UVec3::new(1, 1, 0);
        grid.mark_dirty_for_pos(center_pos);

        // Only the containing chunk should be dirty
        assert!(
            grid.dirty_chunks.contains(&(0, 0, 0)),
            "Containing chunk should be dirty"
        );
        let chunk = &grid.chunks[[0, 0, 0]];

        // No edges should be dirty (not an edge position)
        for dir in Dir::cardinal() {
            if dir == Dir::Up || dir == Dir::Down {
                continue; // Skip vertical edges in 2D grid
            }
            assert!(
                !chunk.is_edge_dirty(dir),
                "Edge {dir:?} of chunk (0,0,0) should NOT be dirty for interior position"
            );
        }

        // Neighboring chunks should not be dirty
        let neighbor_coords = [(1, 0, 0), (0, 1, 0)];
        for &(nx, ny, nz) in &neighbor_coords {
            assert!(
                !grid.dirty_chunks.contains(&(nx, ny, nz)),
                "Neighbor chunk ({nx},{ny},{nz}) should NOT be dirty"
            );
        }

        // Now test edge position
        let edge_pos = UVec3::new(3, 1, 0); // Right edge of chunk (0, 0, 0)
        grid.mark_dirty_for_pos(edge_pos);

        let chunk = &grid.chunks[[0, 0, 0]];
        assert!(
            chunk.is_edge_dirty(Dir::East),
            "East edge should now be dirty"
        );

        assert!(
            grid.dirty_chunks.contains(&(1, 0, 0)),
            "Neighbor chunk (1, 0, 0) should be dirty due to edge cell"
        );

        let neighbor = &grid.chunks[[1, 0, 0]];
        assert!(
            neighbor.is_edge_dirty(Dir::West),
            "Neighbor chunk (1, 0, 0) should have WEST edge dirty"
        );
    }

    // Helper function to check graph node invariants
    fn assert_graph_node_invariants(grid: &Grid<CardinalNeighborhood>) {
        let nodes = grid.graph.nodes();
        let mut node_positions = std::collections::HashSet::new();

        for node in &nodes {
            if node_positions.contains(&node.pos) {
                panic!("Duplicate node found at position {:?}", node.pos);
            }
            node_positions.insert(node.pos);

            // Ensure that no node has edges to itself
            for (neighbor, _) in &node.edges {
                assert!(
                    neighbor != &node.pos,
                    "Node {:?} should not have an edge to itself",
                    node.pos
                );
            }
        }
    }

    #[test]
    fn test_dirty_chunk_rebuild_and_pathfinding() {
        use crate::nav::Nav;
        use bevy::math::UVec3;

        // Create a 16x16 grid with 4x4 chunks (so 4x4 chunks)
        let grid_settings = GridSettingsBuilder::new_2d(16, 16).chunk_size(4).build();
        let mut grid: Grid<CardinalNeighborhood> = Grid::new(&grid_settings);

        // Build the grid initially
        grid.build();

        assert_graph_node_invariants(&grid);

        // There should be a path from (0,0,0) to (15,15,0)
        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(15, 15, 0)));
        assert!(path.is_some(), "Path should exist in empty grid");

        // Block a vertical wall at x=8
        for y in 0..16 {
            grid.set_nav(UVec3::new(8, y, 0), Nav::Impassable);
        }
        grid.build();

        assert_graph_node_invariants(&grid);

        assert!(
            !grid.graph.nodes().is_empty(),
            "There should be nodes in the graph after rebuilds"
        );

        // Now there should be no path from left to right
        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(15, 15, 0)));

        assert!(path.is_none(), "Path should not exist after wall");

        // Astar should never panic on getting neighbors
        // if everything is set up correctly
        let _ = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(15, 15, 0)).astar());

        // Open a gap in the wall at (8,8)
        grid.set_nav(UVec3::new(8, 8, 0), Nav::Passable(1));
        grid.build();

        // Now a path should exist again, and should pass through (8,8,0)
        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(15, 15, 0)));
        assert!(path.is_some(), "Path should exist after opening gap");
        let path = path.unwrap();
        assert!(
            path.path().contains(&UVec3::new(8, 8, 0)),
            "Path should go through the gap at (8,8,0)"
        );

        // Check that only the affected chunks are dirty after set_nav
        // (After build, dirty_chunks should be cleared)
        assert!(
            grid.dirty_chunks.is_empty(),
            "Dirty chunks should be cleared after build"
        );

        // Let's make sure the nodes in graph look correct after all the rebuilds
        assert_graph_node_invariants(&grid);

        // Astar should never panic on getting neighbors
        // if everything is set up correctly
        let _ = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(15, 15, 0)).astar());
    }

    #[test]
    fn test_needs_build() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);
        assert!(grid.needs_build(), "Grid should need build initially");

        // After building, it should still not need a build
        grid.build();
        assert!(
            !grid.needs_build(),
            "Grid should not need build after initial build"
        );

        // Mark a position dirty and check needs_build
        grid.set_nav(UVec3::new(1, 1, 0), Nav::Impassable);
        assert!(
            grid.needs_build(),
            "Grid should need build after marking dirty"
        );

        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(10, 10, 0)));

        assert!(path.is_none(), "Path should not exist after marking dirty");

        grid.build();
        assert!(
            !grid.needs_build(),
            "Grid should not need build after rebuild"
        );
    }

    #[test]
    fn test_path_with_portal() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        // Create a portal at (5,5,0) leading to (10,10,0)
        grid.set_nav(
            UVec3::new(2, 2, 0),
            Nav::Portal(Portal {
                target: UVec3::new(10, 10, 0),
                cost: 1,
                one_way: false,
            }),
        );

        // Fill 5,0,0 to 5,12,0 with impassable nav
        for y in 0..12 {
            grid.set_nav(UVec3::new(5, y, 0), Nav::Impassable);
        }

        grid.build();

        // Ensure node is at portal position
        let portal_node = grid.graph.node_at(UVec3::new(2, 2, 0));
        assert!(portal_node.is_some(), "Portal node should exist");
        let portal_node = portal_node.unwrap();

        portal_node
            .edges
            .iter()
            .find_map(|(neighbor, edge)| {
                if *neighbor == UVec3::new(10, 10, 0) {
                    assert_eq!(edge.cost(), 1, "Portal cost should be 1");
                    Some(())
                } else {
                    None
                }
            })
            .expect("Portal edge to (10,10,0) should exist");

        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(11, 11, 0)));

        assert!(path.is_some(), "Path should exist with portal");
    }

    #[test]
    fn test_stairs() {
        let grid_settings = GridSettingsBuilder::new_3d(16, 16, 4)
            .chunk_size(4)
            .chunk_depth(1)
            .default_impassable()
            .build();

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        // Fill 0,0,0 to 7,7,0 with passable nav
        for x in 0..8 {
            for y in 0..16 {
                grid.set_nav(UVec3::new(x, y, 0), Nav::Passable(1));
            }
        }

        // Fill 8,8,2 to 15, 15,3 with passable nav
        for x in 8..16 {
            for y in 0..16 {
                grid.set_nav(UVec3::new(x, y, 2), Nav::Passable(1));
            }
        }

        grid.set_nav(UVec3::new(7, 4, 1), Nav::Passable(1));

        grid.set_nav(
            UVec3::new(7, 4, 0),
            Nav::Portal(Portal {
                target: UVec3::new(7, 4, 2),
                cost: 1,
                one_way: false,
            }),
        );

        grid.set_nav(
            UVec3::new(7, 4, 2),
            Nav::Portal(Portal {
                target: UVec3::new(7, 4, 0),
                cost: 1,
                one_way: false,
            }),
        );

        grid.build();

        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(0, 0, 0), UVec3::new(12, 4, 2)));

        assert!(path.is_some(), "Path should exist with portal");

        let path = grid.pathfind(&mut PathfindRequest::new(UVec3::new(12, 4, 2), UVec3::new(0, 0, 0)));

        assert!(path.is_some(), "Path should exist with portal in reverse");
    }
}
