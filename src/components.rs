//! Components for pathfinding, collision, and debugging.
use bevy::{
    color::palettes::css,
    ecs::entity::Entity,
    math::{UVec3, Vec3},
    prelude::{Color, Component},
    reflect::Reflect,
    transform::components::Transform,
};

use crate::debug::DebugTilemapType;

/// An entities position on the pathfinding [`crate::grid::Grid`].
/// You'll need to maintain this position if you use the plugin pathfinding systems.
#[derive(Component, Default, Debug, Clone, Eq, PartialEq, Hash)]
pub struct AgentPos(pub UVec3);

/****************************************
    PATHFINDING COMPONENTS
*****************************************/

/// Determines which algorithm to use for pathfinding.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PathfindMode {
    /// Hierarchical pathfinding with the final path refined with line tracing.
    #[default]
    Refined,
    /// Hierarchical pathfinding using only cached paths. Use this if you're not concerned with trying to find the shortest path.
    Coarse,
    /// Full-grid A* pathfinding without hierarchy.
    /// Useful for small grids or a turn based pathfinding path where movement cost needs to be the most accurate and cpu usage isn't a concern.
    AStar,
}

/// Insert [`Pathfind`] on an entity to pathfind to a goal.
/// Once the plugin systems have found a path, [`NextPos`] will be inserted.
#[derive(Component, Default, Debug)]
pub struct Pathfind {
    /// The goal to pathfind to.
    pub goal: UVec3,
    /// Will attempt to return the best path if full route isn't found.
    pub partial: bool,

    /// The [`PathfindMode`] to use for pathfinding.
    /// Defaults to [`PathfindMode::Refined`] which is hierarchical pathfinding with full refinement.
    pub mode: PathfindMode,
}

impl Pathfind {
    /// Creates a new [`Pathfind`] component with the given goal.
    /// An HPA* refined path will be returned by default.
    /// If you want to use a different pathfinding mode, use the [`Pathfind::mode()`] method.
    /// If you want to allow partial paths, use the [`Pathfind::partial()`] method.
    /// # Example
    /// ```
    /// use bevy::math::UVec3;
    /// use bevy_northstar::prelude::*;
    ///
    /// let pathfind = Pathfind::new(UVec3::new(5, 5, 0))
    ///     .mode(PathfindMode::AStar)
    ///     .partial();
    /// ```
    ///
    pub fn new(goal: UVec3) -> Self {
        Pathfind {
            goal,
            ..Default::default()
        }
    }

    /// Shorthand constructor for 2D pathfinding to avoid needing to construct a [`bevy::math::UVec3`].
    /// This will set the z-coordinate to 0.
    pub fn new_2d(x: u32, y: u32) -> Self {
        Pathfind {
            goal: UVec3::new(x, y, 0),
            ..Default::default()
        }
    }

    /// Shorthand constructor for 3D pathfinding to avoid needing to construct a [`bevy::math::UVec3`].
    pub fn new_3d(x: u32, y: u32, z: u32) -> Self {
        Pathfind {
            goal: UVec3::new(x, y, z),
            ..Default::default()
        }
    }

    /// Sets the pathfinding mode. See [`PathfindMode`] for options.
    pub fn mode(mut self, mode: PathfindMode) -> Self {
        self.mode = mode;
        self
    }

    /// Allow partial paths.
    /// The pathfinding system will return the best path it can find
    /// even if it can't find a full route to the goal.
    pub fn partial(mut self) -> Self {
        self.partial = true;
        self
    }
}

/// The next position in the path inserted into an entity by the pathfinding system.
/// The `pathfind` system in [`crate::plugin::NorthstarPlugin`] will insert this.
/// Remove [`NextPos`] after you've moved the entity to the next position and
/// a new [`NextPos`] will be inserted on the next frame.
#[derive(Component, Default, Debug)]
pub struct NextPos(pub UVec3);

// See src/path.rs for the Path component

/****************************************
    COLLISION COMPONENTS
*****************************************/

/// Marker component for entities that dynamically block paths during navigation.
///
/// The pathfinding system’s collision avoidance checks for entities with this component
/// to treat their positions as temporarily blocked.
///
/// **Do not** use this component for static obstacles such as walls or terrain.
/// Static geometry should be handled separately with [`crate::grid::Grid::set_nav()`] in [`crate::grid::Grid`].
#[derive(Component, Default)]
pub struct Blocking;

/// Marker component that is inserted on an entity when a collision is detected.
/// The built-in pathfinding system will try to pathfind for this entity every frame unless
/// you handle the failure in a custom way.
#[derive(Component, Default, Debug)]
pub struct PathfindingFailed;

/// Marker component that is inserted on an entity when local avoidance fails.
/// Currently this marker is handled by the [`crate::plugin::NorthstarPlugin`] `reroute_path` system and can be ignored
/// unless the desire is to handle the failure in a custom way.
#[derive(Component, Default, Debug)]
pub struct AvoidanceFailed;

/// Marker component that is inserted on an entity when path rerouting in [`crate::plugin::NorthstarPlugin`] `reroute_path` fails.
/// This happens well all avoidance options have been exhausted and the entity cannot be rerouted to its goal.
/// **You will need to handle this failure in your own system before the entity can be pathed again**.
/// Examples would be to set a new goal or wait for a certain amount of time before trying to reroute again.
#[derive(Component, Default, Debug)]
pub struct RerouteFailed;

/****************************************
    DEBUGGING COMPONENTS
*****************************************/

/// Add this component to the same entity as [`DebugPath`] to offset the debug gizmos.
/// Useful for aligning the gizmos with your tilemap rendering offset.
#[derive(Component, Default)]
pub struct DebugOffset(pub Vec3);

/// Component for debugging an entity's [`crate::path::Path`].
#[derive(Component)]
#[require(DebugOffset)]
pub struct DebugPath {
    /// The [`Color`] of the path gizmo.
    pub color: Color,
    /// Draw the HPA* high level graph path between chunk entrances.
    /// This is useful for debugging the HPA* algorithm.
    pub draw_unrefined: bool,
}

impl DebugPath {
    /// Creates a new [`DebugPath`] component with the specified color.
    /// The default color is red.
    pub fn new(color: Color) -> Self {
        DebugPath {
            color,
            draw_unrefined: false,
        }
    }
}

impl Default for DebugPath {
    fn default() -> Self {
        DebugPath {
            color: bevy::prelude::Color::Srgba(css::RED),
            draw_unrefined: false,
        }
    }
}

/// Component for debugging [`crate::grid::Grid`].
/// You need to insert [`DebugGrid`] as a child of your map.
#[derive(Reflect, Component)]
#[require(Transform)]
pub struct DebugGrid {
    pub(crate) tile_width: u32,
    pub(crate) tile_height: u32,
    pub(crate) map_type: DebugTilemapType,
    pub(crate) draw_chunks: bool,
    pub(crate) draw_cells: bool,
    pub(crate) draw_entrances: bool,
    pub(crate) draw_cached_paths: bool,
}

impl DebugGrid {
    /// The width and height of a tile in pixels. This is required because your tile pixel dimensions may not match the grid size.
    pub fn tile_size(&mut self, width: u32, height: u32) -> &Self {
        self.tile_width = width;
        self.tile_height = height;
        self
    }

    /// Set the [`DebugTilemapType`] which is used to determine how the grid is visualized (e.g., square or isometric). Align this with the style of your tilemap.
    pub fn map_type(&mut self, map_type: DebugTilemapType) -> &Self {
        self.map_type = map_type;
        self
    }

    /// Will outline the chunks that the grid is divided into.
    pub fn set_draw_chunks(&mut self, value: bool) -> &Self {
        self.draw_chunks = value;
        self
    }

    /// Toggle draw_chunks.
    pub fn toggle_chunks(&mut self) -> &Self {
        self.draw_chunks = !self.draw_chunks;
        self
    }

    /// Will draw the [`crate::nav::NavCell`]s in your grid. This should align to each tile in your tilemap.
    pub fn set_draw_cells(&mut self, value: bool) -> &Self {
        self.draw_cells = value;
        self
    }

    /// Toggle draw_cells.
    pub fn toggle_cells(&mut self) -> &Self {
        self.draw_cells = !self.draw_cells;
        self
    }

    /// The entrances are the cells in each chunk that connect to other chunks.
    /// This will draw the entrances calculated by the HPA* algorithm.
    /// This is very useful for debugging the HPA* algorithm and understanding how chunks are connected to build the hierarchy.
    pub fn set_draw_entrances(&mut self, value: bool) -> &Self {
        self.draw_entrances = value;
        self
    }

    /// Toggle draw_entrances.
    pub fn toggle_entrances(&mut self) -> &Self {
        self.draw_entrances = !self.draw_entrances;
        self
    }

    /// Draws the internal cached paths between the entrances in the same chunk.
    /// This is only really useful for debugging odd issues with the HPA* crate.
    pub fn set_draw_cached_paths(&mut self, value: bool) -> &Self {
        self.draw_cached_paths = value;
        self
    }

    /// Toggle draw_cached_paths.
    pub fn toggle_cached_paths(&mut self) -> &Self {
        self.draw_cached_paths = !self.draw_cached_paths;
        self
    }
}

/// Builder for [`DebugGrid`].
/// Use this to configure debugging for a grid before inserting it into your map entity.
/// Insert the returned [`DebugGrid`] as a child of the entity with your [`crate::grid::Grid`] component.
pub struct DebugGridBuilder {
    tile_width: u32,
    tile_height: u32,
    tilemap_type: DebugTilemapType,
    draw_chunks: bool,
    draw_cells: bool,
    draw_entrances: bool,
    draw_cached_paths: bool,
}

impl DebugGridBuilder {
    /// Creates a new [`DebugGridBuilder`] with the specified tile width and height.
    pub fn new(tile_width: u32, tile_height: u32) -> Self {
        Self {
            tile_width,
            tile_height,
            tilemap_type: DebugTilemapType::Square,
            draw_chunks: false,
            draw_cells: false,
            draw_entrances: false,
            draw_cached_paths: false,
        }
    }

    /// Sets the draw type of the tilemap.
    /// This is used to determine how the grid is visualized (e.g., square or isometric).
    /// Use the shorthand methods [`DebugGridBuilder::isometric()`] to set this instead.
    pub fn tilemap_type(mut self, tilemap_type: DebugTilemapType) -> Self {
        self.tilemap_type = tilemap_type;
        self
    }

    pub fn isometric(mut self) -> Self {
        self.tilemap_type = DebugTilemapType::Isometric;
        self
    }

    /// Enables drawing the outline of chunks the grid is divided into.
    pub fn enable_chunks(mut self) -> Self {
        self.draw_chunks = true;
        self
    }

    /// Enables drawing the [`crate::nav::NavCell`]s in your grid.
    /// Useful for visualizing the navigation movement options in your grid and how they align with your tilemap.
    pub fn enable_cells(mut self) -> Self {
        self.draw_cells = true;
        self
    }

    /// Enables drawing the chunk entrances calculated by the HPA* algorithm.
    /// This is useful for debugging how chunks are connected and how the HPA* algorithm builds its hierarchy.
    pub fn enable_entrances(mut self) -> Self {
        self.draw_entrances = true;
        self
    }

    /// Enables drawing the cached paths between entrances in the same chunk.
    /// Is only useful for debugging odd issues with the HPA* crate.
    pub fn enable_cached_paths(mut self) -> Self {
        self.draw_cached_paths = true;
        self
    }

    /// Builds the final [`DebugGrid`] component with the configured settings to be inserted into your map entity.
    /// You need to call this methdod to finalize the builder and create the component.
    pub fn build(self) -> DebugGrid {
        DebugGrid {
            tile_width: self.tile_width,
            tile_height: self.tile_height,
            map_type: self.tilemap_type,
            draw_chunks: self.draw_chunks,
            draw_cells: self.draw_cells,
            draw_entrances: self.draw_entrances,
            draw_cached_paths: self.draw_cached_paths,
        }
    }
}

/****************************************
    GRID RELATIONSHIPS
*****************************************/

/// The [`AgentOfGrid`] component is used to create a relationship between an agent or entity and the grid it belongs to.
/// Pass your [`crate::grid::Grid`] entity to this component and insert it on your entity to relate it so all
/// pathfinding systems and debugging know which grid to use.
#[derive(Component)]
#[relationship(relationship_target = GridAgents)]
pub struct AgentOfGrid(pub Entity);

/// The [`GridAgents`] component is used to store a list of entities that are agents in a grid.
/// See [`AgentOfGrid`] for more information on how to associate an entity with a grid.
#[derive(Component)]
#[relationship_target(relationship = AgentOfGrid, linked_spawn)]
pub struct GridAgents(Vec<Entity>);

impl GridAgents {
    pub fn entities(&self) -> &[Entity] {
        &self.0
    }
}
