## v0.6.1
- Fix for Waypoint refinement not factoring in cell cost accurately.

## v0.6.0
- Bevy 0.18 Support

## v0.5.0
- Bevy 0.17 Support

### Features
- `CardinalIsoNeighborhood` added for better 3d isometric cardinal movement
- `3d_voxel_world` example: Demonstrates 3d pathfinding. Thanks to [@KarlsonxD](https://github.com/JtotheThree/bevy_northstar/pull/34) for adding this!

## v0.4.0

**BREAKING**
- `Grid::pathfind` now requires `PathfindArgs` as an argument. The other pathfinding methods on Grid have been removed. `Grid::pathfind` now handles all algorithms set in the arguments.

See the [migration guide](https://jtothethree.github.io/bevy_northstar/migrations/002_v0.4.0.html) for more details on how to migrate.

### Features
- *Navigation Masks*: Per Agent masks can now be created to alter the cost or impassability only for that agent.
- *Any-Angle Pathfinding*: Waypoint and Theta* algorithms. Designed for smooth movement and returns only the points needed to avoid walls. Waypoint uses HPA* and is ~10× faster than Theta* while returning similar paths.
- Path refinement in HPA* is now anti-aliased to remove path jaggedness.
- Set the default plugin pathfinding algorithm with `PathfindSettings`.
- Lots of optimization.

## v0.3.2

**BREAKING**
- `Dir` enum variants renamed to CamelCase and reordered as more directions were added.
- `Nav::Ramp` replaced with `Nav::Portal` for general purpose transitions (ramps, portals, elevators, etc).

### Features
- Improved 3D support to better handle 2.5D (isometric maps, 2d tilemaps with height).
- New `isometric` example showcasing full 2.5D pathfinding and `Nav::Portal` usage.
- `Nav::Portal` to support ramps/stairs, elevators, or teleporters during pathfinding.
- Added `NoCornerCuttingFlat` neighbor filter: blocks diagonal moves on the same Z level but allows vertical diagonal jumps. Useful for 2.5D movement.
- Added `Path::next()` to peek ahead in the path. Useful for checking the next position against your map.
- Added trimming to `PathfindMode::Coarse` to ensure that only one high level node is pathed to in the goal chunk.
- Chunk depth is no longer required to be divisible by the grid depth.
- All crate components now derive `Reflect` and are registered for inspection.

### Debug Features
- `DebugGrid::set_depth(u32)` and `depth()` added to allow drawing debug grid at a specific Z depth.
- `DebugDepthYOffsets` component aligns the debug grid with custom Z-level Y offsets.
- `DebugCursor` component and `DebugGridBuilder::show_connections_on_hover()` added to only show cached paths and edges on node hover.

### Fixes
- Fixed race condition in `next_position` that sometimes caused missed `NextPos` component inserts.
- Fixed agents in rare occasions blocking themselves on next path step.
- Fixed HPA* refinement sometimes violating `NeighborFilter` constraints.

## v0.3.1
### Features
* The grid can now be updated during gameplay. After calling `Grid::set_nav()` to change navigation at a position just call `Grid::build()` again. 
* `Grid::build()` now rebuilds only areas affected by `set_nav()` when calling it after the initial build.
* `Grid::build()` now processes chunks in parallel! Large performance boost for the initial build and allows rebuilding larger areas of the grid in a single frame.
* `parallel` feature has been added and is enabled by default. You can disable default features if targetting WASM and multithreading gives you issue.
* Right-click a tile in the basic example to change the navigation data and dynamically rebuild the grid.

## v0.3.0
**BREAKING CHANGES**
`GridSettings` has been replaced with a builder `GridSettingsBuilder`.
`GridPos` component has been renamed to `AgentPos`.
`Point` has been removed and fully reworked as `Nav` and `NavCell`.
`Pathfind` component has been reworked with a fluent/chain constructor pattern.
`DebugMap` component has been renamed to `DebugGrid` and reworked.
`DebugMapType` has been renamed to `DebugTilemapType`.
`DebugPath` has removed tile_width, tile_height, and map_type.

See the [migration guide](https://jtothethree.github.io/bevy_northstar/migrations/001_v0.3.0.html) for more details on how to migrate.

### Features
* Updated the basic example to demonstrate more realistic usage of the crate.
* `NorthstarPlugin` systems now stagger agent processing based on the `NorthstarPluginSettings` configuration.
* Added the `NorthstarPluginSettings` resource to configure how many agents can be processed per frame by the pathfinding systems.
* Added the `NeighborFilter` trait. Filters can now be applied to neighbor generation—for example, to prevent corner clipping. Multiple filters can be chained.
* Added `NoCornerClipping`, `NoCornerCutting` filters which customize diagonal movement around and/or through wall corners.
* Neighbors are now precomputed as bitmasks and cached during `Grid::build()`. This provides a general performance boost and a significant improvement when using neighbor filters.
* Added `PathfindMode::Coarse`, which returns a hierarchical path without refinement. Refinement is the step that optimizes a path for shortest distance
* `AgentOfGrid` relationship has been created to relate an entity with a specific `Grid`. This will allow supporting multiple grids in the future.
* HPA* path refinement has been optimized for performance. A* performance has been further optimized.
* Optimized the collision avoidance system by limiting checks to a localized search area, reducing stutters from full A* searches.

### Debug Features
* `DebugPath` now uses the `AgentOfGrid` relationship and child relationships making it cleaner to setup.
* Added `DebugGridBuilder` to simplify the creation and configuration of `DebugGrid`.

## v0.2.1
**Fixes**
* `Plugin::update_blocking_map` system now properly queries `Blocking` (previously was including all `GridPos` entities as blockers for collision). Credit [#11 @AwfulToTheEar](https://github.com/JtotheThree/bevy_northstar/pull/11).
* `pathfind` now checks grid bounds; logs an error and return `None` if out of bounds.
* Fixed repeated pathfind attempts due to unintended component reinsertion in `NorthstarPlugin`. Use `PathfindingFailed` to handle failures gracefully.
* Fixed a spawning issue in the examples where entities were not spawning with transforms adjusted for the bevy_ecs_tilemap offset.

**Features**
* Added `Grid::is_path_viable`. Returns `true` if there's a path possible from the start to the goal while skipping the refinement step that the standard `pathfind` functions runs. This should be a more performant call if you just need to test if path exists.
* Added `BlockingMap` to `prelude`

## v0.2.0
First official public release