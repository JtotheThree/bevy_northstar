## v0.3.0
**BREAKING CHANGES**
`GridSettings` has been replaced with a builder `GridSettingsBuilder`.
`GridPos` component has been renamed to `AgentPos`.
`Point` has been removed and fully reworked as `Nav` and `NavCell`.
`Pathfind` component has been reworked with a fluent/chain constructor pattern.
`DebugMap` component has been renamed to `DebugGrid` and reworked.
`DebugMapType` has been renamed to `DebugTilemapType`.
`DebugPath` has removed tile_width, tile_height, and map_type.

See the migration guide for more details on how to migrate.

### Features
* Updated the basic example to demonstrate a very minimal visual example of using the crate.
* `NorthstarPluginSettings` resource added to configure the maximum agents per frame that can be handled by the pathfinding systems.
* `NorthstarPlugin` systems will now stagger agents defined by `NorthstarPluginSettings`.
* Added `NeighborFilter` trait. You can now apply filters to the neighbors to prevent things like corner clipping. Filters can be chained.
* Added `NoCornerClipping` neighbor filter to optionally prevent ordinal movement through wall corners.
* Added `NoCornerCutting` neighbor filter to prevent all diagonal movement in the direction of an adjacent wall.
* Neighbors are now precomputed as bitmasks and cached when `Grid::build()` is called. This is a slight performance boost in general but a large performance boost when filtering neighbors.
* All pathfinding HPA*, A*, Djikstra uses the cached neighbors instead of calculating neighbors per neighbor call.
* Added `PathfindMode::Coarse` which fetches a heirarchichal path without refining it with the line trace algorithm.
* `AgentOfGrid` relationship has been created to relate an entity with a specific `Grid`. This will allow supporting multiple grids in the future.
* HPA* path refinement has been optimized for performance.
* Optimized collision `avoidance` system to confine collision to a local search area to prevent stutters from a full A* search.

### Debug Features
* `DebugPath` now uses the `AgentOfGrid` relationship and child relationships making it cleaner to instantiate.
* `DebugGridBuilder` has been added to make creating the `DebugGrid` more ergonomic.

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