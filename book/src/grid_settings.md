# Configuring the Grid with GridSettingsBuilder

Use `GridSettingsBuild` to generate `GridSettings` to pass to the `Grid` constructor.
You will need to call `build()` to get the returned settings to pass to `Grid`.

```rust,no_run
use bevy_northstar::prelude::*;

let grid_settings = GridSettingsBuilder::new_2d(128, 128)
    .chunk_size(16)
    .default_impassable()
    .add_neighbor_filter(filter::NoCornerClipping)
    .enable_collision()
    .avoidance_distance(5)
    .build();

let grid: Grid<OrdinalNeighborhood> = Grid::new(&grid_settings);
```


# GridSettingsBuilder Configuration Breakdown
## Constructors
### `new_2d(width, height)`
Request a 2d grid. The grid always uses UVec3 in the background so this is just a helper constructor to ignore Z.
`width` and `height` are the dimensions of the grid, not pixel size.
### `new_3d(width, height, depth)`
Request a 3d grid. `depth` being the z dimension.
## Chunk Settings
### `chunk_size(size)`
`Default: 16`

The grid is divided down into regions called chunks. You can configure the size of these regions with `chunk_size(size)`. Larger chunks will reduce build time and pathfinding time, while smaller chunks will create more optimal paths.

### `chunk_depth(size)`
`Default: 1`

Ignore if your game is fully 2d. The chunk regions depth is determined separately. 1 is fine for fake 3d maps like isometric maps with a few height layers.

### `enable_diagonal_connections()`

By default the entrances to chunks only look for other chunks that are cardinally adjacent. Enabling diagonal connections will create entrances in corners where diagonally adjacent chunks are accessible.

Enabling this will increase build and pathfinding time but can result in better paths if you have a noisy map.

## Default Navigation Settings

### `default_movement_cost(cost)`
`Default: 1`

Initializes each grid cell with the set cost.

### `default_impassable()`
`Default: Passable`

Initializes each grid cell as `Nav::Impassable`. Useful if you're procedurally generating your map and can speed up setting cells by only digging out what's needed.

## Collision Settings

### `enable_collision()`
`Default: Collision disabled`

Enables the built-in pathfinding systems to use collision avoidance so entities with the `Blocking` component will never occupy the same position.

### `avoidance_distance(distance)`
`Default: 4`

The collision avoidance system will look ahead in the path to see if any upcoming path positions might be blocked and will re-route to avoid the collision. This will set how far the system looks ahead. Longer distances will affect CPU performance.

## Neighbor Settings

### `add_neighbor_filter(filter)`

Pass a `NeighborFilter` that will be applied to viable neighbors when they are precalculated. See [Neighbor Filters](./neighbor_filters.md).