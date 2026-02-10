# Navigation Masks
Navigation masks are an advanced feature let you change the grid cell cost and impassability per agent on a per-call basis.

> ⚠️ Warning: Navigation Masks do not work well with the Theta* pathfinding algorithm.
> Due to the nature of the Theta* it would be astronomically expensive to handle custom masks.

## Navigation Mask Layers
A `NavMaskLayer` defines a layer of navigation data masks. Layers are then combined in `NavMask`. This allows you to represent special conditions (e.g., water tiles, faction boundaries) and assign them to individual agents through masks.

Most `NavMaskLayer` methods require a reference to a `Grid` to define bounds for caching data.

### `NavMaskLayer::insert_mask(grid, pos, mask)`
Set a mask at a single cell.

### `NavMaskLayer::insert_region_fill(grid, region, mask)`
Fill an entire region with the same mask.

### `NavMaskLayer::insert_region_outline(grid, region, mask)`
Apply a mask to the outline of a region. Useful for constraining agents with `NavCellMask::ImpassableOverride`.

### `NavMaskLayer::insert_hashmap(grid, masks<UVec3, NavCellMask>)`
Populate the layer from a `HashMap` of positions and masks.

### `NavMaskLayer::insert_hashset(grid, masks<UVec3>, mask)`
Apply the same mask to all positions in the set.

### `NavMaskLayer::clear()`
Remove all data from the layer including the cache.

See [NavCellMask](#navcellmask) for mask options.

> ⚠️ Arc Result
> `NavMaskLayer` is an `Arc<Mutex>` wrapper so it can be shared in Bevy systems.  
> All methods return a `Result`. Errors are returned as `String` if the data lock fails.

### Example: Creating a navigation mask layer

```rust,no_run
let layer = NavMaskLayer::new();
layer.insert_region_fill(
    &grid,
    NavRegion::new(UVec3::new(0, 0, 0), UVec3::new(10, 10, 10)),
    NavCellMask::ModifyCost(5),
).ok();
```

## Navigation Mask
A `NavMask` combines one or more layers and is used during pathfinding or passed in for manual pathfinding calls.
Masks can be shared across agents or created per agent.

Layers are processed in order added with the last layer modifying last.

> If a cell is impassable in the grid or in any layer with `NavCellMask::ImpassableOverride`, the cell is regarded as impassable. Because neighbors are precomputed on the grid it is not currently possible to take an impassable grid cell and make it passable. If such behavior is desired, one strategy would be to make it passable on the grid and apply an impassable mask cell for your agents.

`NavMask` also includes a cache for mask HPA* lookups. See [Performance Notes](#performance-notes).

* `NavMask::add_layer(NavMaskLayer)`
Adds a layer on top of the `NavMask`. This clears the cache in `NavMask`.

* `NavMask::with_additional_layer(NavMaskLayer)`
Create a copy of a `NavMask` adding an additional `NavMaskLayer` on top. Useful if you have a common `NavMask` for your agents and just need to add a layer for a single agent.

* `NavMask::flatten()`
Merges all of the layers in the mask to a single layer. If you start running into performance issues with masks with a large number of layers this should be useful.

* `NavMask::clear()`
Clears all the layers and cached data from the mask.


> ⚠️ Arc Result
> `NavMask` is an `Arc<Mutex>` wrapper.
> All methods return a `Result`. Errors are returned as `String` if the data lock fails.

### Example: Creating a NavMask

```rust,no_run
let layer = NavMaskLayer::new();
layer.insert_region_fill(
    &grid,
    NavRegion::new(UVec3::new(0, 0, 0), UVec3::new(10, 10, 10)),
    NavCellMask::ModifyCost(5),
).ok();

let mask = NavMask::new();
mask.add_layer(layer).ok();
```

## NavCellMask
`NavCellMask` is an enum  has two options you can use to layer navigation data.
Currently it supports:

* `NavCellMask::ImpassableOverride`: Will set the cell to impassable. Setting this for a cell on any layer will make the cell impassable no matter what any of the other layers are set to.

* `NavCellMask::ModifyCost(i32)`: The i32 past can be used to add or subtract to the overall cost of the cell. For example if you wanted water cells to cost more for infantry but not vehicles you could create a mask for the infanty units that uses ModifyCost(5) for example.


## Using Masks in Pathfinding
Use the `AgentMask` component to associate a mask with a specific entity when using the plugin systems. 
Because `NavMask` is an `Arc` wrapper, it can be cloned without duplicating the data.

Using the Plugin Pathfinding systems:

```rust,no_run
let mask = NavMask::new();

commands
    .entity(player)
    .insert((
        Pathfind::new(UVec3::new(8, 8, 0),
        AgentMask(mask.clone()),
    ));
```

In direct manual pathfinding calls you can pass the mask with `PathfindArgs::mask`:

```rust,no_run
let grid_settings = GridSettingsBuilder::new_2d(16, 16).build();
let grid = CardinalGrid::new(&grid_settings);

let mask = NavMask::new();

let path = grid.pathfind(
    PathfindArgs::new(UVec3::new(0, 0, 0), UVec3::new(7, 7, 0))
        .mask(mask.clone())
);
```

## Debugging NavMask
You can debug a NavMask using `DebugGrid`. See [Debugging](./debugging.md) for more information on setting up `DebugGrid`.

Example 
```rust, no_run
#[derive(Resource)]
struct MyNavMasks(HashMap<String, NavMask>);

fn apply_debug_mask(debug_grid: Single<&mut DebugGrid>, masks: Res<MyNavMasks>) {
    let mut debug_grid = debug_grid.into_inner();

    let mask = masks.0.get("example_mask").unwrap();
    debug_grid.set_debug_mask(mask.clone());
    // In order to view the debugged navigation masks, you'll need to enable debug cell gizmos
    debug_grid.set_draw_cells(true);

    // You can later clear the debug mask with:
    debug_grid.clear_debug_mask();
}
```

## Performance Notes
Adding a navigation mask to a pathfinding call will decrease the performance of the pathfinding. A navigation mask used in HPA* (Refined, Waypoint) pathfinding will cache any look ups that are done for future use. After a certain usage in HPA* the cost will greatly decrease.

Every change to the mask will currently clear the cache. You may want to avoid updating masks too frequently if you find you're running into performance issues.