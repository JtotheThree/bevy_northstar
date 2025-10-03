# Manual Pathfinding

You don't need to use the pathfinding systems in the `NorthstarPlugin` in order to take advantage of this crate. 

You can use both, or choose to not add the `NorthstarPlugin` and call the pathfinding functions completely manually.

If you don't use `NorthstarPlugin` you'll need to maintain your own `BlockingMap` or `HashMap<UVec3, Entity>` to pass to the `pathfind` function to provide it a list of blocked positions.

All pathfinding can be done calling the `pathfind` method on the the `Grid` component.

```rust,no_run
fn manual_pathfind(
    mut commands: Commands,
    player: Single<(Entity, &AgentPos, &MoveAction), With<Player>>,
    grid: Single<&CardinalGrid>,
    // If using the plugin you can use the BlockingMap resource for an auto-updated blocking list.
    blocking: Res<BlockingMap>,
) {
    let grid = grid.into_inner();
    let (player, grid_pos, move_action) = player.into_inner();

    let path = grid.pathfind(
        PathfindArgs::new(grid_pos.0, move_action.0)
    );

    info!("Path {:?}", path);
}
```

The `Grid` pathfinding methods return an `Option<Path>`. `None` will be returned if no viable path is found.

## PathfindArgs

`Grid::pathfind` takes `PathfindArgs` which works just like the `Pathfind` component to build the arguments for pathfinding.

```rust,no_run
let args = PathfindArgs::new(start, goal).astar().max_distance(50);
```

#### `new(start: UVec3, goal: UVec3) -> Self`
Construct `PathfindArgs` with the starting position and the goal position.

### Modes (Algorithms)

* `refined()` (default)
Uses HPA* and then refines the path with line tracing. Produces smooth, efficient paths.

* `coarse()`
Uses cached HPA* cluster data only. Extremely fast, but paths may be less accurate.

* `waypoints()`
Returns only the waypoints needed to navigate around obstacles. Useful for continuous movement systems (e.g., steering behaviors).

* `astar()`
Runs a standard grid-based A* search.

* `thetastar()`
Runs the Theta* any-angle algorithm. Produces fluid, natural-looking paths by cutting corners where possible.

Or, set directly with:

```
args.mode(PathfindMode::AStar);
```

### Search Limits
You can constrain how far or where the algorithm searches:

* `partial()`
If the goal cannot be reached, returns the closest path instead of failing.
Note that this doesn't work with the default refined (HPA*) or waypoints algorithm as HPA* doesn't handle partial pathing well. You'll want to use this only with AStar or ThetaStar.

* `max_distance(n: u32)`
Stops searching after the set distance from the start. No path is returned if the goal is farther away. 

* `search_region(region: NavRegion)`
Restricts the search to a given region. See [NavRegion](https://docs.rs/bevy_northstar/latest/bevy_northstar/struct.NavRegion.html).