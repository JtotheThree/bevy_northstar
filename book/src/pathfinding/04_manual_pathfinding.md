# Manual Pathfinding

You don't need to use the pathfinding systems in the `NorthstarPlugin` in order to take advantage of this crate. 

You can use both, or choose to not add the `NorthstarPlugin` and call the pathfinding functions completely manually.

If you don't use `NorthstarPlugin` you'll need to maintain your own `BlockingMap` or `HashMap<UVec3, Entity>` to pass to the `pathfind` function to provide it a list of blocked positions.

All of the pathfinding calls can be done on the `Grid` component.

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

    let path = grid.pathfind(grid_pos.0, move_action.0, blocking, false);

    // Setting use_partial to true will allow the pathfinding to return a partial path if a complete path isn't found.

    // If you're not using collision you can pass an empty hashmap for the blocking map.
    let path = grid.pathfind(grid_pos.0, move_action.0, HashMap::new(), true);

    // There are also Coarse and AStar methods
    let path = grid.pathfind_coarse(grid_pos.0, move_action.0, blocking, false);
    let path = grid.pathfind_astar(grid_pos.0, move_action.0, blocking, false);
}
```

The `Grid` pathfinding methods return an `Option<Path>`. `None` will be returned if no viable path is found.
