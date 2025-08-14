# Pathfinding With Built-In Systems

## Pathfind and AgentPos Components

To use the `NorthstarPlugin` pathfinding systems, insert a `Pathfind` and `AgentPos` on the entity you want to pathfind.
You will need to maintain the grid position in `AgentPos`.

```rust,no_run
commands
    .spawn((
        Name::new("Player"),
        // Request a path to 8,8
        Pathfind::new(UVec3::new(8, 8, 0)),
        // The entities current position in the grid
        AgentPos(UVec3::new(4, 4, 0))
        Blocking, // Insert the Blocking component if using collision and this entity should block others.
    ));
```

There are also shorthand constructors for creating `Pathfind`.

```rust,no_run
// If you're on a 2d grid you can use `new_2d()` without having to create a new UVec3. 
Pathfind::new_2d(8, 8)
// Same with 3d
Pathfind::new_3d(8, 8, 4)
```
### Pathfind Configuration
Pathfind has configuration options you can set by chaining.

#### `mode(mode)`
`Default: PathfindMode::Refined` 

Use this to set the desired algorithm to find the goal. Ex: `Pathfind::new_2d(8, 8).mode(PathfindMode::AStar)`.
See below for a list of `PathfindMode`s and their description.

#### `partial()`
`Default: Not enabled`

Apply `.partial()` to request an incomplete path if the goal is not reachable. Ex: `Pathfind::new_2d(4, 4).mode(PathfindMode::Astar).partial()`.

## NextPos
The pathfind system detects entities with a changed `Pathfind` component. It then runs the pathfinding algorithm and, if a valid path is found, inserts the next step as a `NextPos` component.

You should consume the `NextPos` component by moving the entity accordingly and then removing the component afterward. In a subsequent frame, the next_position system will insert the next `NextPos` if the path is not yet complete.

If collision avoidance is enabled, the next_position system will also handle local avoidance. It may adjust the path if another entity is blocking the current path within the configured avoidance_distance (as set in `GridSettingsBuilder`).

See [Grid Settings](./grid_settings.md) for more information on enabling and configuring collision.

Example movement system:
```rust,no_run
fn movement(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AgentPos, &NextPos)>,
) {
    for (entity, mut agent_pos, next_pos) in &mut query {
        // Set the entities GridPos to the NextPos UVec3.
        agent_pos.0 = next_pos.0;

        // Update the entities translation
        let translation = Vec3::new(
            next.0.x as f32 * 32.0, // Assuming tiles are 32x32
            next.0.y as f32 * 32.0,
            0.0
        );

        commands.entity(entity)
            .insert(Transform::from_translation(translation))
            .remove::<NextPos>();
    }
}
```

## Pathfinding/Collision Marker Components

### `PathfindingFailed`
This component is inserted into an entity if a path to the desired goal cannot be found.
You will want to create a system that determines how to handle the failure in a way unique to your game.

### `AvoidanceFailed` 
This component is inserted when collision avoidance is enabled and the entity cannot find a path around a local `Blocking` entity.

The `reroute_path` system will automatically attempt to compute a new full HPA* path to resolve the issue in the next frame. You may also choose to handle this yourself in a custom system.

### `RerouteFailed` 
This component is added when all attempts to resolve a collision-related pathing issue have failed, meaning no viable path to the goal exists at the moment or the entity is stuck.

You must handle this case in your own system — for example, by:
- Selecting a new goal
- Waiting and retrying after a delay
- Alerting the player/user


```rust,no_run
fn handle_pathfind_failed(
    mut commands: Commands,
    mut query: Query<(Entity, &Name, &Pathfind), With<PathfindingFailed>>,
) {
    for (entity, name, path) in &mut query {
        log::warn!("{} cannot find a route to {}!", name, path.goal);

        let new_goal = locate_new_cheese();

        commands
            .entity(entity)
            .insert(Pathfind::new(new_goal))
            .remove::<PathfindingFailed>();
    }
}

fn handle_reroute_failed(
    mut commands: Commands,
    mut query: Query<Entity, With<RerouteFailed>>,
) {
    for entity in &mut query {
        let some_new_goal = UVec3::new(3, 30, 0); // Just some random new goal

        commands
            .entity(entity)
            .insert(Pathfind::new(some_new_goal))
            .remove::<RerouteFailed>();
    }
}
```

## PathingSet

The `NorthstarPlugin` pathfinding systems run in their own system set named `PathingSet`.

You can use the set to ensure that your systems dealing with pathfinding and entity movement happen before or after the pathing systems.

```rust,no_run
app.add_systems(Update, move_pathfinders.before(PathingSet));
```

## Staggering Pathfinding in the `NorthstarPlugin` Systems

The systems provided by `NorthstarPlugin` are designed to stagger how many agents can process pathfinding and collision avoidance in a single frame.

By default, both limits are set to `128` agents per frame. This may be too high if you actually have that many active agents. On the other hand, setting it too low can cause noticeable delays, with agents appearing to sit idle. You’ll want to tune these values based on your game’s performance needs.

Currently the `reroute_path` system that attempts to reroute agents that have a failed path that local collision avoidance is unable to resolve can still cause stutter. In a future update it will be moved into any async call.

To override the default settings, insert the `NorthstarPluginSettings` resource into your app:

```rust,no-run
App::new()
    .insert_resource(NorthstarPluginSettings {
        max_pathfinding_agents_per_frame: 16,
        max_collision_avoidance_agents_per_frame: 16,
    })
```