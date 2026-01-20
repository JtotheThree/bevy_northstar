# Quick Start

## Cargo Setup

Add required dependencies to your `Cargo.toml` file:

```toml
[dependencies]
bevy = "0.18"
bevy_northstar = "0.6"
```

### Features

#### `Stats`
Enable statistics reporting, includes pathfinding, collision avoidance, and grid build time stats. Useful for diagnosing frame spikes. See [Debugging](./debugging.md).

#### `Parallel` *(Enabled by default)*
Uses `rayon` to build the navigation grid in parallel. This greatly speeds up grid rebuilds, but may cause issues in WASM builds.
To disable parallelism (e.g. for WASM support), disable default features:
```toml
bevy_northstar = { version = "0.5", default-features = false }
```
> **⚠️ Warning**  
> If you disable `parallel`, rebuilding the grid during gameplay (e.g. mining, explosions, etc.) can be quite expensive. You will want to confine your updates in a single frame to the least amount of affected chunks.

### Debug Optimization
Pathfinding and grid algorithms involve a lot of branching, which can make debug builds significantly slower. You can set the optimization settings for this crate so you can still debug your game without the performance hit.

Follow [Bevy Quickstart Cargo Workspaces](https://bevy.org/learn/quick-start/getting-started/setup/#cargo-workspaces) to add `opt-level = 3` to your `Cargo.toml` dependencies.

Or alternatively add the following to your `Cargo.toml`:
```toml
[profile.dev.package."bevy_northstar"]
opt-level = 3
```

## Crate Usage

The basic requirements to use the crate are to spawn an entity with a `Grid` component, adjust the navigation data, and then call `Grid::build()` so the chunk entrances and internal paths can be calculated. 

To use the built-in pathfinding systems for the crate, add the `NorthstarPlugin` specifying the `Neighborhood` to use.

`CardinalNeighborhood` (North, East, South, West) is a good neighborhood to start with. See [Neighborhoods](./neighborhood/01_neighborhoods.md) for the full list and details.

```rust,no_run
use bevy::prelude::*;
use bevy_northstar::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Add the Northstar Plugin with a selected neighborhood to use the built-in pathfinding systems
        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
        .add_systems(Startup, (startup, build_grid.after(startup)))
        .run();
}

fn startup(mut commands: Commands) {
    // Configure the grid
    let grid_settings = GridSettingsBuilder::new_2d(64, 48).chunk_size(16).build();

    // Spawn the grid used for pathfinding.
    commands.spawn(Grid::<CardinalNeighborhood>::new(&grid_settings));
}

fn build_grid(grid: Single<&mut Grid<CardinalNeighborhood>>) {
    let mut grid = grid.into_inner();

    // Let's set the position 8, 8 to a wall
    grid.set_nav(UVec3::new(8, 8, 0), Nav::Impassable);

    // The default settings set every position as passable but for demonstration let's set one
    // Nav::Passable takes a movement cost which determines how expensive it is to move to that position.
    grid.set_nav(UVec3::new(4, 4, 0), Nav::Passable(1));

    info!("Building the grid...");

    // The grid needs to be built after setting the points.
    // Building the grid will calculate the chunk entrances and cache internal paths.
    grid.build();

    info!("Grid built successfully!");
}
```

## Grid Generic Neighborhood Shorthand Types
The following shorthand types are also available for constructing and referencing a `Grid::<N>`.

* CardinalGrid
* CardinalGrid3d
* OrdinalGrid
* OrdinalGrid3d

Rather than `Grid<CardinalNeighborhood>::new` you can use `CardinalGrid::new`.
Rather than querying `Single<&mut Grid<CardinalNeighborhood>>` you can query `Single<&mut CardinalGrid>`

## Quick NorthstarPlugin Pathfinding System Usage
Using the example above we can create a system to look for entities without a path and give them a goal.

If you're not interested in using the built-in systems, see [Pathfinding](./pathfinding.md) for examples.

```rust,no_run
use bevy::prelude::*;
use bevy_northstar::prelude::*;

fn spawn(mut commands: Commands) {
    let grid_settings = GridSettingsBuilder::new_2d(64, 48).chunk_size(16).build();
    // Store the spawned entity to relate entities to this grid.
    let grid_entity = commands.spawn(CardinalGrid::new(&grid_settings));

    // Let's spawn a pathfinding Player agent
    commands.spawn((
        Name::new("Player"),
        // AgentPos is required to position your entity in the grid.
        AgentPos(UVec3::new(4, 4, 0)),
        // Here we relate this agent to the grid we created.
        AgentOfGrid(grid_entity),
    ));
}

fn pathfind_agents(
    // The Pathfind component acts as a request to pathfind to a goal.
    query: Query<Entity, Without<Pathfind>>,
    mut commands: Commands,
) {
    for entity in &query {
        // Let's request to pathfind to 7,7.
        commands.entity(entity).insert(Pathfind::new(UVec3::new(7, 7, 0)));
    }
}

fn move_player(
    mut query: Query<(Entity, &mut AgentPos, &NextPos)>
    mut commands: Commands,
) {
    for (entity, mut agent_pos, next_pos) in &mut query {
        // NextPos contains the next valid position in the path.
        // Here we update just the AgentPos to keep it aligned with the grid,
        // but for real usage you would also likely update the transform for world positioning.
        agent_pos.0 = next_pos.0;

        // Remove the NextPos component and the pathfinding system will insert a new NextPos with the next position in the path.
        commands.entity(entity).remove::<NextPos>();
        // Once the agent reaches its goal, Pathfind will be removed.
    }
}
```

## Grid as a Component
Currently the plugin Pathfinding and Debug systems expect there to be only a single copy of the Grid component which means you can't currently use them if you want to support multiple grids in your project. 

Normally it would make sense for this to be Bevy `Resource` but this decision was made so the plugin can update to support multiple grids in the future without making breaking API changes. If your project needs to support multiple pathfinding grids you can avoid using the NorthstarPlugin and NorthstarDebugPlugin and call the pathfinding functions directly on the `Grid` components for the time being.
