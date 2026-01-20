# bevy_northstar
[![Crates.io](https://img.shields.io/crates/v/bevy_northstar)](https://crates.io/crates/bevy_northstar)
[![docs](https://docs.rs/bevy_northstar/badge.svg)](https://docs.rs/bevy_northstar/)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/jtothethree/bevy_northstar/blob/main/LICENSE)
[![Crates.io](https://img.shields.io/crates/d/bevy_northstar)](https://crates.io/crates/bevy_northstar)
[![Following released Bevy versions](https://img.shields.io/badge/Bevy%20tracking-released%20version-lightblue)](https://bevyengine.org/learn/quick-start/plugin-development/#main-branch-tracking)

## A 2d/3d hierarchical pathfinding crate for Bevy. 

`bevy_northstar` works by dividing the map into chunks and then calculates nodes based on the entrances between chunks. The nodes are used in pathfinding to get a higher level path that is significantly faster to calculate over long distances. Once the high level path is determined between a start and goal position it can be refined to get a more accurate path. The crate is designed to significantly reduce the performance hit of A* pathfinding on complex maps.

The crate provides:

## Features
- **Pathfinding Options** - Choose between optimized HPA* algorithms or traditional A* per call. Includes Any-Angle HPA* and Theta* for continuous movement games.
- **Supports 2D, 2.5D, and 3D Pathfinding** - Handles top-down, isometric, and layered maps with support for vertical transitions. Free 3D movement is also supported.
- **Neighbor Filtering and Caching** - Precomputed, optionally filtered neighbors are cached to avoid redundant processing.
- **Dynamic Changes** - Modify your tilemap and only affected areas will be recalculated during grid rebuilds.
- **Navigation Masks** – Adjust tile costs or impassability per agent on each pathfinding request.
- **Parallel Building** - Chunk calculations run in parallel for faster builds. Can be disabled to support WASM.
- **Memory Efficient** - Neighbors are stored in compact bitmasks, reduced memory on large maps.
- **Gizmo Debug View** – Visualize the HPA* grid and entity paths using debug components.
- **Dynamic Collision & Avoidance** – Optional collision avoidance system. Just the add the `Blocking` component to flag blockers.
- **Bevy Integration** – Systems and components for pathfinding and collision avoidance. Pathfindng systems are able to stagger agents across multiple frames.

![Isometric Example](https://github.com/JtotheThree/bevy_northstar/raw/main/assets/images/isometric_demo.gif)

## Demo
`cargo run --example demo --features stats --release`

Press P to switch between Refined HPA*, Coarse HPA*, and traditional A*
Press C to disable/enable collision
Press R to change the nav data for random tiles every 5 seconds. 

![Demo](https://github.com/JtotheThree/bevy_northstar/raw/main/assets/images/demo.png)


## Documentation

* API Reference on [`docs.rs`](https://docs.rs/bevy_northstar/latest/bevy_northstar/)
* Usage guide and explanations in the [`book`](https://jtothethree.github.io/bevy_northstar/)
* Examples in the repository

## Feature Flags
This crate has the following Cargo features:

- `stats`: Enables pathfinding benchmarks. Useful to get an idea of how much time it's using per frame.
- `parallel`: Enabled by default. Disable default features to run grid builds single-threaded if needed for WASM.

# Quick Start

Add required dependencies to your `Cargo.toml` file:

```toml
[dependencies]
bevy = "0.17"
bevy_northstar = "0.5"
```

The basic requirements to use the crate are to spawn an entity with a `Grid` component, adjust the grid cells, and then call `Grid::build()` so the neighbors, chunk entrances, and internal paths can be calculated. 

To use the built-in pathfinding systems for the crate, insert the NorthstarPlugin specifying the `Neighborhood` to use.

The built-in neighborhoods are:
* `CardinalNeighborhood` 4 directions allowing no diagonal movement.
* `CardinalNeighborhood3d` 6 directions, including up and down, allowing no diagonal movement.
* `CardinalIsoNeighborhood` 8 directions, similar to Cardinal 3d, except allows vertical diagonal movement.
* `OrdinalNeighborhood` 8 directions allowing for diagonal movement.
* `OrdinalNeighborhood3d` 26 directions which includes the base ordinal movements and their up/down directions.

```rust,no_run
use bevy::prelude::*;
use bevy_northstar::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Add the Northstar Plugin with a selected neighborhood to use the built in pathfinding systems
        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
        .add_systems(Startup, (startup, build_grid.after(startup)))
        .run();
}

fn startup(mut commands: Commands) {
    commands.spawn(Camera2d::default());

    // Build the grid settings.
    let grid_settings = GridSettingsBuilder::new_2d(16, 16)
        .chunk_size(4)
        .build();

    // Spawn the grid component
    commands.spawn(CardinalGrid::new(&grid_settings));
}

fn build_grid(grid: Single<&mut CardinalGrid>) {
    let mut grid = grid.into_inner();

    // Let's set the position 8, 8 to a wall
    grid.set_nav(UVec3::new(8, 8, 0), Nav::Impassable);

    info!("Building the grid...");

    // The grid needs to be built after setting the cells nav data.
    // Building the grid will calculate the chunk entrances and cache internal paths.
    grid.build();

    info!("Grid built successfully!");
}
```

## Debug Optimization
Pathfinding and grid algorithms involve a lot of branching, which can make debug builds significantly slower. You can set the optimization settings for this crate so you can still debug your game without the performance hit.

Follow [Bevy Quickstart Cargo Workspaces](https://bevy.org/learn/quick-start/getting-started/setup/#cargo-workspaces) to add `opt-level = 3` to your `Cargo.toml` dependencies.

Or alternatively add the following to your `Cargo.toml`:
```toml
[profile.dev.package."bevy_northstar"]
opt-level = 3
```

## Bevy Compatibility

|bevy|bevy_northstar|
|---|---|
|0.18|0.6|
|0.17|0.5|
|0.16|0.2/0.3/0.4|


## Roadmap / TODO
- **Add Refinement Filters** - Improve path quality with additional heuristics and optional filters to smooth jagged paths.
- **Add Support For Multiple Grids & HPA Levels** – Implement multiple hierarchical levels for improved efficiency.

## Assets credits
- [kenny-minimap-pack](https://kenney.nl/assets/minimap-pack): an 8x8 tileset from [Kenney](https://kenney.nl/), licensed under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)


## Thanks
Thanks to the following crates and blogs that have been used as references
* <https://github.com/evenfurther/pathfinding>
* <https://github.com/mich101mich/hierarchical_pathfinding>
* <https://alexmelenchon.github.io/Hierarchial-Pathfinding-Research/>
