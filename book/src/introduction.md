# Introduction

[`bevy_northstar`](https://github.com/jtothethree/bevy_northstar) is a [`Hierarchical Pathfinding`](https://alexene.dev/2019/06/02/Hierarchical-pathfinding.html) plugin for [Bevy](https://bevy.org/).

The crate provides:

* Pathfinding Grids: A grid defines the navigable area and stores precalculated neighbors, chunks, entrances, and internal paths used for pathfinding.

* Pathfinding Systems: Bevy systems to handle pathfinding and collision for you. They are not required to use HPA* or other pathfinding algorithms.

* Pathfinding Algorithms: You can call the pathfinding functions direclty if you desire to handle the pathfinding logic in your own systems or just want to do a one off call.

* Debugging Tools: Easily visualize the grid and calculated paths to troubleshoot any tilemap and pathfinding issues.

The crate is currently designed for use with 2d and 3d grid based tilemaps. It is not dependent on any specific tilemap Bevy crate, though it's been designed for ease of use with [`bevy_ecs_tilemap`](https://github.com/StarArawn/bevy_ecs_tilemap) and any related crates such as [`bevy_ecs_tiled`](https://github.com/adrien-bon/bevy_ecs_tiled) and [`bevy_ecs_ldtk`](https://github.com/Trouv/bevy_ecs_ldtk).

# Who This Crate Is For

The current target for this crate is for games with large tile based maps and designed to support large sim games like Dwarf Fortress or Rimworld and also RPGs with grid like movement like Fallout 1/2 and Roguelikes.

It does not support Flowfield pathfinding which is better suited in RTS games where many agents are often pathfinding to the single goal. HPA* is better suited for many agents with their own unique goals.

Nav meshes are not supported yet but are planned; they’re better suited for games with freeform movement, think Zelda-style movement, rather than grid-constrained paths.

Other crates that might fit those projects better:
#### Flowfield
* [bevy_pathfinding](https://crates.io/crates/bevy_pathfinding)
* [bevy_flowfield_tiles_plugin](https://crates.io/crates/bevy_flowfield_tiles_plugin)
#### NavMesh
* [vleue_navigator](https://crates.io/crates/vleue_navigator)
* [bevy_landmass](https://crates.io/crates/bevy_landmass)
#### DIY A*
* [pathfindng](https://crates.io/crates/pathfinding): The general pathfinding crate is a good place to start if you just want to implement simple A*.

# 3D: Why and When

Even if your game is technically 2D, many 2D tilemap games feature 3D movement. Isometric games are a great example, but even top-down games like Zelda often include a concept of depth. There are also fully 3D games that use grid-based movement in three dimensions, such as X-COM.

If your 2D game allows regular movement between floors or terrain levels, you’ll likely want to use a 3D grid.
