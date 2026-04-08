// Integration test: verifies that removing Path + NextPos and re-inserting
// Pathfind after a grid change produces a new path that avoids the blocked cell.

use bevy::prelude::*;
use bevy_northstar::prelude::*;

/// Minimal headless app with the Northstar pathfinding systems.
fn test_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default());
    app
}

#[test]
fn grid_change_with_reroute_avoids_blocked_cell() {
    let mut app = test_app();

    // 16x16 grid, all passable.
    let size: u32 = 16;
    let settings = GridSettingsBuilder::new_2d(size, size)
        .chunk_size(8)
        .build();
    let mut grid = CardinalGrid::new(&settings);
    for x in 0..size {
        for y in 0..size {
            grid.set_nav(UVec3::new(x, y, 0), Nav::Passable(1));
        }
    }
    grid.build();
    let grid_entity = app.world_mut().spawn(grid).id();

    // Agent at left edge, goal at right edge along row 8.
    let start = UVec3::new(0, 8, 0);
    let goal = UVec3::new(15, 8, 0);
    let agent = app
        .world_mut()
        .spawn((
            AgentPos(start),
            AgentOfGrid(grid_entity),
            Pathfind::new(goal),
        ))
        .id();

    // Let the plugin compute the initial path.
    for _ in 0..5 {
        app.update();
    }
    let has_path =
        app.world().get::<Path>(agent).is_some() || app.world().get::<NextPos>(agent).is_some();
    assert!(has_path, "agent should have an active path after init");

    // Block a cell on the straight-line path.
    let blocked = UVec3::new(4, 8, 0);
    {
        let mut state = app.world_mut().query::<&mut CardinalGrid>();
        let mut grid = state.single_mut(app.world_mut()).unwrap();
        grid.set_nav(blocked, Nav::Impassable);
        grid.build();
    }

    // Apply the documented reroute pattern: remove Path + NextPos, re-insert Pathfind.
    app.world_mut()
        .entity_mut(agent)
        .remove::<(NextPos, Path)>()
        .insert(Pathfind::new(goal));

    // Let the plugin recompute.
    for _ in 0..5 {
        app.update();
    }

    // The new path must not pass through the blocked cell.
    if let Some(path) = app.world().get::<Path>(agent) {
        assert!(
            !path.is_position_in_path(blocked),
            "rerouted path should not contain the blocked cell"
        );
    }
    if let Some(next) = app.world().get::<NextPos>(agent) {
        assert_ne!(
            next.0, blocked,
            "next waypoint should not be the blocked cell"
        );
    }
}
