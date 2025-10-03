// This is a basic example for setting up the plugin.
// LEFT CLICK to pathfind to a tile.
// RIGHT CLICK to toggle the navigation state of a tile.

use bevy::prelude::*;
use bevy_northstar::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Add the Northstar Plugin with a selected neighborhood to use the built in pathfinding systems
        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
        // Add the Debug Plugin to visualize the grid and pathfinding
        // This is optional, but it helps to visualize the grid and pathfinding.
        // We use it here because we don't have a tilemap or any other visual representation.
        .add_plugins(NorthstarDebugPlugin::<CardinalNeighborhood>::default())
        .add_systems(Startup, (startup, build_grid.after(startup)))
        .add_systems(Update, (draw_player, move_player, input))
        .run();
}

fn startup(mut commands: Commands) {
    commands.spawn(Camera2d);

    // Build the grid settings.
    let grid_settings = GridSettingsBuilder::new_2d(64, 48).chunk_size(8).build();

    // Spawn the grid used for pathfinding.
    commands
        .spawn(CardinalGrid::new(&grid_settings))
        // Spawning the debug grid as a child of the grid entity.
        .with_child((
            DebugGridBuilder::new(12, 12) // 12x12 tile pixel size.
                .enable_cells() // Draws every cell in the grid. We're using it to draw "tiles".
                .build(),
            // Offset the debug grid to the center of the world.
            DebugOffset(Vec3::new(-384.0, -288.0, 0.0)),
        ));

    // Let's position the player on the map first.
    let player_transform = Transform::from_translation(Vec3::new(
        4.0 * 12.0 - 384.0, // Align with the grid cell size and offset.
        4.0 * 12.0 - 288.0,
        0.0,
    ));

    // Let's spawn a player entity that will be used to demonstrate pathfinding.
    commands.spawn((
        Name::new("Player"),
        AgentPos(UVec3::new(4, 4, 0)), // Starting position in the grid.
        player_transform,
    ));
}

fn build_grid(grid: Single<&mut CardinalGrid>) {
    let mut grid = grid.into_inner();

    // Let's set every 3rd cell in the grid to be impassable.
    // In a real game you would set the grid to match your tilemap or level design.
    // Then iterate over your tiles and set the navigation data accordingly.
    for x in 0..grid.width() {
        for y in 0..grid.height() {
            // Create some staggered impassable cells.
            if x % 2 == 0 && y % 3 == 0 {
                grid.set_nav(UVec3::new(x, y, 0), Nav::Impassable);
            }
        }
    }

    info!("Building the grid...");

    // The grid needs to be built after setting the grid cell nav data.
    // Building the grid will calculate the chunk entrances and cache internal paths.
    grid.build();

    info!("Grid built successfully!");
}

fn draw_player(query: Query<&Transform, With<AgentPos>>, mut gizmos: Gizmos) {
    for transform in &query {
        // Draw a simple circle at the agent's position.
        gizmos.circle_2d(
            Vec2::new(transform.translation.x, transform.translation.y),
            4.0,                             // Radius of the circle.
            Color::srgba_u8(0, 255, 0, 255), // Color of the circle.
        );
    }
}

fn move_player(
    mut query: Query<(Entity, &mut AgentPos, &NextPos, &mut Transform)>,
    mut commands: Commands,
) {
    let offset = Vec3::new(-384.0, -288.0, 0.0); // Offset to center on the world.

    for (entity, mut agent_pos, next_pos, mut transform) in &mut query {
        // Set the transform position to the agent's position in the grid.
        transform.translation = Vec3::new(
            next_pos.0.x as f32 * 12.0 + offset.x, // Align with the grid cell size.
            next_pos.0.y as f32 * 12.0 + offset.y,
            0.0,
        );

        // Update the agent's position to the next position.
        agent_pos.0 = next_pos.0;

        // Now we remove the NextPos component from the player to consume it.
        // This is important to get the next updated position in the path.
        commands.entity(entity).remove::<NextPos>();
    }
}

fn input(
    input: Res<ButtonInput<MouseButton>>,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform, &Transform), With<Camera>>,
    player: Single<Entity, With<AgentPos>>,
    grid: Single<&mut CardinalGrid>,
    mut commands: Commands,
) {
    let window = window.into_inner();
    let (camera, camera_transform, _) = camera.into_inner();
    let player = player.into_inner();

    let clicked_tile = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok())
        .map(|cursor_position| {
            let offset = Vec2::new(-384.0, -288.0);
            let cursor_position = cursor_position - offset;
            UVec3::new(
                (cursor_position.x / 12.0).round() as u32,
                (cursor_position.y / 12.0).round() as u32,
                0,
            )
        });
    // Most of this isn't important for using the crate and is standard Bevy usage.
    // We just want to demonstrate how to use the pathfinding system with a mouse click.
    if input.just_pressed(MouseButton::Left) {
        if let Some(goal) = clicked_tile {
            // This is the important bit here.
            // We insert a Pathfind component with the goal position.
            // The pathfinding system will insert a NextPos component
            // on the next frame.
            commands
                .entity(player)
                .insert(Pathfind::new(goal).partial());
        }
    }

    // Right click to toggle the navigation state of the clicked tile.
    // This demonstrates how to dynamically change the grid's navigation data.
    if input.just_pressed(MouseButton::Right) {
        if let Some(position) = clicked_tile {
            let mut grid = grid.into_inner();

            if let Some(nav) = grid.nav(position) {
                if !matches!(nav, Nav::Impassable) {
                    // If the cell is passable, we set it to impassable.
                    grid.set_nav(position, Nav::Impassable);
                } else {
                    // If the cell is impassable, we set it to passable with a cost of 1.
                    grid.set_nav(position, Nav::Passable(1));
                }
            } else {
                return;
            }
            // You must call `build` after modifying the grid to update the internal state.
            grid.build();
        }
    }
}
