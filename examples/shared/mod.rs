use bevy::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use bevy_northstar::prelude::*;

// Config used for the examples

#[derive(Resource, Debug, Default)]
pub struct Config {
    pub use_astar: bool,
    pub paused: bool,
}

// State used for the examples

#[derive(Clone, Debug, Default, Hash, Eq, States, PartialEq)]
pub enum State {
    #[default]
    Loading,
    Playing,
}

// Plugin used for the examples
pub struct SharedPlugin<N: Neighborhood + 'static> {
    _marker: std::marker::PhantomData<N>,
}

impl<N: Neighborhood + 'static> Default for SharedPlugin<N> {
    fn default() -> Self {
        SharedPlugin {
            _marker: std::marker::PhantomData,
        }
    }
}

impl<N: Neighborhood + 'static> Plugin for SharedPlugin<N> {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_hud)
            .add_systems(
                Update,
                (
                    input::<N>,
                    tick.run_if(in_state(State::Playing)),
                    under_cursor,
                    update_stat_text.run_if(in_state(State::Playing)),
                    update_pathfind_type_text.run_if(in_state(State::Playing)),
                    update_collision_text::<N>.run_if(in_state(State::Playing)),
                ),
            )
            .add_event::<Tick>()
            .insert_state(State::Loading)
            .insert_resource(Walkable::default())
            .insert_resource(Stats::default())
            .insert_resource(Config::default());
    }
}

// Walkable is used to store walkable tiles in the grid
// as a utility for spawning pathfinders.

#[derive(Resource, Debug, Default)]
pub struct Walkable {
    pub tiles: Vec<Vec3>,
}

// Timing tick
#[derive(Event, Default)]
pub struct Tick;

// Generate a tick event 4x a second, unless paused.
pub fn tick(time: Res<Time>, mut tick_writer: EventWriter<Tick>, config: Res<Config>) {
    if config.paused {
        return;
    }

    if time.elapsed_secs() % 0.25 < time.delta_secs() {
        tick_writer.write_default();
    }
}

// HUD and Text Components

#[derive(Component, Debug)]
pub struct StatText;

#[derive(Component, Debug)]
struct CollisionText;

#[derive(Component, Debug)]
pub struct PathfindTypeText;

#[derive(Component, Debug)]
pub struct EntityDebugText;

pub fn setup_hud(mut commands: Commands) {
    commands
        .spawn((
            Text::new("Key [p]| Algorithm: "),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(50.0),
                left: Val::Px(0.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            PathfindTypeText,
        ));

    commands
        .spawn((
            Text::new("Avg Path Time: "),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(100.0),
                left: Val::Px(0.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            StatText,
        ));

    commands
        .spawn((
            Text::new("Key [c]| Collision: "),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(150.0),
                left: Val::Px(0.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            CollisionText,
        ));

    commands
        .spawn((
            Text::new(""),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(200.0),
                left: Val::Px(0.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            EntityDebugText,
        ));
}

#[allow(clippy::type_complexity)]
pub fn under_cursor(
    mut query: Query<&mut TextSpan, With<EntityDebugText>>,
    windows: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform, &Transform), With<Camera>>,
    minions: Query<(Entity, &GlobalTransform)>,
    map_query: Single<(
        &TilemapGridSize,
        &TilemapSize,
        &TilemapTileSize,
        &TilemapType,
        &TilemapAnchor,
    )>,
    troubleshooting: Query<(
        Entity,
        &AgentPos,
        Option<&Pathfind>,
        Option<&Path>,
        Option<&NextPos>,
        Option<&AvoidanceFailed>,
    )>,
) {
    let window = windows.into_inner();
    let (camera, camera_transform, _) = camera.into_inner();
    let (grid_size, map_size, tile_size, tilemap_type, anchor) = map_query.into_inner();

    if let Some(cursor_position) = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok())
    {
        for mut span in &mut query {
            let mut text = String::new();
            let offset = Vec2::new(0.0, 0.0);

            let tile_pos = TilePos::from_world_pos(
                &(cursor_position + offset),
                map_size,
                grid_size,
                tile_size,
                tilemap_type,
                anchor,
            );

            // Prepend text with GridPos
            text.push_str(&format!("Tile: {:?} ", tile_pos));

            for (entity, transform) in minions.iter() {
                let distance = (transform.translation() - cursor_position.extend(0.0)).length();

                if distance < 8.0 {
                    text.push_str(&format!("{:?} ", entity));

                    // print all the data in the troubleshooting query
                    for (entity_other, position, pathfind, path, next, avoidance_failed) in
                        troubleshooting.iter()
                    {
                        if entity == entity_other {
                            text.push_str(&format!("{:?} ", position));
                            text.push_str(&format!("{:?} ", pathfind));
                            text.push_str(&format!("{:?} ", path));
                            text.push_str(&format!("{:?} ", next));
                            text.push_str(&format!("{:?} ", avoidance_failed));
                        }
                    }
                }
            }

            **span = text;
        }
    }
}

pub fn update_stat_text(stats: Res<Stats>, mut query: Query<&mut TextSpan, With<StatText>>) {
    for mut span in &mut query {
        **span = format!("{:.2}ms", stats.pathfinding.average_time * 1000.0);
    }
}

fn update_collision_text<N: Neighborhood + 'static>(
    stats: Res<Stats>,
    mut query: Query<&mut TextSpan, With<CollisionText>>,
    grid: Query<&Grid<N>>,
) {
    let grid = if let Ok(grid) = grid.single() {
        grid
    } else {
        return;
    };

    for mut span in &mut query {
        if grid.collision() {
            **span = format!("{:.2}ms", stats.collision.average_time * 1000.0);
        } else {
            **span = "Off".to_string();
        }
    }
}

pub fn update_pathfind_type_text(
    config: Res<Config>,
    mut query: Query<&mut TextSpan, With<PathfindTypeText>>,
) {
    for mut span in &mut query {
        **span = if config.use_astar {
            "A*".to_string()
        } else {
            "HPA*".to_string()
        };
    }
}

// Shared input system for all examples.
#[allow(clippy::too_many_arguments)]
pub fn input<N: Neighborhood + 'static>(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    camera: Single<(&mut Transform, &mut Projection), With<Camera>>,
    mut pathfinders: Query<(Entity, &Pathfind)>,
    mut config: ResMut<Config>,
    mut stats: ResMut<Stats>,
    grid: Single<&mut Grid<N>>,
    mut commands: Commands,
) {
    let mut grid = grid.into_inner();

    let (mut transform, mut projection) = camera.into_inner();
    if let Projection::Orthographic(ref mut ortho) = &mut *projection {
        let mut direction = Vec3::ZERO;

        if keyboard_input.pressed(KeyCode::KeyA) {
            direction -= Vec3::new(1.0, 0.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyD) {
            direction += Vec3::new(1.0, 0.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyW) {
            direction += Vec3::new(0.0, 1.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyS) {
            direction -= Vec3::new(0.0, 1.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyZ) {
            ortho.scale += 0.1;
        }

        if keyboard_input.pressed(KeyCode::KeyX) {
            ortho.scale -= 0.1;
        }

        if keyboard_input.just_pressed(KeyCode::Space) {
            config.paused = !config.paused;
        }

        if keyboard_input.just_pressed(KeyCode::KeyP) {
            config.use_astar = !config.use_astar;
            stats.reset_pathfinding();
            stats.reset_collision();

            // Remove pathfind from all pathfinders
            for (entity, _) in pathfinders.iter_mut() {
                commands
                    .entity(entity)
                    .remove::<Pathfind>()
                    .remove::<NextPos>()
                    .remove::<Path>();
            }
        }

        if keyboard_input.just_pressed(KeyCode::KeyC) {
            let current_collision = grid.collision();
            grid.set_collision(!current_collision);
            stats.reset_collision();
            // Remove pathfind from all pathfinders
            for (entity, _) in pathfinders.iter_mut() {
                commands
                    .entity(entity)
                    .remove::<Pathfind>()
                    .remove::<NextPos>()
                    .remove::<Path>();
            }
        }

        if ortho.scale < 0.3 {
            ortho.scale = 0.3;
        }

        let z = transform.translation.z;
        transform.translation += time.delta_secs() * direction * 500.;
        // Important! We need to restore the Z values when moving the camera around.
        // Bevy has a specific camera setup and this can mess with how our layers are shown.
        transform.translation.z = z;
    }
}
