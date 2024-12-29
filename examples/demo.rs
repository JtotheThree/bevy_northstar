use bevy::{
    dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin},
    prelude::*,
};

use bevy_northstar::prelude::*;

use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use rand::seq::SliceRandom;

#[derive(Resource, Debug, Default)]
pub struct Stats {
    average_time: f64,
    average_length: f64,
    pathfind_time: Vec<f64>,
    pathfind_length: Vec<f64>,
    count: u64,
}

#[derive(Resource, Debug, Default)]
pub struct Config { 
    use_astar: bool,
}

impl Stats {
    pub fn add(&mut self, time: f64, length: f64) {
        self.pathfind_time.push(time);
        self.pathfind_length.push(length);

        self.average_time = self.pathfind_time.iter().sum::<f64>() / self.pathfind_time.len() as f64;
        self.average_length = self.pathfind_length.iter().sum::<f64>() / self.pathfind_length.len() as f64;
        self.count += 1;
    }

    pub fn reset(&mut self) {
        self.average_time = 0.0;
        self.average_length = 0.0;
        self.pathfind_time.clear();
        self.pathfind_length.clear();
        self.count = 0;
    }
}

#[derive(Clone, Debug, Default, Hash, Eq, States, PartialEq)]
pub enum State {
    #[default]
    Loading,
    Playing,
}

#[derive(Resource, Debug, Default)]
pub struct Walkable {
    pub tiles: Vec<Vec3>,
}

fn main() {
    App::new()
        // Bevy default plugins
        .add_plugins(DefaultPlugins)
        .add_plugins(FpsOverlayPlugin {
            config: FpsOverlayConfig {
                text_config: TextStyle {
                    // Here we define size of our overlay
                    font_size: 32.0,
                    // We can also change color of the overlay
                    color: Color::srgb(0.0, 1.0, 0.0),
                    // If we want, we can use a custom font
                    font: default(),
                },
            },
        })
        // bevy_ecs_tilemap and bevy_ecs_tiled main plugins
        .add_plugins(TilemapPlugin)
        .add_plugins(TiledMapPlugin::default())
        // bevy_northstar plugins
        .add_plugins(NorthstarPlugin)
        .add_plugins(NorthstarDebugPlugin::<OrdinalNeighborhood> {
            config: NorthstarDebugConfig {
                grid_width: 8,
                grid_height: 8,
                draw_entrances: true,
                draw_chunks: true,
                draw_cached_paths: false,
                draw_path: true,
                draw_points: false,
                ..Default::default()
            },
            ..Default::default()
        })
        .insert_resource(Grid::<OrdinalNeighborhood>::new(&GridSettings {
            width: 128,
            height: 128,
            depth: 1,
            chunk_size: 16,
            chunk_depth: 1,
            chunk_ordinal: false,
            default_cost: 1,
            default_wall: true,
            jump_height: 1,
        }))
        // Observe the LayerCreated event to build the grid
        .observe(layer_created)
        // Add our systems and run the app!
        .add_systems(Startup, startup)
        .add_systems(OnEnter(State::Playing), spawn_minions)
        .add_systems(Update, input)
        .add_systems(Update, move_pathfinders)
        .add_systems(Update, tick.run_if(in_state(State::Playing)))
        .add_systems(Update, pathfind_minions.run_if(in_state(State::Playing)))
        .add_systems(Update, set_new_goal.run_if(in_state(State::Playing)))
        .add_systems(Update, update_stat_text.run_if(in_state(State::Playing)))
        .add_systems(Update, update_pathfind_type_test.run_if(in_state(State::Playing)))
        .add_event::<Tick>()
        .insert_state(State::Loading)
        .insert_resource(Walkable::default())
        .insert_resource(Stats::default())
        .insert_resource(Config::default())
        .run();
}

#[derive(Event, Default)]
struct Tick;

// Generate a tick event
fn tick(time: Res<Time>, mut tick_writer: EventWriter<Tick>) {
    if time.elapsed_seconds() % 0.1 < time.delta_seconds() {
        tick_writer.send_default();
    }
}

#[derive(Component, Debug)]
struct StatText;

#[derive(Component, Debug)]
struct PathfindTypeText;


fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Spawn a 2D camera (required by Bevy)
    commands.spawn(Camera2dBundle::default());

    // Load the map ...
    let map_handle: Handle<TiledMap> = asset_server.load("demo_128.tmx");

    // ... then spawn it !
    let mut map_entity = commands.spawn(TiledMapHandle(map_handle));

    // You can eventually add some extra settings to your map
    map_entity.insert((
        TiledMapSettings {
            layer_positioning: LayerPositioning::Centered,
            ..default()
        },
        TilemapRenderSettings {
            render_chunk_size: UVec2::new(32, 32),
            ..Default::default()
        },
    ));

    // Pathfinding Type Text
    let pathfinding_text_style = TextStyle {
        // Here we define size of our overlay
        font_size: 32.0,
        // We can also change color of the overlay
        color: Color::srgb(0.0, 1.0, 0.0),
        // If we want, we can use a custom font
        font: default(),
    };

    commands.spawn(NodeBundle {
        style: Style {
            // We need to make sure the overlay doesn't affect the position of other UI nodes
            position_type: PositionType::Absolute,
            bottom: Val::Px(50.0),
            left: Val::Px(0.0),
            ..default()
        },
        // Render overlay on top of everything
        z_index: ZIndex::Global(i32::MAX - 32),
        ..default()
    }).with_children(|c| {
        c.spawn((
            TextBundle::from_sections([
                TextSection::new("Key [P] Algorithm: ", pathfinding_text_style.clone()),
                TextSection::from_style(pathfinding_text_style),
            ]),
            PathfindTypeText,
        ));
    });

    // Stats Text
    let stat_text_style = TextStyle {
        // Here we define size of our overlay
        font_size: 32.0,
        // We can also change color of the overlay
        color: Color::srgb(0.0, 1.0, 0.0),
        // If we want, we can use a custom font
        font: default(),
    };

    commands
        .spawn(NodeBundle {
            style: Style {
                // We need to make sure the overlay doesn't affect the position of other UI nodes
                position_type: PositionType::Absolute,
                bottom: Val::Px(100.0),
                left: Val::Px(0.0),
                ..default()
            },
            // Render overlay on top of everything
            z_index: ZIndex::Global(i32::MAX - 32),
            ..default()
        })
        .with_children(|c| {
            c.spawn((
                TextBundle::from_sections([
                    TextSection::new("Avg Path Time: ", stat_text_style.clone()),
                    TextSection::from_style(stat_text_style),
                ]),
                StatText,
            ));
        });
}

fn update_stat_text(
    stats: Res<Stats>,
    mut query: Query<(&mut Text, &StatText)>,
) {
    for (mut text, _) in query.iter_mut() {
        text.sections[1].value = format!("{:.2}ms", stats.average_time * 1000.0);
    }
}

fn update_pathfind_type_test(
    config: Res<Config>,
    mut query: Query<(&mut Text, &PathfindTypeText)>,
) {
    for (mut text, _) in query.iter_mut() {
        text.sections[1].value = if config.use_astar {
            "A*".to_string()
        } else {
            "HPA*".to_string()
        };
    }
}

fn move_pathfinders(
    mut commands: Commands,
    grid: Res<Grid<OrdinalNeighborhood>>,
    mut minions: Query<(Entity, &mut Path, &mut Minion)>,
    mut tick_reader: EventReader<Tick>,
) {
    for _ in tick_reader.read() {
        for (entity, mut path, mut minion) in minions.iter_mut() {
            if path.is_empty() {
                continue;
            }

            let next = path.pop().unwrap();
            let next = UVec3::new(next.x as u32, next.y as u32, next.z as u32);

            let point = grid.get_point(next);
            if point.wall {
                continue;
            }

            let position = Vec3::new(next.x as f32 * 8.0 + 4.0, next.y as f32 * 8.0 + 4.0, 4.0);
            minion.position = next;

            commands.entity(entity).insert(Transform::from_translation(position));
        }
    }
}

#[derive(Component, Debug)]
struct Minion {
    position: UVec3,
}

#[derive(Component, Debug)]
struct Goal {
    position: UVec3,
}

fn pathfind_minions(
    mut commands: Commands,
    grid: Res<Grid<OrdinalNeighborhood>>,
    mut minions: Query<(Entity, &Minion, &Goal), Without<Path>>,
    mut stats: ResMut<Stats>,
    config: Res<Config>,
) {
    for (entity, minion, goal) in minions.iter_mut() {
        let start = minion.position;
        let goal = goal.position;

        let now = std::time::Instant::now();
        
        let path = if config.use_astar {
            grid.get_astar_path(start, goal)
        } else { 
            grid.get_path(start, goal)
        };
        
        let elapsed = now.elapsed().as_secs_f64();

        if path.is_none() {
            info!("No path found for {:?} from {:?} to {:?}", entity, start, goal);
            let path = if config.use_astar {
                grid.get_astar_path(start, goal)
            } else { 
                grid.get_path(start, goal)
            };
            continue;
        }

        let path = path.unwrap();

        stats.add(elapsed, path.len() as f64);

        commands.entity(entity).insert(path);
    }
}

fn set_new_goal(
    mut commands: Commands,
    mut minions: Query<(Entity, &Path, &mut Goal)>,
    walkable: Res<Walkable>,
) {
    for (entity, path, mut goal) in minions.iter_mut() {
        let new_goal = walkable.tiles.choose(&mut rand::thread_rng()).unwrap();

        if path.is_empty() {
            commands.entity(entity).remove::<Path>();

            // Set a new random goal
            goal.position = UVec3::new((new_goal.x / 8.0) as u32, (new_goal.y / 8.0) as u32, 0);
        }
    }
}

fn spawn_minions(
    mut commands: Commands,
    grid: Res<Grid<OrdinalNeighborhood>>,
    layer_entity: Query<Entity, With<TiledMapTileLayer>>,
    asset_server: Res<AssetServer>,
    mut walkable: ResMut<Walkable>,
) {
    let layer_entity = layer_entity.iter().next().unwrap();

    walkable.tiles = Vec::new();
    for x in 0..grid.get_width() {
        for y in 0..grid.get_height() {
            if grid.get_point(UVec3::new(x, y, 0)).wall == false {
                let position = Vec3::new(x as f32 * 8.0, y as f32 * 8.0, 0.0);

                walkable.tiles.push(position);
            }
        }
    }



    let mut count = 0;

    while count <= 128 {
        let position = walkable.tiles.choose(&mut rand::thread_rng()).unwrap();
        let goal = walkable.tiles.choose(&mut rand::thread_rng()).unwrap();


        let transform = Vec3::new(position.x + 4.0, position.y + 4.0, 4.0);

        commands.spawn(SpriteBundle {
            texture: asset_server.load("tile_0018.png"),
            transform: Transform::from_translation(transform),
            ..Default::default()
        }).insert(Minion {
            position: UVec3::new((position.x / 8.0) as u32, (position.y / 8.0) as u32, 0), 
        }).insert(Goal {
            position: UVec3::new((goal.x / 8.0) as u32, (goal.y / 8.0) as u32, 0),
        }).set_parent(layer_entity);

        count += 1;
    }
}

fn layer_created(
    trigger: Trigger<TiledLayerCreated>,
    q_layer: Query<&Name, With<TiledMapLayer>>,
    map_asset: Res<Assets<TiledMap>>,
    mut grid: ResMut<Grid<OrdinalNeighborhood>>,
    mut state: ResMut<NextState<State>>,
) {
    // We can either access the layer components
    if let Ok(name) = q_layer.get(trigger.event().layer) {
        info!("Received TiledLayerCreated event for layer '{}'", name);
    }

    // Or directly the underneath Tiled Layer structure
    let layer = trigger.event().layer(&map_asset);
    layer.as_tile_layer().map(|layer| {
        let width = layer.width().unwrap();
        let height = layer.height().unwrap();

        for x in 0..width {
            for y in 0..height {
                let tile = layer.get_tile(x as i32, y as i32);
                if tile.is_some() {
                    let tile_id = tile.unwrap().id();

                    if tile_id == 14 {
                        grid.set_point(UVec3::new(x as u32, height - 1 - y as u32, 0), Point::new(1, false));
                    } else {
                        grid.set_point(UVec3::new(x as u32, height - 1 - y as u32, 0), Point::new(0, true));
                    }
                }
            }
        }
    });

    info!("Loaded layer: {:?}", layer);
    grid.build();

    state.set(State::Playing);
}

pub fn input(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut Transform, &mut OrthographicProjection), With<Camera>>,
    mut config: ResMut<Config>,
    mut stats: ResMut<Stats>,
) {
    for (mut transform, mut ortho) in query.iter_mut() {
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

        if keyboard_input.just_pressed(KeyCode::KeyP) {
            config.use_astar = !config.use_astar;
            stats.reset();
        }

        if ortho.scale < 0.5 {
            ortho.scale = 0.5;
        }

        let z = transform.translation.z;
        transform.translation += time.delta_seconds() * direction * 500.;
        // Important! We need to restore the Z values when moving the camera around.
        // Bevy has a specific camera setup and this can mess with how our layers are shown.
        transform.translation.z = z;
    }
}