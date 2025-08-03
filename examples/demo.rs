use bevy::{
    dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin},
    platform::collections::HashMap,
    prelude::*,
    text::FontSmoothing,
};

use bevy_northstar::prelude::*;

use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use rand::seq::IndexedRandom;

mod shared;

fn main() {
    App::new()
        // Bevy default plugins
        .add_plugins(DefaultPlugins)
        // FPS Overlay
        .add_plugins(FpsOverlayPlugin {
            config: FpsOverlayConfig {
                text_config: TextFont {
                    font_size: 32.0,
                    font_smoothing: FontSmoothing::default(),
                    font: default(),
                    ..default()
                },
                text_color: Color::srgb(0.0, 1.0, 0.0),
                enabled: true,
                ..default()
            },
        })
        // bevy_ecs_tilemap and bevy_ecs_tiled main plugins
        .add_plugins((TilemapPlugin, TiledMapPlugin::default()))
        // bevy_northstar plugins
        .add_plugins((
            NorthstarPlugin::<OrdinalNeighborhood>::default(),
            NorthstarDebugPlugin::<OrdinalNeighborhood>::default(),
        ))
        // Add the SharedPlugin for unrelated pathfinding systems shared by the examples
        .add_plugins(shared::SharedPlugin::<OrdinalNeighborhood>::default())
        // Observe the LayerCreated event to build the grid from the Tiled layer
        .add_observer(layer_created)
        // Startup and State Systems
        .add_systems(Startup, startup)
        .add_systems(OnEnter(shared::State::Playing), spawn_minions)
        // Update Systems
        .add_systems(
            Update,
            (
                move_pathfinders.before(PathingSet),
                set_new_goal.run_if(in_state(shared::State::Playing)),
                handle_pathfinding_failed.run_if(in_state(shared::State::Playing)),
                randomize_nav
                    .run_if(in_state(shared::State::Playing))
                    .before(PathingSet),
                update_tile_colors.run_if(in_state(shared::State::Playing)),
            ),
        )
        // You only need to add the `NorthstarPluginSettings` resource if you want to change the default settings.
        // The default settings are 16 agents per frame for pathfinding and 32 for collision avoidance.
        .insert_resource(NorthstarPluginSettings {
            max_pathfinding_agents_per_frame: 48,
            max_collision_avoidance_agents_per_frame: 64,
        })
        .insert_resource(TileTexturesToUpdate::default())
        .run();
}

fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Get our anchor positioning calculated
    let anchor = TilemapAnchor::Center;

    let tilemap_size = TilemapSize { x: 128, y: 128 };
    let tilemap_gridsize = TilemapGridSize { x: 8.0, y: 8.0 };
    let tilemap_tilesize = TilemapTileSize { x: 8.0, y: 8.0 };

    let offset = anchor.as_offset(
        &tilemap_size,
        &tilemap_gridsize,
        &tilemap_tilesize,
        &TilemapType::Square,
    );

    let camera_offset = Vec3::new(
        offset.x + (tilemap_size.x as f32 * tilemap_gridsize.x) / 2.0,
        offset.y + (tilemap_size.y as f32 * tilemap_gridsize.y) / 2.0,
        1.0,
    );

    // Spawn a 2D camera and set the position based on the centered anchor offset
    commands.spawn(Camera2d).insert(Transform {
        translation: camera_offset,
        ..Default::default()
    });

    // Load the map ...
    let map_handle: Handle<TiledMap> = asset_server.load("demo_128.tmx");

    let mut map_entity = commands.spawn((TiledMapHandle(map_handle), anchor));

    let grid_settings = GridSettingsBuilder::new_2d(128, 128)
        .chunk_size(16)
        .enable_collision()
        // You can add a neighbor filter like this. It will add a little overhead on refined paths.
        //.add_neighbor_filter(filter::NoCornerCutting)
        .avoidance_distance(4)
        .build();

    // Insert the grid as a child of the map entity. This won't currently affect anything, but in the future
    // we may want to have the grid as a child of the map entity so that multiple grids can be supported.
    map_entity.insert((
        TilemapRenderSettings {
            render_chunk_size: UVec2::new(32, 32),
            ..Default::default()
        },
        Grid::<OrdinalNeighborhood>::new(&grid_settings),
    ));

    // Add the debug map as a child of the entity containing the Grid.
    // Set the translation to offset the the debug gizmos.
    map_entity.with_child((
        DebugGridBuilder::new(8, 8)
            .enable_chunks()
            .enable_entrances()
            .enable_cached_paths()
            .enable_show_connections_on_hover()
            .build(),
        // Add the offset to the debug gizmo so that it aligns with your tilemap.
        DebugOffset(offset.extend(0.0)),
    ));
}

fn layer_created(
    trigger: Trigger<TiledLayerCreated>,
    map_asset: Res<Assets<TiledMap>>,
    grid: Single<&mut Grid<OrdinalNeighborhood>>,
    mut state: ResMut<NextState<shared::State>>,
) {
    let mut grid = grid.into_inner();

    let layer = trigger.event().get_layer(&map_asset);
    if let Some(layer) = layer {
        if let Some(tile_layer) = layer.as_tile_layer() {
            let width = tile_layer.width().unwrap();
            let height = tile_layer.height().unwrap();

            for x in 0..width {
                for y in 0..height {
                    let tile = tile_layer.get_tile(x as i32, y as i32);
                    if let Some(tile) = tile {
                        let tile_id = tile.id();

                        if tile_id == 14 {
                            grid.set_nav(UVec3::new(x, height - 1 - y, 0), Nav::Passable(1));
                        } else {
                            grid.set_nav(UVec3::new(x, height - 1 - y, 0), Nav::Impassable);
                        }
                    }
                }
            }
        }
    }

    info!("Loaded layer: {:?}", layer);
    grid.build();

    state.set(shared::State::Playing);
}

fn spawn_minions(
    mut commands: Commands,
    grid: Single<(Entity, &Grid<OrdinalNeighborhood>)>,
    layer_entity: Query<Entity, With<TiledMapTileLayer>>,
    tilemap: Single<(
        &TilemapSize,
        &TilemapTileSize,
        &TilemapGridSize,
        &TilemapAnchor,
    )>,
    asset_server: Res<AssetServer>,
    mut walkable: ResMut<shared::Walkable>,
    config: Res<shared::Config>,
) {
    let (grid_entity, grid) = grid.into_inner();
    let (map_size, tile_size, grid_size, anchor) = tilemap.into_inner();

    let offset = anchor.as_offset(map_size, grid_size, tile_size, &TilemapType::Square);

    let layer_entity = layer_entity.iter().next().unwrap();

    walkable.tiles = Vec::new();
    for x in 0..grid.width() {
        for y in 0..grid.height() {
            if grid.is_passable(UVec3::new(x, y, 0)) {
                let position = Vec3::new(x as f32 * 8.0, y as f32 * 8.0, 0.0);

                walkable.tiles.push(position);
            }
        }
    }

    let mut count = 0;

    while count < 128 {
        let position = walkable.tiles.choose(&mut rand::rng()).unwrap();
        let goal = walkable.tiles.choose(&mut rand::rng()).unwrap();

        let transform = Vec3::new(position.x, position.y, 4.0) + offset.extend(0.0);

        // Generate random color
        let color = Color::srgb(
            rand::random::<f32>(),
            rand::random::<f32>(),
            rand::random::<f32>(),
        );

        let mut pathfind = Pathfind::new_2d((goal.x / 8.0) as u32, (goal.y / 8.0) as u32);

        match config.mode {
            PathfindMode::AStar => pathfind = pathfind.mode(PathfindMode::AStar),
            PathfindMode::Coarse => pathfind = pathfind.mode(PathfindMode::Coarse),
            PathfindMode::Refined => pathfind = pathfind.mode(PathfindMode::Refined),
            PathfindMode::ThetaStar => pathfind = pathfind.mode(PathfindMode::ThetaStar),
        }

        commands
            .spawn(Sprite {
                image: asset_server.load("tiles/tile_0018_edit.png"),
                color,
                ..Default::default()
            })
            .insert(Name::new(format!("{color:?}")))
            //.insert(DebugPath::new(color))
            .insert(AgentOfGrid(grid_entity))
            .insert(Blocking)
            .insert(Transform::from_translation(transform))
            .insert(AgentPos(UVec3::new(
                (position.x / 8.0) as u32,
                (position.y / 8.0) as u32,
                0,
            )))
            .insert(pathfind)
            .insert(ChildOf(layer_entity));

        count += 1;
    }
}

fn move_pathfinders(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AgentPos, &Pathfind, &NextPos)>,
    grid: Single<&Grid<OrdinalNeighborhood>>,
    tilemap: Single<(
        &TilemapSize,
        &TilemapTileSize,
        &TilemapGridSize,
        &TilemapAnchor,
    )>,
    mut tick_reader: EventReader<shared::Tick>,
) {
    let (map_size, tile_size, grid_size, anchor) = tilemap.into_inner();
    let grid = grid.into_inner();

    for _ in tick_reader.read() {
        let offset = anchor.as_offset(map_size, grid_size, tile_size, &TilemapType::Square);

        for (entity, mut position, pathfind, next) in query.iter_mut() {
            if grid.nav(next.0) == Some(Nav::Impassable) {
                // We're making random dynamic changes to the grid in `randomize_nav`.
                // It's up to us to handle this case. It's not automatically handled by the pathfinding systems because different games will want to handle it differently.
                commands
                    .entity(entity)
                    // For our case, we'll just reinsert the same goal to generate a new path.
                    .insert(Pathfind::new(pathfind.goal).mode(pathfind.mode))
                    .remove::<NextPos>();
                continue;
            }

            position.0 = next.0;

            let translation = Vec3::new(
                next.0.x as f32 * grid_size.x + offset.x,
                next.0.y as f32 * grid_size.y + offset.y,
                4.0,
            );

            commands
                .entity(entity)
                .insert(Transform::from_translation(translation))
                .remove::<NextPos>();
        }
    }
}

fn set_new_goal(
    mut commands: Commands,
    mut minions: Query<Entity, (Without<Path>, Without<Pathfind>)>,
    walkable: Res<shared::Walkable>,
    config: Res<shared::Config>,
) {
    for entity in minions.iter_mut() {
        let new_goal = walkable.tiles.choose(&mut rand::rng()).unwrap();

        let mut pathfind = Pathfind::new_2d((new_goal.x / 8.0) as u32, (new_goal.y / 8.0) as u32);

        match config.mode {
            PathfindMode::AStar => pathfind = pathfind.mode(PathfindMode::AStar),
            PathfindMode::Coarse => pathfind = pathfind.mode(PathfindMode::Coarse),
            PathfindMode::Refined => pathfind = pathfind.mode(PathfindMode::Refined),
            PathfindMode::ThetaStar => pathfind = pathfind.mode(PathfindMode::ThetaStar),
        }

        commands.entity(entity).insert(pathfind);
    }
}

#[allow(clippy::type_complexity)]
fn handle_pathfinding_failed(
    mut commands: Commands,
    minions: Query<Entity, Or<(With<PathfindingFailed>, With<RerouteFailed>)>>,
    config: Res<shared::Config>,
    walkable: Res<shared::Walkable>,
) {
    // Pathfinding failed, normally we might have our AI come up with a new plan,
    // but for this example, we'll just reroute to a new random goal.
    for entity in &minions {
        //log::info!("Pathfinding failed for entity {entity:?}, setting new goal.");
        let new_goal = walkable.tiles.choose(&mut rand::rng()).unwrap();

        let mut pathfind = Pathfind::new_2d((new_goal.x / 8.0) as u32, (new_goal.y / 8.0) as u32);

        match config.mode {
            PathfindMode::AStar => pathfind = pathfind.mode(PathfindMode::AStar),
            PathfindMode::Coarse => pathfind = pathfind.mode(PathfindMode::Coarse),
            PathfindMode::Refined => pathfind = pathfind.mode(PathfindMode::Refined),
            PathfindMode::ThetaStar => pathfind = pathfind.mode(PathfindMode::ThetaStar),
        }

        commands
            .entity(entity)
            .insert(pathfind)
            .remove::<PathfindingFailed>()
            .remove::<RerouteFailed>();
    }
}

// FOR GRID REBUILDING DEMO: We store a list of tile positions that have changed to Impassable or Passable.
// This is used to update the tile colors in the grid to represent the new state of the tiles.
#[derive(Resource, Default)]
struct TileTexturesToUpdate {
    tiles: HashMap<TilePos, Color>,
}

const IMPASSABLE_COLOR: Color = Color::srgba(1.0, 0.0, 0.0, 1.0); // Red, 70% alpha
const PASSABLE_COLOR: Color = Color::srgba(0.0, 1.0, 0.0, 1.0); // Light green, 50% alpha

// FOR GRID REBUILDING DEMO: Simple system to add a green or red overlay to the tiles that have changed.
// This is a simple way to visualize the changes in the grid without adding the complexity of changing the tile textures.
fn update_tile_colors(
    mut query: Query<(&TilePos, &mut TileColor)>, // Adjust component name to your color type
    tiles_to_update: Res<TileTexturesToUpdate>,
) {
    for (tile_pos, mut tile_color) in query.iter_mut() {
        if tiles_to_update.tiles.contains_key(tile_pos) {
            tile_color.0 = tiles_to_update
                .tiles
                .get(tile_pos)
                .cloned()
                .unwrap_or_default();
        }
    }
}

// FOR GRID REBUILDING DEMO: Every five seconds, randomly changes the nav data for 25 tiles in the grid.
// It then calls `grid::build()` to rebuild only the affected areas of the grid.
// This is a simple way to demonstrate how the grid can be modified dynamically.
fn randomize_nav(
    grid: Single<&mut Grid<OrdinalNeighborhood>>,
    positions: Query<&AgentPos>,
    mut timer: Local<Timer>,
    time: Res<Time>,
    mut tiles_to_update: ResMut<TileTexturesToUpdate>,
    config: Res<shared::Config>,
) {
    // Initialize the timer if it hasn't been already.
    if timer.finished() && timer.duration().as_secs_f32() == 0.0 {
        // Set to repeat every few seconds by default.
        *timer = Timer::from_seconds(5.0, TimerMode::Repeating);
    }

    timer.tick(time.delta());
    if timer.just_finished() {
        if !config.random_rebuild {
            // If random_rebuild is false, we don't do anything.
            return;
        }

        let mut grid = grid.into_inner();

        // Build a list of agentpos positions to avoid modifying their tiles
        let agent_positions: Vec<UVec3> = positions.iter().map(|pos| pos.0).collect();

        // This is a bit of an extreme example.
        // Most games would not need to modify this many tiles in a variety of chunks in a single frame.
        // For example, an explosion or an entity mining a wall might only modify tiles in a single chunk.
        for _ in 0..1 {
            let x = rand::random::<u32>() % grid.width();
            let y = rand::random::<u32>() % grid.height();
            let z = rand::random::<u32>() % grid.depth();

            let pos = UVec3::new(x, y, z);

            if agent_positions.contains(&pos) {
                // Skip positions that are already occupied by agents.
                continue;
            }

            // random Nav::Passable(1) or Nav::Impassable
            if rand::random::<bool>() {
                grid.set_nav(pos, Nav::Passable(1));
                tiles_to_update
                    .tiles
                    .insert(TilePos::new(pos.x, pos.y), PASSABLE_COLOR);
            } else {
                grid.set_nav(pos, Nav::Impassable);
                tiles_to_update
                    .tiles
                    .insert(TilePos::new(pos.x, pos.y), IMPASSABLE_COLOR);
            }
        }

        grid.build();

        // All the paths might be invalid now so we need to remove the path component and trigger a pathfinding update.
        // The current `NorthstarPlugin` systems will not handle this automatically.
        /*for (entity, pathfind) in &pathfinders {
            // Reinsert the Path component to trigger a pathfinding update.
            commands
                .entity(entity)
                .insert(Pathfind {
                    goal: pathfind.goal,
                    partial: pathfind.partial,
                    mode: pathfind.mode,
                })
                .remove::<Path>();
        }*/
    }
}
