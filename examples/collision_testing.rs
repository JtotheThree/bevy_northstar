use bevy::{
    dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin},
    log,
    platform::collections::HashMap,
    prelude::*,
    text::FontSmoothing,
};

use bevy_northstar::prelude::*;

use bevy_ecs_tiled::prelude::*;
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
        .add_plugins((TilemapPlugin, TiledPlugin::default()))
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
            ),
        )
        .run();
}

fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Get our anchor positioning calculated
    let anchor = TilemapAnchor::Center;

    let tilemap_size = TilemapSize { x: 16, y: 16 };
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
    commands.spawn(Camera2d).insert((
        Transform {
            translation: camera_offset,
            ..Default::default()
        },
        Projection::Orthographic(OrthographicProjection {
            scale: 0.01,
            ..OrthographicProjection::default_2d()
        }),
    ));

    // Load the map ...
    let map_handle: Handle<TiledMapAsset> = asset_server.load("demo_16.tmx");

    let mut map_entity = commands.spawn((TiledMap(map_handle), anchor));

    let grid_settings = GridSettingsBuilder::new_2d(16, 16)
        .chunk_size(8)
        .default_impassable()
        .enable_collision()
        .avoidance_distance(4)
        .build();

    // You can eventually add some extra settings to your map
    map_entity.insert((
        TilemapRenderSettings {
            render_chunk_size: UVec2::new(32, 32),
            ..Default::default()
        },
        Grid::<OrdinalNeighborhood>::new(&grid_settings),
    ));

    map_entity.with_child((
        DebugGridBuilder::new(8, 8).build(),
        DebugOffset(offset.extend(0.0)),
        Transform::from_translation(offset.extend(0.0)),
    ));
}

fn layer_created(
    trigger: Trigger<TiledEvent<LayerCreated>>,
    map_asset: Res<Assets<TiledMapAsset>>,
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
    layer_entity: Query<Entity, With<TiledLayer>>,
    tilemap: Single<(
        &TilemapSize,
        &TilemapTileSize,
        &TilemapGridSize,
        &TilemapAnchor,
    )>,
    asset_server: Res<AssetServer>,
    mut walkable: ResMut<shared::Walkable>,
) {
    let (grid_entity, grid) = grid.into_inner();
    let (map_size, tile_size, grid_size, anchor) = tilemap.into_inner();

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

    let offset = anchor.as_offset(map_size, grid_size, tile_size, &TilemapType::Square);

    let mut count = 0;
    let mut rng = rand::rng();

    while count < 6 {
        let position = walkable.tiles.choose(&mut rng).unwrap();

        let transform = Vec3::new(position.x, position.y, 4.0) + offset.extend(0.0);

        // Generate random color
        let color = Color::srgb(
            rand::random::<f32>(),
            rand::random::<f32>(),
            rand::random::<f32>(),
        );

        commands
            .spawn(Sprite {
                image: asset_server.load("tiles/tile_0018_edit.png"),
                color,
                ..Default::default()
            })
            .insert(Name::new(format!("{color:?}")))
            .insert(DebugPath {
                color,
                draw_unrefined: false,
            })
            .insert(AgentOfGrid(grid_entity))
            .insert(Blocking)
            .insert(Transform::from_translation(transform))
            .insert(AgentPos(UVec3::new(
                (position.x / 8.0) as u32,
                (position.y / 8.0) as u32,
                0,
            )))
            .insert(ChildOf(layer_entity));

        count += 1;
    }
}

fn move_pathfinders(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AgentPos, &NextPos)>,
    tilemap: Single<(
        &TilemapSize,
        &TilemapTileSize,
        &TilemapGridSize,
        &TilemapAnchor,
    )>,
    mut tick_reader: EventReader<shared::Tick>,
) {
    let (map_size, tile_size, grid_size, anchor) = tilemap.into_inner();

    for _ in tick_reader.read() {
        let mut debug_next = HashMap::new();

        let offset = anchor.as_offset(map_size, grid_size, tile_size, &TilemapType::Square);

        for (entity, mut position, next) in query.iter_mut() {
            if debug_next.contains_key(&next.0) {
                log::error!(
                    "Entity {:?} has the same next position as another entity: {:?} {:?}",
                    entity,
                    next,
                    debug_next.get(&next.0)
                );
                continue;
            }

            debug_next.insert(next.0, entity);

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
    config: Res<shared::Config>,
    walkable: Res<shared::Walkable>,
) {
    for entity in minions.iter_mut() {
        let new_goal = walkable.tiles.choose(&mut rand::rng()).unwrap();

        let mut pathfind = Pathfind::new_2d((new_goal.x / 8.0) as u32, (new_goal.y / 8.0) as u32);

        match config.mode {
            PathfindMode::AStar => pathfind = pathfind.mode(PathfindMode::AStar),
            PathfindMode::Coarse => pathfind = pathfind.mode(PathfindMode::Coarse),
            PathfindMode::Refined => pathfind = pathfind.mode(PathfindMode::Refined),
            PathfindMode::Waypoints => pathfind = pathfind.mode(PathfindMode::Waypoints),
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
        log::info!("Pathfinding failed for entity {entity:?}, setting new goal.");
        let new_goal = walkable.tiles.choose(&mut rand::rng()).unwrap();

        let mut pathfind = Pathfind::new_2d((new_goal.x / 8.0) as u32, (new_goal.y / 8.0) as u32);

        match config.mode {
            PathfindMode::AStar => pathfind = pathfind.mode(PathfindMode::AStar),
            PathfindMode::Coarse => pathfind = pathfind.mode(PathfindMode::Coarse),
            PathfindMode::Refined => pathfind = pathfind.mode(PathfindMode::Refined),
            PathfindMode::Waypoints => pathfind = pathfind.mode(PathfindMode::Waypoints),
            PathfindMode::ThetaStar => pathfind = pathfind.mode(PathfindMode::ThetaStar),
        }

        commands
            .entity(entity)
            .insert(pathfind)
            .remove::<PathfindingFailed>()
            .remove::<RerouteFailed>();
    }
}
