use bevy::{log, prelude::*};

use bevy_northstar::prelude::*;

use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;

mod shared;

#[derive(Component)]
struct Player;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Add the Northstar Plugin with a selected neighborhood to use the built in pathfinding systems
        .add_plugins(NorthstarPlugin::<OrdinalNeighborhood>::default())
        // Add the Debug Plugin to visualize the grid and pathfinding
        .add_plugins(NorthstarDebugPlugin::<OrdinalNeighborhood>::default())
        .add_plugins((TilemapPlugin, TiledMapPlugin::default()))
        .add_systems(Startup, startup)
        .add_systems(OnEnter(shared::State::Playing), spawn_player)
        .add_systems(
            Update,
            (input, move_player).run_if(in_state(shared::State::Playing)),
        )
        .add_observer(layer_created)
        .insert_state(shared::State::Loading)
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
    let map_handle: Handle<TiledMap> = asset_server.load("demo_128.tmx");

    let mut map_entity = commands.spawn((TiledMapHandle(map_handle), anchor));

    let grid_settings = GridSettingsBuilder::new_2d(128, 128)
        .chunk_size(16)
        //.enable_collision()
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
        DebugGridBuilder::new(8, 8).build(),
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

fn spawn_player(
    mut commands: Commands,
    grid: Single<(Entity, &mut Grid<OrdinalNeighborhood>)>,
    layer_entity: Query<Entity, With<TiledMapTileLayer>>,
    tilemap: Single<(
        &TilemapSize,
        &TilemapTileSize,
        &TilemapGridSize,
        &TilemapAnchor,
    )>,
    asset_server: Res<AssetServer>,
) {
    let (grid_entity, _) = grid.into_inner();
    let (map_size, tile_size, grid_size, anchor) = tilemap.into_inner();
    let layer_entity = layer_entity.iter().next().unwrap();

    let offset = anchor.as_offset(map_size, grid_size, tile_size, &TilemapType::Square);

    let position = UVec3::new(60, 60, 0);
    let translation = Vec3::new(
        offset.x + (position.x as f32 * grid_size.x) + (tile_size.x / 2.0),
        offset.y + (position.y as f32 * grid_size.y) + (tile_size.y / 2.0),
        1.0,
    );

    let mut debug_path = DebugPath::new(Color::srgb(0.0, 1.0, 0.0));
    debug_path.draw_unrefined = true;

    commands.spawn((
        Player,
        Sprite {
            image: asset_server.load("tiles/tile_0018_edit.png"),
            ..Default::default()
        },
        AgentPos(position),
        debug_path,
        AgentOfGrid(grid_entity),
        Transform::from_translation(translation),
        ChildOf(layer_entity),
    ));
}

#[allow(clippy::too_many_arguments)]
fn input(
    input: Res<ButtonInput<MouseButton>>,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform, &Transform), With<Camera>>,
    player: Single<Entity, With<AgentPos>>,
    map_query: Query<shared::MapQuery>,
    debug_grid: Single<&mut DebugGrid>,
    grid: Single<&mut Grid<OrdinalNeighborhood>>,
    mut commands: Commands,
) {
    let window = window.into_inner();
    let (camera, camera_transform, _) = camera.into_inner();
    let player = player.into_inner();
    let mut debug_grid = debug_grid.into_inner();
    let grid = grid.into_inner();

    let map = map_query.iter().next().expect("No map found in the query");

    let clicked_tile: Option<TilePos> = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok())
        .and_then(|cursor_position| {
            let offset = Vec2::new(0.0, 0.0);
            let cursor_position = cursor_position - offset;

            TilePos::from_world_pos(
                &cursor_position,
                map.map_size,
                map.grid_size,
                map.tile_size,
                map.map_type,
                map.anchor,
            )
        });

    if input.just_pressed(MouseButton::Left) {
        if let Some(goal) = clicked_tile {
            let mask_layer = NavMaskLayer::new();
            mask_layer
                .insert_region_fill(
                    &grid,
                    NavRegion::new(UVec3::new(64, 64, 0), UVec3::new(84, 84, 0)),
                    NavCellMask::ModifyCost(500),
                )
                .unwrap();

            let nav_mask = NavMask::new();
            nav_mask.add_layer(mask_layer).ok();

            debug_grid.set_debug_mask(nav_mask.clone());

            log::info!("Pathfinding to: {:?}", goal);
            commands
                .entity(player)
                .insert(Pathfind::new(UVec3::new(goal.x, goal.y, 0)).mode(PathfindMode::Waypoints))
                .insert(AgentMask(nav_mask));
        }
    }
}

fn move_player(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AgentPos, &NextPos, &mut Transform)>,
    map_query: Query<shared::MapQuery>,
    time: Res<Time>,
) {
    let map = map_query.iter().next().expect("No map found in the query");

    for (entity, mut agent_pos, next_pos, mut transform) in query.iter_mut() {
        let tile_pos = TilePos::new(next_pos.0.x, next_pos.0.y);
        let world_pos = tile_pos.center_in_world(
            map.map_size,
            map.grid_size,
            map.tile_size,
            map.map_type,
            map.anchor,
        );

        let target = Vec3::new(world_pos.x, world_pos.y, 1.0);
        let direction = target - transform.translation;
        let distance = direction.length();

        if distance > 0.5 {
            let movement_distance = 100.0 * time.delta_secs();
            if movement_distance >= distance {
                // Would overshoot, so snap instead
                transform.translation = target;
                agent_pos.0 = next_pos.0;
                commands.entity(entity).remove::<NextPos>();
            } else {
                // Normal movement
                let normalized_direction = direction / distance; // Avoid normalize() for better precision
                transform.translation += normalized_direction * movement_distance;
            }
        } else {
            agent_pos.0 = next_pos.0;
            commands.entity(entity).remove::<NextPos>();
        }
    }
}
