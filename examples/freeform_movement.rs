use bevy::prelude::*;

use bevy_northstar::prelude::*;

use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;

mod shared;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Add the Northstar Plugin with a selected neighborhood to use the built in pathfinding systems
        .add_plugins(NorthstarPlugin::<OrdinalNeighborhood>::default())
        // Add the Debug Plugin to visualize the grid and pathfinding
        .add_plugins(NorthstarDebugPlugin::<OrdinalNeighborhood>::default())
        .add_systems(Startup, startup)
        .run();
}

fn startup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
) {
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
        ..Default
        ::default()
    });
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
    
}