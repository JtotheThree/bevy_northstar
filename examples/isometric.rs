// Example of a 2.5D isometric tilemap using Bevy Northstar
// Press ` to toggle the debug view, +/- to change the z depth of the debug view.
// In the debug view hover over an entrance node to see its internal connections.

use bevy::{log, prelude::*};
use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use bevy_northstar::prelude::*;

mod shared;

// Game state
#[derive(Clone, Debug, Default, Hash, Eq, States, PartialEq)]
pub enum State {
    #[default]
    Loading,
    Playing,
}

// Tiled tile properties component, see bevy_ecs_tiled for how to use Tiled properties.
#[derive(Component, Default, Debug, Reflect)]
#[reflect(Component)]
pub struct Tile {
    pub info: TileInfo,
}

// Tiled TileInfo properties, see bevy_ecs_tiled for how to use Tiled properties.
#[derive(Component, Default, Debug, Reflect, Clone)]
#[reflect(Component)]
pub struct TileInfo {
    pub height: i32,
    pub ramp: bool,
}

// Player marker
#[derive(Component)]
pub struct Player;

// Marks entity that needs to be Y Sorted and holds additional height offset information.
#[derive(Component, Reflect, Debug)]
pub struct YSort(pub f32);

// The sprite offset where the sprite should be Y sorted, this would usually be placed at the foot of the player etc.
#[derive(Component, Debug, Reflect)]
pub struct Pivot(pub Vec2);

// Event that lets other systems know to wait until animations are completed.
#[derive(Debug, Event)]
pub struct AnimationWaitEvent;

// Event that signals the completion of the tilemap loading.
#[derive(Debug, Event)]
pub struct LoadCompleteEvent;

// Cursor resource to track the moused over tile
#[derive(Default, Resource, Reflect)]
pub struct Cursor {
    pub tile: Option<UVec3>,
}

// The Y offset for each higher tile layer in our isometric Tiled map.
// Higher layers in isometric need to be y offset so that they align properly in the isometric view.
const LAYER_Y_OFFSET: f32 = 16.0;
// In our example the tileset tiles are used to determine the height of a tile, not just the layer.
// Each taller tile increases in 4 pixels.
const HEIGHT_OFFSET: f32 = 4.0;
// The maximum height our height/layer multipliers can reach in our example map.
// Technically the map goes to height 13, but the player can't reach those heights.
const MAX_HEIGHT: u32 = 9;
// Offset to position our sprite propery onto the center of a tile.
const PLAYER_CENTER_OFFSET: f32 = 4.0;

// Constants for animation
pub const LERP_SPEED: f32 = 22.0;
pub const POSITION_TOLERANCE: f32 = 0.1;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(ImagePlugin::default_nearest()))
        .add_plugins(TiledMapPlugin::default())
        // Add the Northstar Plugin with an isometric neighborhood for 3D pathfinding.
        .add_plugins(NorthstarPlugin::<OrdinalNeighborhood3d>::default())
        // Add the Debug Plugin to visualize the grid and pathfinding.
        .add_plugins(NorthstarDebugPlugin::<OrdinalNeighborhood3d>::default())
        .add_event::<LoadCompleteEvent>()
        .add_event::<AnimationWaitEvent>()
        .add_systems(Startup, startup)
        .add_systems(
            PreUpdate,
            (
                update_cursor,
                input,
                debug_input,
                move_pathfinders,
                warp.after(move_pathfinders),
            )
                .run_if(in_state(State::Playing)),
        )
        .add_systems(
            Update,
            (animate_move, pathfind_error).run_if(in_state(State::Playing)),
        )
        .add_systems(
            PostUpdate,
            (y_sort, camera_follow_player).run_if(in_state(State::Playing)),
        )
        .add_observer(tile_created)
        .add_observer(loading_complete)
        .register_type::<Tile>()
        .register_type::<TileInfo>()
        .insert_resource(Cursor::default())
        .insert_state(State::Loading)
        .insert_resource(NorthstarPluginSettings {
            pathfind_settings: PathfindSettings {
                default_mode: PathfindMode::ThetaStar,
                ..Default::default()
            },
            ..Default::default()
        })
        .run();
}

fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.spawn(Camera2d);

    let grid_settings = GridSettingsBuilder::new_3d(64, 64, 13)
        .chunk_size(8)
        // For 2.5D you will likely want a chunk depth greater than 1.
        // This will allow short paths to use direct A* to create more natural paths to height changes.
        .chunk_depth(4)
        .default_impassable()
        // This is a great example of when to use a neighbor filter.
        // Since we're Y Sorting, we don't want to allow the player to move diagonally around walls as the sprite will z transition through the wall.
        // We use `NoCornerCuttingFlat` here instead of `NoCornerCutting` because we want to allow diagonal movement to other height levels.
        .add_neighbor_filter(filter::NoCornerCuttingFlat)
        .build();

    let map_handle: Handle<TiledMap> = asset_server.load("isotilemap.tmx");

    commands.spawn((
        TiledMapHandle(map_handle),
        TilemapAnchor::Center,
        TilemapRenderSettings {
            render_chunk_size: UVec2::new(1, 1),
            y_sort: true,
        },
        // Ensure that bevy_ecs_tiled uses layer z offsets that align with our height offsets.
        TiledMapLayerZOffset(LAYER_Y_OFFSET),
        Grid::<OrdinalNeighborhood3d>::new(&grid_settings),
    ));
}

fn tile_created(
    trigger: Trigger<TiledTileCreated>,
    mut query: Query<(&mut TileInfo, &TilePos)>,
    grid: Single<&mut OrdinalGrid3d>,
    mut commands: Commands,
) {
    let mut grid = grid.into_inner();

    if let Ok((mut tile_info, tile_pos)) = query.get_mut(trigger.event().entity) {
        // HACK: I'm not aware of anyway to know when all the tiles have been fully loaded.
        // There's a hiddden tile with a height of 99 in the corner of the map on layer 2
        // with a height of 99 that is used to determine the end of the tilemap loading.
        if tile_info.height == 99 {
            commands.trigger(LoadCompleteEvent);
            return;
        }

        let layer = trigger.event().layer;

        // Treat the higher layers as having a higher height offset.
        let layer_height_offset = match layer.id {
            0 => 0,
            1 => 4,
            2 => 8,
            _ => 0,
        };

        // Readjust the tile_info height based on the layer.
        tile_info.height += layer_height_offset as i32;

        // Manual way of setting our teleporter without needing to add a bunch of object layer handling to the example.
        if tile_pos.x == 11 && tile_pos.y == 28 && tile_info.height == 8 {
            // We want a two-way teleporter, but we want to ensure the player doesn't warp back and forth,
            // so we set the target of the first teleporter to be adjacent to the second teleporter.
            grid.set_nav(
                UVec3::new(tile_pos.x, tile_pos.y, tile_info.height as u32),
                // Long hand way to create a portal.
                Nav::Portal(Portal {
                    target: UVec3::new(2, 45, 0),
                    cost: 1,
                    one_way: true,
                }),
            )
        } else if tile_pos.x == 1 && tile_pos.y == 46 && tile_info.height == 4 {
            grid.set_nav(
                UVec3::new(tile_pos.x, tile_pos.y, tile_info.height as u32),
                // Short hand way, new takes the target UVec3 and cost.
                Nav::Portal(Portal::to(UVec3::new(12, 28, 8), 1, true)),
            )
        // If the tile is a ramp, we set it as a portal with the target being the same x,y but a higher z position.
        // This allows the player to climb higher elevations in the map.
        } else if tile_info.ramp {
            grid.set_nav(
                UVec3::new(tile_pos.x, tile_pos.y, tile_info.height as u32),
                Nav::Portal(Portal::to(
                    UVec3::new(tile_pos.x, tile_pos.y, 4 + layer_height_offset),
                    1,
                    false,
                )),
            );
        } else {
            // We've hit a bog standard walkable tile, so we'll set nav as passable there.
            let pos = UVec3::new(tile_pos.x, tile_pos.y, tile_info.height as u32);
            grid.set_nav(pos, Nav::Passable(1));

            // You don't have to do the following, but it makes designing maps easier.
            // All of our tiles are technically walkable, so when tiles are stacked on top of each other it can create paths through the stack.
            // You could design that out of our your tilemap, but it's easier on design to just make a few tiles below impassable.
            if tile_info.height > 4 {
                for z in 2..tile_info.height {
                    let squash_pos = UVec3::new(tile_pos.x, tile_pos.y, z as u32);
                    if grid.in_bounds(squash_pos) {
                        grid.set_nav(squash_pos, Nav::Impassable);
                    }
                }
            }
        }
    }
}

fn loading_complete(
    _: Trigger<LoadCompleteEvent>,
    map_query: Query<shared::MapQuery>,
    camera: Single<(&mut Transform, &mut Projection), With<Camera>>,
    grid: Single<(Entity, &mut OrdinalGrid3d)>,
    asset_server: Res<AssetServer>,
    mut state: ResMut<NextState<State>>,
    mut commands: Commands,
) {
    let (grid_entity, mut grid) = grid.into_inner();

    grid.build();

    let player_start = UVec3::new(32, 25, 0);

    // Insert the debug grid as a child to the grid entity
    if let Some(map) = map_query.iter().next() {
        commands.entity(grid_entity).with_child((
            DebugGridBuilder::new(32, 16).isometric().build(),
            DebugOffset(
                map.anchor
                    .as_offset(map.map_size, map.grid_size, map.tile_size, map.map_type)
                    .extend(0.0),
            ),
            DebugDepthYOffsets(
                (0..=MAX_HEIGHT)
                    .map(|h| (h, h as f32 * HEIGHT_OFFSET))
                    .collect(),
            ),
        ));
    }

    let center = map_query
        .iter()
        .next()
        .map(|map| {
            TilePos::new(player_start.x, player_start.y).center_in_world(
                map.map_size,
                map.grid_size,
                map.tile_size,
                map.map_type,
                map.anchor,
            )
        })
        .unwrap();

    log::info!("Spawning player at: {:?}", center);

    // Spawn the player.
    commands.spawn((
        Player,
        Sprite::from_image(asset_server.load("player.png")),
        AgentPos(player_start),
        AgentOfGrid(grid_entity),
        Transform::from_translation(Vec3::new(center.x, center.y + PLAYER_CENTER_OFFSET, 0.0)),
        // Tag our Player as Y Sortable
        YSort(0.0),
        // Adds a pivot to where we want to start the Y sort on the player sprite.
        Pivot(Vec2::new(0.0, -10.0)),
        DebugPath::new(Color::srgb(1.0, 0.0, 0.0)),
    ));

    // Zoom camera into the player
    let (mut transform, mut projection) = camera.into_inner();
    if let Projection::Orthographic(ref mut ortho) = &mut *projection {
        ortho.scale = 0.5;
        transform.translation = Vec3::new(center.x, center.y + PLAYER_CENTER_OFFSET, 10.0);
    }

    state.set(State::Playing);
}

fn update_cursor(
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform, &Transform), With<Camera>>,
    mut cursor: ResMut<Cursor>,
    map_query: Query<shared::MapQuery>,
    tile_storage: Query<&TileStorage>,
    tile_info: Query<&TileInfo>,
    debug_cursor: Single<&mut DebugCursor>,
) {
    let window = window.into_inner();
    let (camera, camera_transform, _) = camera.into_inner();
    let mut debug_cursor = debug_cursor.into_inner();

    if let Some(mut cursor_position) = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok())
    {
        let map = map_query.iter().next().expect("No map found in the query");

        let offset = Vec2::new(0.0, map.grid_size.y / 2.0);
        debug_cursor.0 = Some(cursor_position);
        cursor_position += offset;

        let mut selected_tile = None;

        // Test the highest to lowest tile near the cursor position and select the higher tile if there's a conflict.
        for test_height in (0..=MAX_HEIGHT).rev() {
            let height_visual_offset = test_height as f32 * HEIGHT_OFFSET;
            let adjusted_cursor = cursor_position + Vec2::new(0.0, -height_visual_offset);

            if let Some(tile_pos) = TilePos::from_world_pos(
                &adjusted_cursor,
                map.map_size,
                map.grid_size,
                map.tile_size,
                map.map_type,
                map.anchor,
            ) {
                // Find the tile with the highest height at this position
                let mut top_tile: Option<(Entity, i32)> = None;
                for storage in tile_storage.iter() {
                    if let Some(tile) = storage.get(&tile_pos) {
                        if let Ok(info) = tile_info.get(tile) {
                            let tile_height = info.height as f32;
                            let mut tile_world = TilePos::center_in_world(
                                &tile_pos,
                                map.map_size,
                                map.grid_size,
                                map.tile_size,
                                map.map_type,
                                map.anchor,
                            );
                            // Add height offset to the tile world position
                            // The visual "click" area of each tile's height is different to the view of the user
                            // so we adjust that here.
                            tile_world.y += tile_height * HEIGHT_OFFSET;

                            // At a certain point the tiles in this tile position are actually a full tile y offset away from the cursor
                            // so we want to ensure we ignore these.
                            if (cursor_position.y - tile_world.y).abs() > LAYER_Y_OFFSET {
                                continue; // Skip tiles that are too far y offset
                            }

                            if top_tile.is_none() || info.height > top_tile.unwrap().1 {
                                top_tile = Some((tile, info.height));
                            }
                        }
                    }
                }

                if let Some((_, height)) = top_tile {
                    if height as u32 == test_height {
                        selected_tile = Some(UVec3::new(tile_pos.x, tile_pos.y, test_height));
                        break; // Found the topmost tile
                    }
                }
            }
        }

        cursor.tile = selected_tile;
    }
}

fn input(
    player: Single<Entity, With<Player>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<&mut Transform, With<Camera>>,
    cursor: Res<Cursor>,
    mut commands: Commands,
) {
    let player = player.into_inner();
    let mut camera_transform = camera.into_inner();

    if keyboard.pressed(KeyCode::KeyW) {
        // Move the camera up
        camera_transform.translation.y += 10.0;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        // Move the camera down
        camera_transform.translation.y -= 10.0;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        // Move the camera left
        camera_transform.translation.x -= 10.0;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        // Move the camera right
        camera_transform.translation.x += 10.0;
    }

    if mouse.just_pressed(MouseButton::Left) {
        if let Some(tile) = cursor.tile {
            log::info!("Pathfinding to tile: {:?}", tile);
            commands
                .entity(player)
                .insert(Pathfind::new_3d(tile.x, tile.y, tile.z));
        }
    }
}

fn debug_input(keyboard: Res<ButtonInput<KeyCode>>, mut debug_query: Query<&mut DebugGrid>) {
    if let Ok(mut debug_grid) = debug_query.single_mut() {
        if keyboard.just_pressed(KeyCode::Backquote) {
            debug_grid.toggle_cells();
            debug_grid.toggle_chunks();
            debug_grid.toggle_entrances();
            debug_grid.toggle_cached_paths();
            debug_grid.toggle_show_connections_on_hover();
        }

        if keyboard.just_pressed(KeyCode::Equal) {
            let new_depth = debug_grid.depth() + 1;

            if new_depth <= MAX_HEIGHT {
                debug_grid.set_depth(new_depth);
            }
        }

        if keyboard.just_pressed(KeyCode::Minus) {
            let current_depth = debug_grid.depth();
            debug_grid.set_depth(current_depth.saturating_sub(1));
        }
    }
}

// PathfindingFailed is added to the entity when a route to the requested goal is not found.
// Normally you might want to provide some feedback ot the player, but for this example we just log it.
fn pathfind_error(query: Query<Entity, With<PathfindingFailed>>, mut commands: Commands) {
    for entity in query.iter() {
        log::error!("Pathfinding failed for entity: {:?}", entity);
        commands
            .entity(entity)
            .remove::<PathfindingFailed>()
            .remove::<Pathfind>()
            .remove::<NextPos>();
    }
}

// Warp system to bypass the animation system for teleporters.
fn warp(
    mut query: Query<(&mut AgentPos, &mut Path, &mut Transform)>,
    map_query: Query<shared::MapQuery>,
    grid: Single<&OrdinalGrid3d>,
) {
    let map = map_query.iter().next().expect("No map found in the query");
    let grid = grid.into_inner();

    for (mut position, mut path, mut transform) in query.iter_mut() {
        if let Some(Nav::Portal(portal)) = grid.nav(position.0) {
            // Check if this is just a ramp, if so, we don't need to warp.
            // You could also just check the tilemap type from your map.
            if position.0.x == portal.target.x && position.0.y == portal.target.y {
                continue;
            }

            // Update the position to the portal's target
            position.0 = portal.target;

            // Update the transform to the portal's target position
            let tile_pos = TilePos::new(position.0.x, position.0.y);
            let base_vec = TilePos::center_in_world(
                &tile_pos,
                map.map_size,
                map.grid_size,
                map.tile_size,
                map.map_type,
                map.anchor,
            );

            // Set the transform position to the portal's target position,
            // We're going to cheat a bit not bother with proper z height calculation and
            // just let animate_move clean it up.
            transform.translation = Vec3::new(
                base_vec.x,
                base_vec.y + (portal.target.z as f32 * HEIGHT_OFFSET) + PLAYER_CENTER_OFFSET,
                transform.translation.z,
            );

            // Check if the next position in the path is the target of the portal
            // We do this because the player may just step into the portal as the goal,
            // if not we need to remove a step from the path.
            if let Some(next_pos) = path.next() {
                if next_pos == portal.target {
                    path.pop();
                }
            }
        }
    }
}

fn move_pathfinders(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AgentPos, &NextPos)>,
    animation_reader: EventReader<AnimationWaitEvent>,
) {
    if !animation_reader.is_empty() {
        return;
    }

    for (entity, mut position, next) in query.iter_mut() {
        position.0 = next.0;

        commands.entity(entity).remove::<NextPos>();
    }
}

fn animate_move(
    mut query: Query<(&AgentPos, &mut Transform, &mut YSort)>,
    map_query: Query<shared::MapQuery>,
    time: Res<Time>,
    mut ev_wait: EventWriter<AnimationWaitEvent>,
) {
    let map = map_query.iter().next().expect("No map found in the query");

    for (position, mut transform, mut ysort) in query.iter_mut() {
        let tile_pos = TilePos {
            x: position.0.x,
            y: position.0.y,
        };

        let base_vec = TilePos::center_in_world(
            &tile_pos,
            map.map_size,
            map.grid_size,
            map.tile_size,
            map.map_type,
            map.anchor,
        );

        let height_offset = position.0.z as f32 * HEIGHT_OFFSET;

        ysort.0 = height_offset - (MAX_HEIGHT - 1) as f32 * HEIGHT_OFFSET;

        let target = Vec3::new(
            base_vec.x,
            base_vec.y + height_offset + PLAYER_CENTER_OFFSET, // Apply elevation and center offset here
            transform.translation.z,
        );

        let d = (target - transform.translation).length();
        let animating = if d > POSITION_TOLERANCE {
            transform.translation = transform
                .translation
                .lerp(target, LERP_SPEED * time.delta_secs());
            true
        } else {
            transform.translation = target;
            false
        };

        if animating {
            ev_wait.write(AnimationWaitEvent);
        }
    }
}

fn y_sort(
    mut query: Query<(&mut Transform, &YSort, &Pivot)>,
    map_query: Query<(&TilemapSize, &TilemapTileSize)>,
) {
    let Some((map_size, tile_size)) = map_query.iter().next() else {
        return;
    };

    let max_y = map_size.y as f32 * tile_size.y;

    for (mut transform, ysort, pivot) in query.iter_mut() {
        let y = transform.translation.y + pivot.0.y;

        // Match the y-sort algorithm that bevy_ecs_tilemap uses.
        // We don't need to worry about Tiled layers because we handle our own height offsets and sort all that out in the
        // animate_move system and ensure that the bevy_ecs_tiled TiledMapLayerZOffset aligns with our height offsets.
        transform.translation.z = ysort.0 + (1.0 - (y / max_y));
    }
}

const CAMERA_MARGIN: f32 = 25.0;
const CAMERA_LERP_SPEED: f32 = 8.0;

fn camera_follow_player(
    time: Res<Time>,
    windows: Query<&Window>,
    mut camera_query: Query<&mut Transform, (With<Camera2d>, Without<Player>)>,
    player_query: Single<&Transform, With<Player>>,
) {
    let Some(window) = windows.iter().next() else {
        return;
    };

    let Some(mut camera_transform) = camera_query.iter_mut().next() else {
        return;
    };

    let player_transform = player_query.into_inner();

    let player_pos = player_transform.translation.truncate();
    let camera_pos = camera_transform.translation.truncate();

    let quarter_width = window.width() / 8.0;
    let quarter_height = window.height() / 8.0;

    let left_edge = camera_pos.x - quarter_width + CAMERA_MARGIN;
    let right_edge = camera_pos.x + quarter_width - CAMERA_MARGIN;
    let bottom_edge = camera_pos.y - quarter_height + CAMERA_MARGIN;
    let top_edge = camera_pos.y + quarter_height - CAMERA_MARGIN;

    let mut target_camera_pos = camera_pos;

    if player_pos.x < left_edge {
        target_camera_pos.x = player_pos.x + quarter_width - CAMERA_MARGIN;
    } else if player_pos.x > right_edge {
        target_camera_pos.x = player_pos.x - quarter_width + CAMERA_MARGIN;
    }

    if player_pos.y < bottom_edge {
        target_camera_pos.y = player_pos.y + quarter_height - CAMERA_MARGIN;
    } else if player_pos.y > top_edge {
        target_camera_pos.y = player_pos.y - quarter_height + CAMERA_MARGIN;
    }

    // Smoothly interpolate the camera position
    let lerped = camera_pos.lerp(target_camera_pos, CAMERA_LERP_SPEED * time.delta_secs());
    camera_transform.translation.x = lerped.x;
    camera_transform.translation.y = lerped.y;
}
