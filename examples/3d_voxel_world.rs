use bevy::prelude::*;
use bevy_northstar::prelude::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_voxel_world::prelude::*;
use std::sync::Arc;
// Declare materials as consts for convenience
const SNOWY_BRICK: u8 = 0;
const FULL_BRICK: u8 = 1;
const GRASS: u8 = 2;

// Animation tuning
const LERP_SPEED: f32 = 10.0;
const POSITION_TOLERANCE: f32 = 0.01;
const FLOOR_SIZE: u32 = 32;
const PLAYER_OFFSET: f32 = 0.5;

#[derive(Resource, Clone, Default)]
struct MyMainWorld;

impl VoxelWorldConfig for MyMainWorld {
    type MaterialIndex = u8;
    type ChunkUserBundle = ();

    fn texture_index_mapper(&self) -> Arc<dyn Fn(Self::MaterialIndex) -> [u32; 3] + Send + Sync> {
        Arc::new(|vox_mat: u8| match vox_mat {
            SNOWY_BRICK => [0, 1, 2],
            FULL_BRICK => [2, 2, 2],
            GRASS | _ => [3, 3, 3],
        })
    }

    fn voxel_lookup_delegate(&self) -> VoxelLookupDelegate<Self::MaterialIndex> {
        Box::new(move |_chunk_pos| create_voxel_floor())
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // We can specify a custom texture when initializing the plugin.
        // This should just be a path to an image in your assets folder.
        .add_plugins(VoxelWorldPlugin::with_config(MyMainWorld))
        .add_plugins(PanOrbitCameraPlugin)
        .add_event::<AnimationWaitEvent>()
        .add_systems(Startup, (setup, create_voxel_scene))
        .add_systems(PreUpdate, move_pathfinders)
        .add_systems(
            Update,
            (
                update_cursor_cube,
                mouse_button_input,
                manual_rebuild_grid,
                player_input_3d,
                animate_move,
                pathfind_error,
            ),
        )
        .add_plugins(NorthstarPlugin::<CardinalNeighborhood3d>::default())
        .add_plugins(NorthstarDebugPlugin::<CardinalNeighborhood3d>::default())
        .run();
}

#[derive(Component)]
struct CursorCube {
    voxel_pos: IVec3,
}
// Player marker
#[derive(Component)]
pub struct Player;

// Event that lets other systems know to wait until animations are completed.
#[derive(Debug, Event)]
pub struct AnimationWaitEvent;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Cursor cube
    commands.spawn((
        Transform::from_xyz(0.0, -10.0, 0.0),
        MeshMaterial3d(materials.add(Color::srgba_u8(124, 144, 255, 128))),
        Mesh3d(meshes.add(Mesh::from(Cuboid {
            half_size: Vec3::splat(0.5),
        }))),
        CursorCube {
            voxel_pos: IVec3::new(0, -10, 0),
        },
    ));

    // Camera
    commands.spawn((
        Transform::from_xyz(-5.0, 16.0, -5.0).looking_at(Vec3::new(16.0, 0.0, 16.0), Vec3::Y),
        // This tells bevy_voxel_world to use this cameras transform to calculate spawning area
        VoxelWorldCamera::<MyMainWorld>::default(),
        PanOrbitCamera {
            pan_sensitivity: 0.0,
            focus: Vec3::new(16.0, 0.0, 16.0),
            button_orbit: MouseButton::Middle,
            ..default()
        },
    ));

    // light
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(16.0, 8.0, 16.0),
    ));

    //Debug grid
    // Build the grid settings: cover a larger area and keep flat z-depth.
    // Adjust sizes as needed; this should roughly cover the visible/interactive terrain.
    let grid_settings = GridSettingsBuilder::new_3d(FLOOR_SIZE, 16, FLOOR_SIZE)
        .chunk_size(8)
        // For 2.5D you will likely want a chunk depth greater than 1.
        // This will allow short paths to use direct A* to create more natural paths to height changes.
        .chunk_depth(8)
        .enable_diagonal_connections()
        .default_impassable()
        // This is a great example of when to use a neighbor filter.
        // Since we're Y Sorting, we don't want to allow the player to move diagonally around walls as the sprite will z transition through the wall.
        // We use `NoCornerCuttingFlat` here instead of `NoCornerCutting` because we want to allow diagonal movement to other height levels.
        // .add_neighbor_filter(filter::NoCornerCuttingFlat)
        .build();
    // Call `build()` to return the component.

    let debug_grid = DebugGridBuilder::new(2, 2)
        .set_depth(2)
        .tilemap_type(DebugTilemapType::Square)
        .enable_chunks()
        .enable_entrances()
        .build();
    // Spawn the grid component
    let mut grid_ec = commands.spawn(CardinalGrid3d::new(&grid_settings));
    grid_ec.with_child(debug_grid);
    let grid_entity = grid_ec.id();

    // player
    commands.spawn((
        Transform::from_xyz(
            9.0 + PLAYER_OFFSET,
            1.0 + PLAYER_OFFSET,
            9.0 + PLAYER_OFFSET,
        ),
        Player,
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 0.0))),
        // Northstar agent setup
        AgentPos(UVec3::new(9, 1, 9)),
        AgentOfGrid(grid_entity),
        DebugPath::new(Color::srgb(1.0, 0.0, 0.0)),
    ));
}

fn create_voxel_scene(mut voxel_world: VoxelWorld<MyMainWorld>) {
    // Some bricks
    voxel_world.set_voxel(IVec3::new(16, 1, 16), WorldVoxel::Solid(SNOWY_BRICK));
    voxel_world.set_voxel(IVec3::new(17, 1, 16), WorldVoxel::Solid(SNOWY_BRICK));
    voxel_world.set_voxel(IVec3::new(16, 1, 17), WorldVoxel::Solid(SNOWY_BRICK));
    voxel_world.set_voxel(IVec3::new(16, 1, 15), WorldVoxel::Solid(SNOWY_BRICK));
    voxel_world.set_voxel(IVec3::new(15, 1, 16), WorldVoxel::Solid(FULL_BRICK));
    voxel_world.set_voxel(IVec3::new(14, 1, 16), WorldVoxel::Solid(FULL_BRICK));
    voxel_world.set_voxel(IVec3::new(15, 2, 16), WorldVoxel::Solid(SNOWY_BRICK));
    voxel_world.set_voxel(IVec3::new(14, 2, 16), WorldVoxel::Solid(SNOWY_BRICK));
    voxel_world.set_voxel(IVec3::new(16, 2, 16), WorldVoxel::Solid(SNOWY_BRICK));
}

fn create_voxel_floor() -> Box<dyn FnMut(IVec3) -> WorldVoxel + Send + Sync> {
    Box::new(move |pos: IVec3| {
        if pos.x > 0 && pos.z > 0 && pos.x < FLOOR_SIZE as i32 && pos.z < FLOOR_SIZE as i32 {
            if pos.y < 1 {
                return WorldVoxel::Solid(GRASS);
            }
            return WorldVoxel::Air;
        }
        return WorldVoxel::Unset;
    })
}

fn update_cursor_cube(
    voxel_world_raycast: VoxelWorld<MyMainWorld>,
    camera_info: Query<(&Camera, &GlobalTransform), With<VoxelWorldCamera<MyMainWorld>>>,
    mut cursor_evr: EventReader<CursorMoved>,
    mut cursor_cube: Query<(
        &mut Transform,
        &mut CursorCube,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for ev in cursor_evr.read() {
        // Get a ray from the cursor position into the world
        let (camera, cam_gtf) = camera_info.single().unwrap();
        let Ok(ray) = camera.viewport_to_world(cam_gtf, ev.position) else {
            return;
        };

        if let Some(result) = voxel_world_raycast.raycast(ray, &|(_pos, _vox)| true) {
            let (mut transform, mut cursor_cube, material_handle) =
                cursor_cube.single_mut().unwrap();
            // Move the cursor cube to the position of the voxel we hit
            // Camera is by construction not in a solid voxel, so result.normal must be Some(...)
            let voxel_pos = result.position + result.normal.unwrap();
            transform.translation = voxel_pos + Vec3::new(0.5, 0.5, 0.5);
            cursor_cube.voxel_pos = voxel_pos.as_ivec3();
            if cursor_cube.voxel_pos.x < 0
                || cursor_cube.voxel_pos.y < 0
                || cursor_cube.voxel_pos.z < 0
            {
                //indicate that this voxel position is not ment to spawn a cube
                if let Some(mat) = materials.get_mut(&material_handle.0) {
                    mat.base_color = Color::srgba_u8(255, 144, 124, 128);
                }
            } else {
                if let Some(mat) = materials.get_mut(&material_handle.0) {
                    mat.base_color = Color::srgba_u8(124, 144, 255, 128);
                }
            }
        }
    }
}

fn mouse_button_input(
    buttons: Res<ButtonInput<MouseButton>>,
    mut voxel_world: VoxelWorld<MyMainWorld>,
    cursor_cube: Query<&CursorCube>,
) {
    if buttons.just_pressed(MouseButton::Right) {
        let vox = cursor_cube.single().unwrap();
        //don't allow the player to spawn bricks that will not be navigatable with northstar
        if vox.voxel_pos.x < 0 || vox.voxel_pos.y < 0 || vox.voxel_pos.z < 0 {
            return;
        }
        voxel_world.set_voxel(vox.voxel_pos, WorldVoxel::Solid(FULL_BRICK));
    }
}

// Allow manual rebuild of the navigation grid to cope with async terrain generation timing.
// Press 'G' to (re)build the grid at any time.
fn manual_rebuild_grid(keys: Res<ButtonInput<KeyCode>>, grid: Single<&mut CardinalGrid3d>) {
    if keys.just_pressed(KeyCode::KeyG) {
        let mut grid = grid.into_inner();
        info!("Manual grid rebuild triggered (G)");
        //mark every voxel on height 1 as passable
        for x in 0..FLOOR_SIZE {
            for z in 0..FLOOR_SIZE {
                let squash_pos = UVec3::new(x, 1, z);
                if grid.in_bounds(squash_pos) {
                    grid.set_nav(squash_pos, Nav::Passable(1));
                }
            }
        }
        grid.build();
        info!("Grid rebuilt");
    }
}

// Handle click-to-move using the cursor cube's current voxel position
fn player_input_3d(
    buttons: Res<ButtonInput<MouseButton>>,
    player_q: Query<Entity, With<Player>>,
    cursor_q: Query<&CursorCube>,
    mut commands: Commands,
) {
    if buttons.just_pressed(MouseButton::Left) {
        if let (Ok(player), Ok(cursor)) = (player_q.single(), cursor_q.single()) {
            let pos_i = cursor.voxel_pos;

            // Disallow invalid negative targets
            if pos_i.x < 0 || pos_i.y < 0 || pos_i.z < 0 {
                return;
            }
            let pos = pos_i.as_uvec3();
            info!("Player movement target via click: {:?}", pos);
            commands
                .entity(player)
                .insert(Pathfind::new_3d(pos.x, pos.y, pos.z));
        }
    }
}

// Advance agent logical position along the computed path when not animating
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

// Smoothly animate the player mesh toward the center of its current voxel
fn animate_move(
    mut query: Query<(&AgentPos, &mut Transform)>,
    time: Res<Time>,
    mut ev_wait: EventWriter<AnimationWaitEvent>,
) {
    for (position, mut transform) in query.iter_mut() {
        let target = Vec3::new(
            position.0.x as f32 + PLAYER_OFFSET,
            position.0.y as f32 + PLAYER_OFFSET,
            position.0.z as f32 + PLAYER_OFFSET,
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

// Handle pathfinding failures cleanly
fn pathfind_error(query: Query<Entity, With<PathfindingFailed>>, mut commands: Commands) {
    for entity in query.iter() {
        error!("Pathfinding failed for entity: {:?}", entity);
        commands
            .entity(entity)
            .remove::<PathfindingFailed>()
            .remove::<Pathfind>()
            .remove::<NextPos>();
    }
}
