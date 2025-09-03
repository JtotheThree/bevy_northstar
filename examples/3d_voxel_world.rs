use bevy::prelude::*;
use bevy_northstar::prelude::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_voxel_world::prelude::*;
use std::sync::Arc;
// Declare materials as consts for convenience
const SNOWY_BRICK: u8 = 0;
const FULL_BRICK: u8 = 1;
const GRASS: u8 = 2;

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
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // We can specify a custom texture when initializing the plugin.
        // This should just be a path to an image in your assets folder.
        .add_plugins(VoxelWorldPlugin::with_config(MyMainWorld))
        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Startup, (setup, create_voxel_scene))
        .add_systems(
            Update,
            (update_cursor_cube, mouse_button_input, manual_rebuild_grid),
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

    // player
    commands.spawn((
        Transform::from_xyz(9.5, 1.5, 9.5),
        Player,
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 0.0))),
    ));

    //Debug grid
    // Build the grid settings: cover a larger area and keep flat z-depth.
    // Adjust sizes as needed; this should roughly cover the visible/interactive terrain.
    let grid_settings = GridSettingsBuilder::new_3d(32, 16, 32)
        .chunk_size(8)
        // For 2.5D you will likely want a chunk depth greater than 1.
        // This will allow short paths to use direct A* to create more natural paths to height changes.
        .chunk_depth(8)
        .enable_diagonal_connections()
        // .default_impassable()
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
    let mut grid_entity = commands.spawn(CardinalGrid3d::new(&grid_settings));
    grid_entity.with_child(debug_grid);
}

fn create_voxel_scene(mut voxel_world: VoxelWorld<MyMainWorld>) {
    // Then we can use the `u8` consts to specify the type of voxel

    // 20 by 20 floor
    for x in 0..32 {
        for z in 0..32 {
            voxel_world.set_voxel(IVec3::new(x, 0, z), WorldVoxel::Solid(GRASS));
            // Grassy floor
        }
    }

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

fn update_cursor_cube(
    voxel_world_raycast: VoxelWorld<MyMainWorld>,
    camera_info: Query<(&Camera, &GlobalTransform), With<VoxelWorldCamera<MyMainWorld>>>,
    mut cursor_evr: EventReader<CursorMoved>,
    mut cursor_cube: Query<(&mut Transform, &mut CursorCube)>,
) {
    for ev in cursor_evr.read() {
        // Get a ray from the cursor position into the world
        let (camera, cam_gtf) = camera_info.single().unwrap();
        let Ok(ray) = camera.viewport_to_world(cam_gtf, ev.position) else {
            return;
        };

        if let Some(result) = voxel_world_raycast.raycast(ray, &|(_pos, _vox)| true) {
            let (mut transform, mut cursor_cube) = cursor_cube.single_mut().unwrap();
            // Move the cursor cube to the position of the voxel we hit
            // Camera is by construction not in a solid voxel, so result.normal must be Some(...)
            let voxel_pos = result.position + result.normal.unwrap();
            transform.translation = voxel_pos + Vec3::new(0.5, 0.5, 0.5);
            cursor_cube.voxel_pos = voxel_pos.as_ivec3();
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
        voxel_world.set_voxel(vox.voxel_pos, WorldVoxel::Solid(FULL_BRICK));
    }
}

// Allow manual rebuild of the navigation grid to cope with async terrain generation timing.
// Press 'G' to (re)build the grid at any time.
fn manual_rebuild_grid(keys: Res<ButtonInput<KeyCode>>, grid: Single<&mut CardinalGrid3d>) {
    if keys.just_pressed(KeyCode::KeyG) {
        let mut grid = grid.into_inner();
        info!("Manual grid rebuild triggered (G)");

        grid.build();
        info!("Grid rebuilt");
    }
}
