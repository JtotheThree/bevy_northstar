// Demonstrates rerouting active agents after dynamic grid changes.
//
// When you modify the grid with `set_nav()` + `build()`, existing paths are
// NOT automatically invalidated. You must remove `Path` and `NextPos` from
// each agent and re-insert `Pathfind` so the plugin computes a fresh route.
//
// RIGHT CLICK to toggle a cell between passable and impassable.
//   Watch the colored path lines snap to new routes around the change.

use bevy::prelude::*;
use bevy_northstar::prelude::*;

const TILE: f32 = 24.0;
const GRID_W: u32 = 32;
const GRID_H: u32 = 24;
const MOVE_SPEED: f32 = 100.0;

const WALL_COLOR: Color = Color::srgb(0.8, 0.2, 0.2);
const GRID_LINE_COLOR: Color = Color::srgba(0.4, 0.4, 0.4, 0.3);

/// Marks an agent that loops between two endpoints.
#[derive(Component)]
struct Patrol {
    a: UVec3,
    b: UVec3,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
        .add_plugins(NorthstarDebugPlugin::<CardinalNeighborhood>::default())
        .add_systems(Startup, (startup, build_grid.after(startup)))
        .add_systems(
            Update,
            (smooth_move, patrol_reverse, input, draw_grid, draw_agents),
        )
        .run();
}

fn offset() -> Vec3 {
    Vec3::new(
        -(GRID_W as f32) * TILE / 2.0,
        -(GRID_H as f32) * TILE / 2.0,
        0.0,
    )
}

fn grid_to_world(pos: UVec3) -> Vec3 {
    Vec3::new(pos.x as f32 * TILE, pos.y as f32 * TILE, 0.0) + offset()
}

fn startup(mut commands: Commands) {
    commands.spawn(Camera2d);

    let grid_settings = GridSettingsBuilder::new_2d(GRID_W, GRID_H)
        .chunk_size(8)
        .build();

    commands
        .spawn(CardinalGrid::new(&grid_settings))
        .with_child((
            DebugGridBuilder::new(TILE as u32, TILE as u32).build(),
            DebugOffset(offset()),
        ));

    // Two patrolling agents with colored debug paths.
    let agents = [
        (
            UVec3::new(2, 8, 0),
            UVec3::new(29, 8, 0),
            Color::srgb(0.2, 1.0, 0.2),
        ),
        (
            UVec3::new(2, 16, 0),
            UVec3::new(29, 16, 0),
            Color::srgb(0.3, 0.6, 1.0),
        ),
    ];

    for (start, goal, color) in agents {
        commands.spawn((
            Name::new("Agent"),
            AgentPos(start),
            Pathfind::new(goal),
            Patrol { a: start, b: goal },
            Transform::from_translation(grid_to_world(start)),
            DebugPath::new(color),
        ));
    }

    // HUD
    commands.spawn((
        Text::new("RIGHT CLICK to toggle walls - agents reroute automatically"),
        TextFont {
            font_size: 22.0,
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

fn build_grid(grid: Single<&mut CardinalGrid>) {
    let mut grid = grid.into_inner();

    // Two vertical wall segments with gaps — agents must path around them.
    for y in 3..21 {
        if y != 10 && y != 11 {
            grid.set_nav(UVec3::new(10, y, 0), Nav::Impassable);
        }
        if y != 6 && y != 7 {
            grid.set_nav(UVec3::new(20, y, 0), Nav::Impassable);
        }
    }
    grid.build();
}

/// Draw grid lines and filled red squares for impassable cells.
fn draw_grid(grid: Single<&CardinalGrid>, mut gizmos: Gizmos) {
    let off = offset();
    let half = TILE * 0.45;

    // Thin grid lines.
    for x in 0..=GRID_W {
        let x_pos = x as f32 * TILE + off.x - TILE / 2.0;
        let y_start = off.y - TILE / 2.0;
        let y_end = GRID_H as f32 * TILE + off.y - TILE / 2.0;
        gizmos.line_2d(
            Vec2::new(x_pos, y_start),
            Vec2::new(x_pos, y_end),
            GRID_LINE_COLOR,
        );
    }
    for y in 0..=GRID_H {
        let y_pos = y as f32 * TILE + off.y - TILE / 2.0;
        let x_start = off.x - TILE / 2.0;
        let x_end = GRID_W as f32 * TILE + off.x - TILE / 2.0;
        gizmos.line_2d(
            Vec2::new(x_start, y_pos),
            Vec2::new(x_end, y_pos),
            GRID_LINE_COLOR,
        );
    }

    // Filled squares for walls.
    for x in 0..GRID_W {
        for y in 0..GRID_H {
            let pos = UVec3::new(x, y, 0);
            if let Some(nav) = grid.nav(pos) {
                if matches!(nav, Nav::Impassable) {
                    let center = grid_to_world(pos).truncate();
                    gizmos.rect_2d(
                        Isometry2d::from_translation(center),
                        Vec2::splat(half * 2.0),
                        WALL_COLOR,
                    );
                }
            }
        }
    }
}

/// Draw agents as large colored squares.
fn draw_agents(query: Query<&Transform, With<AgentPos>>, mut gizmos: Gizmos) {
    for tf in &query {
        let pos = tf.translation.truncate();
        let size = TILE * 0.7;
        gizmos.rect_2d(
            Isometry2d::from_translation(pos),
            Vec2::splat(size),
            Color::WHITE,
        );
        gizmos.rect_2d(
            Isometry2d::from_translation(pos),
            Vec2::splat(size - 3.0),
            Color::srgb(0.1, 0.8, 0.1),
        );
    }
}

/// Smoothly move agents toward their NextPos.
fn smooth_move(
    mut query: Query<(Entity, &mut AgentPos, &NextPos, &mut Transform)>,
    mut commands: Commands,
    time: Res<Time>,
) {
    for (entity, mut agent_pos, next_pos, mut transform) in &mut query {
        let target = grid_to_world(next_pos.0);
        let direction = target - transform.translation;
        let distance = direction.length();
        let step = MOVE_SPEED * time.delta_secs();

        if distance <= step.max(1.0) {
            agent_pos.0 = next_pos.0;
            transform.translation = target;
            commands.entity(entity).remove::<NextPos>();
        } else {
            transform.translation += direction.normalize() * step;
        }
    }
}

/// When an agent finishes its path, send it back the other way.
fn patrol_reverse(
    mut query: Query<(Entity, &AgentPos, &Patrol, Option<&NextPos>, Option<&Path>)>,
    mut commands: Commands,
) {
    for (entity, agent_pos, patrol, next_pos, path) in &mut query {
        if next_pos.is_some() || path.is_some_and(|p| !p.is_empty()) {
            continue;
        }
        let goal = if agent_pos.0 == patrol.b {
            patrol.a
        } else {
            patrol.b
        };
        commands.entity(entity).insert(Pathfind::new(goal));
    }
}

fn input(
    mouse: Res<ButtonInput<MouseButton>>,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform), With<Camera>>,
    mut grid: Single<&mut CardinalGrid>,
    agents: Query<(Entity, &Patrol, &AgentPos)>,
    mut commands: Commands,
) {
    if !mouse.just_pressed(MouseButton::Right) {
        return;
    }

    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let (camera, cam_tf) = camera.into_inner();
    let Ok(world_pos) = camera.viewport_to_world_2d(cam_tf, cursor) else {
        return;
    };

    let off = offset();
    let cell = UVec3::new(
        ((world_pos.x - off.x) / TILE).round() as u32,
        ((world_pos.y - off.y) / TILE).round() as u32,
        0,
    );

    let Some(nav) = grid.nav(cell) else { return };
    if matches!(nav, Nav::Impassable) {
        grid.set_nav(cell, Nav::Passable(1));
    } else {
        grid.set_nav(cell, Nav::Impassable);
    }
    grid.build();

    // --- Reroute all active agents ---
    // After modifying the grid, existing Path and NextPos components are stale.
    // Remove both and re-insert Pathfind so the plugin computes a fresh route.
    for (entity, patrol, agent_pos) in &agents {
        let goal = if agent_pos.0 == patrol.b {
            patrol.a
        } else {
            patrol.b
        };
        commands
            .entity(entity)
            .remove::<(NextPos, Path)>()
            .insert(Pathfind::new(goal));
    }
}
