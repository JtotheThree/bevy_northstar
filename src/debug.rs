//! Plugin to add systems for drawing gizmos. For debugging pathfinding.
use bevy::{color::palettes::css, math::Vec2, prelude::*};

use crate::{
    components::{DebugGrid, DebugPath},
    grid::Grid,
    neighbor::Neighborhood,
    path::Path,
    prelude::{AgentOfGrid, DebugOffset},
};

/// Required to calculate how to draw the debug gizmos
#[derive(Reflect, Debug, Clone, Default)]
pub enum DebugTilemapType {
    #[default]
    Square,
    Isometric,
}

/// Debug plugin for the Northstar pathfinding library.
/// Add this plugin to your app to add the systems required to draw the debug helper gizmos.
///
/// # Example
///
/// ```rust,no_run
/// use bevy::prelude::*;
/// use bevy_northstar::prelude::*;
///
/// fn main() {
///    App::new()
///       .add_plugins(DefaultPlugins)
///       .add_plugins(NorthstarDebugPlugin::<CardinalNeighborhood>::default())
///       .add_systems(Startup, setup);
/// }
///
/// fn setup(mut commands: Commands) {
///   let debug_grid = DebugGridBuilder::new(32, 32)
///      .enable_chunks()
///      .enable_entrances()
///      .enable_cached_paths()
///      .build();
///
///   commands.spawn(debug_grid);
/// }
///
#[derive(Clone)]
pub struct NorthstarDebugPlugin<N: Neighborhood + 'static> {
    /// Debug gizmos configuration
    pub _marker: std::marker::PhantomData<N>,
}

impl<N: Neighborhood + 'static> Default for NorthstarDebugPlugin<N> {
    fn default() -> Self {
        NorthstarDebugPlugin {
            _marker: std::marker::PhantomData,
        }
    }
}

impl<N: Neighborhood + 'static> Plugin for NorthstarDebugPlugin<N> {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.add_systems(Update, (draw_debug_map::<N>, draw_debug_paths::<N>))
            .register_type::<DebugGrid>()
            .register_type::<DebugTilemapType>();
    }
}

// / Draw the debug gizmos for the chunks, cells, entrances, and cached paths.
fn draw_debug_map<N: Neighborhood + 'static>(
    query: Query<(&DebugOffset, &DebugGrid)>,
    grid: Query<&Grid<N>>,
    mut gizmos: Gizmos,
) {
    let grid = if let Ok(grid) = grid.single() {
        grid
    } else {
        return;
    };

    for (debug_offset, debug_grid) in query.iter() {
        //let offset = transform.translation.truncate();
        let offset = debug_offset.0.truncate();

        let half_tile_width = debug_grid.tile_width as f32 * 0.5;
        let half_tile_height = debug_grid.tile_height as f32 * 0.5;

        if debug_grid.draw_chunks {
            // Draw chunk boundaries
            let chunk_size = grid.chunk_size();
            let chunk_width = grid.width() / chunk_size;
            let chunk_height = grid.height() / chunk_size;

            match debug_grid.map_type {
                DebugTilemapType::Square => {
                    for x in 0..chunk_width {
                        for y in 0..chunk_height {
                            let bottom_left = Vec2::new(
                                (x * chunk_size) as f32 * debug_grid.tile_width as f32,
                                (y * chunk_size) as f32 * debug_grid.tile_height as f32,
                            );
                            let bottom_right = Vec2::new(
                                ((x + 1) * chunk_size) as f32 * debug_grid.tile_width as f32,
                                (y * chunk_size) as f32 * debug_grid.tile_height as f32,
                            );
                            let top_left = Vec2::new(
                                (x * chunk_size) as f32 * debug_grid.tile_width as f32,
                                ((y + 1) * chunk_size) as f32 * debug_grid.tile_height as f32,
                            );
                            let top_right = Vec2::new(
                                ((x + 1) * chunk_size) as f32 * debug_grid.tile_width as f32,
                                ((y + 1) * chunk_size) as f32 * debug_grid.tile_height as f32,
                            );

                            let bottom_left = bottom_left + offset - half_tile_width;
                            let bottom_right = bottom_right + offset - half_tile_height;
                            let top_left = top_left + offset - half_tile_width;
                            let top_right = top_right + offset - half_tile_height;

                            gizmos.line_2d(bottom_left, bottom_right, css::WHITE);
                            gizmos.line_2d(bottom_right, top_right, css::WHITE);
                            gizmos.line_2d(top_right, top_left, css::WHITE);
                            gizmos.line_2d(top_left, bottom_left, css::WHITE);
                        }
                    }
                }
                DebugTilemapType::Isometric => {
                    for x in 1..chunk_width {
                        for y in 1..chunk_height {
                            let position = Vec2::new(
                                (y as f32 + x as f32) * half_tile_width * chunk_size as f32,
                                (y as f32 - x as f32) * half_tile_height * chunk_size as f32,
                            );

                            let top = position
                                + Vec2::new(0.0, debug_grid.tile_height as f32 * chunk_size as f32);
                            let right = position
                                + Vec2::new(debug_grid.tile_width as f32 * chunk_size as f32, 0.0);
                            let bottom = position
                                - Vec2::new(0.0, debug_grid.tile_height as f32 * chunk_size as f32);
                            let left = position
                                - Vec2::new(debug_grid.tile_width as f32 * chunk_size as f32, 0.0);

                            let offset = offset - Vec2::new(half_tile_width, half_tile_height);

                            gizmos.line_2d(top + offset, right + offset, css::WHITE);
                            gizmos.line_2d(right + offset, bottom + offset, css::WHITE);
                            gizmos.line_2d(bottom + offset, left + offset, css::WHITE);
                            gizmos.line_2d(left + offset, top + offset, css::WHITE);
                        }
                    }
                }
            }
        }

        if debug_grid.draw_cells {
            // Draw cell gizmos
            for x in 0..grid.width() {
                for y in 0..grid.height() {
                    let cell = grid.navcell(UVec3::new(x, y, 0));
                    let color = if cell.is_impassable() {
                        css::RED
                    } else {
                        css::WHITE
                    };

                    let position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (x * debug_grid.tile_width) as f32,
                            (y * debug_grid.tile_height) as f32,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (y as f32 + x as f32) * half_tile_width,
                            (y as f32 - x as f32) * half_tile_height - half_tile_height,
                        ),
                    };

                    gizmos.circle_2d(position + offset, 2.0, color);
                }
            }
        }

        if debug_grid.draw_entrances {
            // Draw graph nodes
            for node in grid.graph().nodes() {
                let position = match debug_grid.map_type {
                    DebugTilemapType::Square => Vec2::new(
                        (node.pos.x * debug_grid.tile_width) as f32,
                        (node.pos.y * debug_grid.tile_height) as f32,
                    ),
                    DebugTilemapType::Isometric => Vec2::new(
                        (node.pos.y as f32 + node.pos.x as f32) * half_tile_width,
                        (node.pos.y as f32 - node.pos.x as f32) * half_tile_height
                            - half_tile_height,
                    ),
                };

                gizmos.circle_2d(position + offset, 2.0, css::MAGENTA);

                // Draw the node connection only to nodes in other chunks
                for edge in node.edges() {
                    let neighbor = grid.graph().node_at(edge);
                    if let Some(neighbor) = neighbor {
                        if neighbor.chunk != node.chunk {
                            let neighbor_position = match debug_grid.map_type {
                                DebugTilemapType::Square => Vec2::new(
                                    (neighbor.pos.x * debug_grid.tile_width) as f32,
                                    (neighbor.pos.y * debug_grid.tile_height) as f32,
                                ),
                                DebugTilemapType::Isometric => Vec2::new(
                                    (neighbor.pos.y as f32 + neighbor.pos.x as f32)
                                        * half_tile_width,
                                    (neighbor.pos.y as f32 - neighbor.pos.x as f32)
                                        * half_tile_height
                                        - half_tile_height,
                                ),
                            };
                            gizmos.line_2d(
                                position + offset,
                                neighbor_position + offset,
                                css::GREEN,
                            );
                        }
                    }
                }
            }
        }

        if debug_grid.draw_cached_paths {
            let path_colors = [
                css::RED,
                css::PURPLE,
                css::YELLOW,
                css::PINK,
                css::DARK_CYAN,
                css::MAGENTA,
                css::GREY,
                css::GREEN,
                css::ORANGE,
                css::NAVY,
            ];

            let mut color_index = 0;

            // Draw cached paths
            for path in grid.graph().all_paths() {
                if debug_grid.draw_entrances && !debug_grid.draw_cached_paths && path.len() > 2 {
                    continue;
                }

                // Iterate over path.path() drawing a line from one cell to the next cell until completed
                let mut iter = path.path().iter();
                let mut prev = iter.next().unwrap();

                for next in iter {
                    let prev_position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (prev.x * debug_grid.tile_width) as f32,
                            (prev.y * debug_grid.tile_height) as f32,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (prev.y as f32 + prev.x as f32) * (debug_grid.tile_width as f32 * 0.5),
                            (prev.y as f32 - prev.x as f32) * (debug_grid.tile_height as f32 * 0.5),
                        ),
                    };

                    let next_position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (next.x * debug_grid.tile_width) as f32,
                            (next.y * debug_grid.tile_height) as f32,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (next.y as f32 + next.x as f32) * (debug_grid.tile_width as f32 * 0.5),
                            (next.y as f32 - next.x as f32) * (debug_grid.tile_height as f32 * 0.5),
                        ),
                    };

                    let mut color = css::BLUE;

                    if debug_grid.draw_entrances && debug_grid.draw_cached_paths {
                        color = path_colors[color_index % path_colors.len()];
                    }

                    gizmos.line_2d(prev_position + offset, next_position + offset, color);

                    prev = next;
                }

                color_index += 1;
            }
        }
    }
}

fn draw_debug_paths<N: Neighborhood + 'static>(
    grid_children: Query<(Entity, &Children), With<Grid<N>>>,
    debug_grid: Query<(&DebugGrid, &DebugOffset)>,
    debug_paths: Query<(&DebugPath, &Path, &AgentOfGrid)>,
    mut gizmos: Gizmos,
) {
    for (grid_entity, child) in grid_children {
        // Find the DebugGrid component for the Grid entity
        let debug_grid_vec: Vec<_> = child
            .iter()
            .filter_map(|child_entity| debug_grid.get(child_entity).ok())
            .collect();

        if debug_grid_vec.is_empty() {
            continue;
        }

        if debug_grid_vec.len() > 1 {
            warn!(
                "Multiple DebugGrid components found for Grid entity: {:?}",
                grid_entity
            );
        }

        let (debug_grid, debug_offset) = debug_grid_vec[0];

        let center_offset = debug_offset.0.truncate();

        for (debug_path, path, parent_grid) in debug_paths {
            if parent_grid.0 != grid_entity {
                continue;
            }

            if path.is_empty() {
                continue;
            }

            let half_tile_width = debug_grid.tile_width as f32 * 0.5;
            let half_tile_height = debug_grid.tile_height as f32 * 0.5;

            // Iterate over path.path() drawing a line from one cell to the next cell until completed
            let mut iter = path.path().iter();
            let mut prev = iter.next().unwrap();

            for next in iter {
                let prev_position = match debug_grid.map_type {
                    DebugTilemapType::Square => Vec2::new(
                        (prev.x * debug_grid.tile_width) as f32,
                        (prev.y * debug_grid.tile_height) as f32,
                    ),
                    DebugTilemapType::Isometric => Vec2::new(
                        (prev.y as f32 + prev.x as f32) * half_tile_width,
                        (prev.y as f32 - prev.x as f32) * half_tile_height - half_tile_height,
                    ),
                };

                let next_position = match debug_grid.map_type {
                    DebugTilemapType::Square => Vec2::new(
                        (next.x * debug_grid.tile_width) as f32,
                        (next.y * debug_grid.tile_height) as f32,
                    ),
                    DebugTilemapType::Isometric => Vec2::new(
                        (next.y as f32 + next.x as f32) * half_tile_width,
                        (next.y as f32 - next.x as f32) * half_tile_height - half_tile_height,
                    ),
                };

                let x = prev_position + center_offset;
                let y = next_position + center_offset;

                gizmos.line_2d(x, y, debug_path.color);

                prev = next;
            }

            if debug_path.draw_unrefined {
                let mut iter = path.graph_path.iter();
                let mut prev = if let Some(p) = iter.next() {
                    p
                } else {
                    continue;
                };

                let inverted_color = debug_path.color.to_srgba();
                let inverted_color = Color::srgba(
                    1.0 - inverted_color.red,
                    1.0 - inverted_color.green,
                    1.0 - inverted_color.blue,
                    inverted_color.alpha,
                );

                for next in iter {
                    let prev_position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (prev.x * debug_grid.tile_width) as f32,
                            (prev.y * debug_grid.tile_height) as f32,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (prev.y as f32 + prev.x as f32) * (debug_grid.tile_width as f32 * 0.5),
                            (prev.y as f32 - prev.x as f32) * (debug_grid.tile_height as f32 * 0.5),
                        ),
                    };

                    let next_position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (next.x * debug_grid.tile_width) as f32,
                            (next.y * debug_grid.tile_height) as f32,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (next.y as f32 + next.x as f32) * (debug_grid.tile_width as f32 * 0.5),
                            (next.y as f32 - next.x as f32) * (debug_grid.tile_height as f32 * 0.5),
                        ),
                    };

                    let x = prev_position + center_offset;
                    let y = next_position + center_offset;

                    gizmos.line_2d(x, y, inverted_color);

                    prev = next;
                }
            }
        }
    }
}
