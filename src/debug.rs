//! Plugin to add systems for drawing gizmos. For debugging pathfinding.
use std::fmt::Debug;

use bevy::{color::palettes::css, log, math::Vec2, platform::collections::HashMap, prelude::*};

use crate::{
    components::{DebugCursor, DebugGrid, DebugNode, DebugPath},
    grid::Grid,
    neighbor::Neighborhood,
    path::Path,
    prelude::{AgentOfGrid, DebugDepthYOffsets, DebugNavMask, DebugOffset, Pathfind},
};

/// Required to calculate how to draw the debug gizmos
#[derive(Reflect, Debug, Clone, Default)]
pub enum DebugTilemapType {
    #[default]
    /// Square tilemap, where each tile is a square.
    Square,
    /// Isometric tilemap, where each tile is a diamond.
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
        app.add_systems(
            Update,
            (
                draw_debug_map::<N>,
                draw_debug_paths::<N>,
                update_debug_node::<N>,
            ),
        )
        .register_type::<DebugGrid>()
        .register_type::<DebugTilemapType>()
        .register_type::<DebugOffset>()
        .register_type::<DebugPath>()
        .register_type::<DebugDepthYOffsets>();
    }
}

// / Draw the debug gizmos for the chunks, cells, entrances, and cached paths.
fn draw_debug_map<N: Neighborhood + 'static>(
    query: Query<(
        &DebugOffset,
        &DebugGrid,
        &DebugNode,
        Option<&DebugDepthYOffsets>,
    )>,
    pathfinds: Query<&Pathfind>,
    grid: Query<&Grid<N>>,
    mut gizmos: Gizmos,
) {
    let grid = if let Ok(grid) = grid.single() {
        grid
    } else {
        return;
    };

    for (debug_offset, debug_grid, debug_cursor, debug_depth_offsets) in query.iter() {
        let half_tile_width = debug_grid.tile_width as f32 * 0.5;
        let half_tile_height = debug_grid.tile_height as f32 * 0.5;

        let offset = debug_offset.0.truncate();
        let offset_center = offset - Vec2::new(half_tile_width, half_tile_height);

        let debug_depth_offsets = if let Some(debug_depth_offsets) = debug_depth_offsets {
            debug_depth_offsets.0.clone()
        } else {
            HashMap::new()
        };

        let y_offset = debug_depth_offsets
            .get(&debug_grid.depth)
            .cloned()
            .unwrap_or_default();

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

                            gizmos.line_2d(bottom_left, bottom_right + y_offset, css::WHITE);
                            gizmos.line_2d(bottom_right, top_right + y_offset, css::WHITE);
                            gizmos.line_2d(top_right, top_left + y_offset, css::WHITE);
                            gizmos.line_2d(top_left, bottom_left + y_offset, css::WHITE);
                        }
                    }
                }
                DebugTilemapType::Isometric => {
                    for x in 1..chunk_width {
                        for y in 1..chunk_height {
                            let position = Vec2::new(
                                (y as f32 + x as f32) * half_tile_width * chunk_size as f32,
                                (y as f32 - x as f32) * half_tile_height * chunk_size as f32
                                    + y_offset,
                            );

                            let top = position
                                + Vec2::new(0.0, debug_grid.tile_height as f32 * chunk_size as f32);
                            let right = position
                                + Vec2::new(debug_grid.tile_width as f32 * chunk_size as f32, 0.0);
                            let bottom = position
                                - Vec2::new(0.0, debug_grid.tile_height as f32 * chunk_size as f32);
                            let left = position
                                - Vec2::new(debug_grid.tile_width as f32 * chunk_size as f32, 0.0);

                            gizmos.line_2d(top + offset_center, right + offset_center, css::WHITE);
                            gizmos.line_2d(
                                right + offset_center,
                                bottom + offset_center,
                                css::WHITE,
                            );
                            gizmos.line_2d(
                                bottom + offset_center,
                                left + offset_center,
                                css::WHITE,
                            );
                            gizmos.line_2d(left + offset_center, top + offset_center, css::WHITE);
                        }
                    }
                }
            }
        }

        if debug_grid.draw_cells {
            if debug_grid.depth > grid.depth() {
                continue;
            }

            let pathfind = debug_grid
                .debug_mask
                .and_then(|entity| pathfinds.get(entity).ok());

            // Draw cell gizmos
            for x in 0..grid.width() {
                for y in 0..grid.height() {
                    let mut cell = grid.navcell(UVec3::new(x, y, debug_grid.depth)).clone();

                    if let Some(pathfind) = pathfind {
                        if let Some(mask) = &pathfind.mask {
                            if let Ok(masked_cell) = mask.get(cell.clone(), UVec3::new(x, y, debug_grid.depth)) {
                                cell = masked_cell;
                            }
                        }
                    }

                    let color = if cell.is_impassable() {
                        css::RED
                    } else {
                        // Full white at 1 cost, and full green at u8::MAX cost.
                        let normalized_cost = cell.cost as f32 / u8::MAX as f32;
                        let other_colors = (1.0 - normalized_cost).clamp(0.0, 1.0);
                        Color::srgb(1.0, other_colors, other_colors).to_srgba()
                    };

                    let position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (x * debug_grid.tile_width) as f32,
                            (y * debug_grid.tile_height) as f32 + y_offset,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (y as f32 + x as f32) * half_tile_width,
                            (y as f32 - x as f32) * half_tile_height - half_tile_height + y_offset,
                        ),
                    };

                    gizmos.circle_2d(position + offset, 2.0, color);
                }
            }
        }

        if debug_grid.draw_entrances {
            // Draw graph nodes
            for node in grid.graph().nodes() {
                let pos_offset = *debug_depth_offsets.get(&node.pos.z).unwrap_or(&0.0);

                let position = match debug_grid.map_type {
                    DebugTilemapType::Square => Vec2::new(
                        (node.pos.x * debug_grid.tile_width) as f32,
                        (node.pos.y * debug_grid.tile_height) as f32 + pos_offset,
                    ),
                    DebugTilemapType::Isometric => Vec2::new(
                        (node.pos.y as f32 + node.pos.x as f32) * half_tile_width,
                        (node.pos.y as f32 - node.pos.x as f32) * half_tile_height
                            - half_tile_height
                            + pos_offset,
                    ),
                };

                let color = if node.pos.z == debug_grid.depth {
                    css::MAGENTA
                } else {
                    css::DARK_BLUE
                };

                gizmos.circle_2d(position + offset, 2.0, color);

                // If draw_edges_on_mouseover is enabled, and this node isn't moused over, skip drawing edges
                if debug_grid.show_connections_on_hover && debug_cursor.0 != Some(node.pos) {
                    continue;
                }

                // Draw the node connection only to nodes in other chunks
                for edge in node.edges() {
                    let neighbor = grid.graph().node_at(edge);
                    if let Some(neighbor) = neighbor {
                        if neighbor.chunk_index != node.chunk_index {
                            let neighbor_offset =
                                *debug_depth_offsets.get(&neighbor.pos.z).unwrap_or(&0.0);

                            let neighbor_position = match debug_grid.map_type {
                                DebugTilemapType::Square => Vec2::new(
                                    (neighbor.pos.x * debug_grid.tile_width) as f32,
                                    (neighbor.pos.y * debug_grid.tile_height) as f32
                                        + neighbor_offset,
                                ),
                                DebugTilemapType::Isometric => Vec2::new(
                                    (neighbor.pos.y as f32 + neighbor.pos.x as f32)
                                        * half_tile_width,
                                    (neighbor.pos.y as f32 - neighbor.pos.x as f32)
                                        * half_tile_height
                                        - half_tile_height
                                        + neighbor_offset,
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

                if debug_grid.show_connections_on_hover && debug_cursor.0 != Some(*prev) {
                    continue;
                }

                for next in iter {
                    let prev_offset = *debug_depth_offsets.get(&prev.z).unwrap_or(&0.0);

                    let prev_position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (prev.x * debug_grid.tile_width) as f32,
                            (prev.y * debug_grid.tile_height) as f32 + prev_offset,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (prev.y as f32 + prev.x as f32) * (debug_grid.tile_width as f32 * 0.5),
                            (prev.y as f32 - prev.x as f32) * (debug_grid.tile_height as f32 * 0.5)
                                + prev_offset
                                - half_tile_height,
                        ),
                    };

                    let next_offset = *debug_depth_offsets.get(&next.z).unwrap_or(&0.0);
                    let next_position = match debug_grid.map_type {
                        DebugTilemapType::Square => Vec2::new(
                            (next.x * debug_grid.tile_width) as f32,
                            (next.y * debug_grid.tile_height) as f32 + next_offset,
                        ),
                        DebugTilemapType::Isometric => Vec2::new(
                            (next.y as f32 + next.x as f32) * (debug_grid.tile_width as f32 * 0.5),
                            (next.y as f32 - next.x as f32) * (debug_grid.tile_height as f32 * 0.5)
                                + next_offset
                                - half_tile_height,
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
    debug_grid: Query<(&DebugGrid, &DebugOffset, Option<&DebugDepthYOffsets>)>,
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

        let (debug_grid, debug_offset, debug_depth_offsets) = debug_grid_vec[0];

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

            // Iterate over full_path drawing a line from one cell to the next cell until completed
            let mut iter = path.path().iter();
            let mut prev = iter.next().unwrap();

            for next in iter {
                let prev_y_offset = if let Some(depth_offsets) = debug_depth_offsets {
                    depth_offsets.0.get(&prev.z).cloned().unwrap_or_default()
                } else {
                    0.0
                };

                let next_y_offset = if let Some(depth_offsets) = debug_depth_offsets {
                    depth_offsets.0.get(&next.z).cloned().unwrap_or_default()
                } else {
                    0.0
                };

                let prev_position = match debug_grid.map_type {
                    DebugTilemapType::Square => Vec2::new(
                        (prev.x * debug_grid.tile_width) as f32,
                        (prev.y * debug_grid.tile_height) as f32 + prev_y_offset,
                    ),
                    DebugTilemapType::Isometric => Vec2::new(
                        (prev.y as f32 + prev.x as f32) * half_tile_width,
                        (prev.y as f32 - prev.x as f32) * half_tile_height - half_tile_height
                            + prev_y_offset,
                    ),
                };

                let next_position = match debug_grid.map_type {
                    DebugTilemapType::Square => Vec2::new(
                        (next.x * debug_grid.tile_width) as f32,
                        (next.y * debug_grid.tile_height) as f32 + next_y_offset,
                    ),
                    DebugTilemapType::Isometric => Vec2::new(
                        (next.y as f32 + next.x as f32) * half_tile_width,
                        (next.y as f32 - next.x as f32) * half_tile_height - half_tile_height
                            + next_y_offset,
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

fn update_debug_node<N: Neighborhood + 'static>(
    mut query: Query<(
        &mut DebugNode,
        &DebugGrid,
        &DebugCursor,
        &DebugOffset,
        &DebugDepthYOffsets,
    )>,
    grid: Query<&Grid<N>>,
) {
    for (mut node, debug_grid, cursor, offset, depth_offsets) in query.iter_mut() {
        let Some(cursor_pos) = cursor.0 else { continue };
        let Ok(grid) = grid.single() else { continue };

        let tile_half_size = Vec2::new(
            debug_grid.tile_width as f32 * 0.5,
            debug_grid.tile_height as f32 * 0.5,
        );

        // Base screen-to-local conversion
        let base_cursor_pos = cursor_pos - offset.0.truncate() + tile_half_size;

        let mut selected_node = None;

        // Depth-sorted scan for visible tile under cursor
        for test_depth in (0..=grid.depth()).rev() {
            let height_offset = depth_offsets
                .0
                .get(&test_depth)
                .cloned()
                .unwrap_or_default();

            let adjusted_cursor = base_cursor_pos + Vec2::new(0.0, -height_offset);

            let (x, y) = match debug_grid.map_type {
                DebugTilemapType::Square => {
                    let x = (adjusted_cursor.x / debug_grid.tile_width as f32).floor() as u32;
                    let y = (adjusted_cursor.y / debug_grid.tile_height as f32).floor() as u32;
                    (x, y)
                }
                DebugTilemapType::Isometric => {
                    let screen_x = adjusted_cursor.x;
                    let screen_y = adjusted_cursor.y;
                    let x = ((screen_x / tile_half_size.x - screen_y / tile_half_size.y) / 2.0)
                        .floor() as u32;
                    let y = ((screen_x / tile_half_size.x + screen_y / tile_half_size.y) / 2.0)
                        .floor() as u32;
                    (x, y)
                }
            };

            if let Some(n) = grid.graph().node_at(UVec3::new(x, y, test_depth)) {
                selected_node = Some(n.pos);
                break;
            }
        }

        node.0 = selected_node;
    }
}
