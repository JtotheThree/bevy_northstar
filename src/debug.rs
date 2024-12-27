//! This module contains some tools to help you debug your application.
//!
use bevy::math::Vec2;
use bevy::{color::palettes::css, prelude::*};

use crate::grid::Grid;
use crate::neighbor::Neighborhood;
use crate::path::Path;

#[derive(Debug, Clone, Default)]
pub enum MapType {
    #[default]
    Square,
    SquareCentered,
    Isometric,
}

/// Debug [Gizmos] configuration
#[derive(Resource, Clone, Debug)]
pub struct NorthstarDebugConfig {
    pub grid_width: u32,
    pub grid_height: u32,
    pub map_type: MapType,
    pub draw_path: bool,
    pub draw_chunks: bool,
    pub draw_entrances: bool,
    pub draw_points: bool,
    pub draw_cached_paths: bool,
    pub color: Color,
}

impl Default for NorthstarDebugConfig {
    fn default() -> Self {
        NorthstarDebugConfig {
            grid_width: 16,
            grid_height: 16,
            map_type: MapType::Square,
            draw_path: true,
            draw_chunks: true,
            draw_entrances: true,
            draw_points: false,
            draw_cached_paths: false,
            color: bevy::prelude::Color::Srgba(css::RED),
        }
    }
}

#[derive(Clone)]
pub struct NorthstarDebugPlugin<N: Neighborhood + 'static> {
    /// Debug gizmos configuration
    pub config: NorthstarDebugConfig,
    pub _marker: std::marker::PhantomData<N>,
}

impl<N: Neighborhood + 'static> Default for NorthstarDebugPlugin<N> {
    fn default() -> Self {
        NorthstarDebugPlugin {
            config: Default::default(),
            _marker: std::marker::PhantomData,
        }
    }
}

impl<N: Neighborhood + 'static> Plugin for NorthstarDebugPlugin<N> {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.insert_resource(self.config.clone())
            .add_systems(Update, (draw_debug_grid::<N>, draw_debug_paths::<N>));
    }
}

fn draw_debug_paths<N: Neighborhood + 'static>(query: Query<&Path>, config: Res<NorthstarDebugConfig>, grid: Res<Grid<N>>, mut gizmos: Gizmos) {
    if !config.draw_path {
        return;
    }

    let offset = match config.map_type {
        MapType::Square => { Vec2::new(0.0, 0.0) },
        MapType::SquareCentered => {
            Vec2::new(
                ((grid.get_width() * config.grid_width) as f32 / 2.0) - config.grid_width as f32,
                ((grid.get_height() * config.grid_height) as f32 / 2.0) - config.grid_height as f32,
            )
        },
        MapType::Isometric => Vec2::new(grid.get_width() as f32 / 4.0, 0.0),
    };

    let mut color_index = 0;

    for path in query.iter() {
        if path.is_empty() {
            continue;
        }

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

        // Draw paths
        // Iterate over path.path() drawing a line from one point to the next point until completed
        let mut iter = path.path().iter();
        let mut prev = iter.next().unwrap();

        for next in iter {
            let prev_position = match config.map_type {
                MapType::Square | MapType::SquareCentered => Vec2::new(
                    (prev.x * config.grid_width) as f32,
                    (prev.y * config.grid_height) as f32,
                ),
                MapType::Isometric => Vec2::new(
                    (prev.y as f32 + prev.x as f32) * (config.grid_width as f32 * 0.5),
                    (prev.y as f32 - prev.x as f32) * (config.grid_height as f32 * 0.5),
                ),
            };

            let next_position = match config.map_type {
                MapType::Square | MapType::SquareCentered => Vec2::new(
                    (next.x * config.grid_width) as f32,
                    (next.y * config.grid_height) as f32,
                ),
                MapType::Isometric => Vec2::new(
                    (next.y as f32 + next.x as f32) * (config.grid_width as f32 * 0.5),
                    (next.y as f32 - next.x as f32) * (config.grid_height as f32 * 0.5),
                ),
            };

            let color = path_colors[color_index % path_colors.len()];

            gizmos.line_2d(prev_position - offset, next_position - offset, color);

            prev = next;
        }

        color_index += 1;
    }
}

fn draw_debug_grid<N: Neighborhood + 'static>(
    config: Res<NorthstarDebugConfig>,
    grid: Res<Grid<N>>,
    mut gizmos: Gizmos,
) {
    let offset = match config.map_type {
        MapType::Square => { Vec2::new(0.0, 0.0) },
        MapType::SquareCentered => {
            Vec2::new(
                ((grid.get_width() * config.grid_width) as f32 / 2.0) - config.grid_width as f32,
                ((grid.get_height() * config.grid_height) as f32 / 2.0) - config.grid_height as f32,
            )
        },
        MapType::Isometric => Vec2::new(grid.get_width() as f32 / 4.0, 0.0),
    };

    if config.draw_points {
        let center = Vec2::new(
            (config.grid_width as f32) / 2.0,
            (config.grid_height as f32) / 2.0,
        );

        // Draw point gizmos
        for x in 0..grid.get_width() {
            for y in 0..grid.get_height() {
                let point = grid.get_point(UVec3::new(x, y, 0));
                let color = if point.wall { css::RED } else { css::WHITE };

                let position = match config.map_type {
                    MapType::Square | MapType::SquareCentered => Vec2::new(
                        (x * config.grid_width) as f32,
                        (y * config.grid_height) as f32,
                    ),
                    MapType::Isometric => Vec2::new(
                        (y as f32 + x as f32) * (config.grid_width as f32 * 0.5),
                        (y as f32 - x as f32) * (config.grid_height as f32 * 0.5),
                    ),
                };

                gizmos.circle_2d(position - offset + center, 2.0, color);
            }
        }
    }

    if config.draw_chunks {
        // Draw chunk boundaries
        let chunk_size = grid.get_chunk_size();
        let chunk_width = grid.get_width() / chunk_size;
        let chunk_height = grid.get_height() / chunk_size;

        let offset = match config.map_type {
            MapType::Square => { Vec2::new(0.0, 0.0) },
            MapType::SquareCentered => {
                Vec2::new(
                    (grid.get_width() * config.grid_width) as f32 / 2.0,
                    (grid.get_height() * config.grid_height) as f32 / 2.0,
                ) - Vec2::new(
                    config.grid_width as f32 * 0.5,
                    config.grid_height as f32 * 0.5,
                )
            }
            MapType::Isometric => Vec2::new(0.0, 0.0),
        };

        for x in 0..chunk_width {
            for y in 0..chunk_height {
                match config.map_type {
                    MapType::Square | MapType::SquareCentered => {
                        let bottom_left = Vec2::new(
                            (x * chunk_size) as f32 * config.grid_width as f32,
                            (y * chunk_size) as f32 * config.grid_height as f32,
                        );
                        let bottom_right = Vec2::new(
                            ((x + 1) * chunk_size) as f32 * config.grid_width as f32,
                            (y * chunk_size) as f32 * config.grid_height as f32,
                        );
                        let top_left = Vec2::new(
                            (x * chunk_size) as f32 * config.grid_width as f32,
                            ((y + 1) * chunk_size) as f32 * config.grid_height as f32,
                        );
                        let top_right = Vec2::new(
                            ((x + 1) * chunk_size) as f32 * config.grid_width as f32,
                            ((y + 1) * chunk_size) as f32 * config.grid_height as f32,
                        );

                        gizmos.line_2d(bottom_left - offset, bottom_right - offset, css::WHITE);
                        gizmos.line_2d(bottom_right - offset, top_right - offset, css::WHITE);
                        gizmos.line_2d(top_right - offset, top_left - offset, css::WHITE);
                        gizmos.line_2d(top_left - offset, bottom_left - offset, css::WHITE);
                    }
                    MapType::Isometric => {
                        let position = Vec2::new(
                            (y as f32 + x as f32)
                                * (config.grid_width as f32 * 0.5)
                                * chunk_size as f32,
                            (y as f32 - x as f32)
                                * (config.grid_height as f32 * 0.5)
                                * chunk_size as f32,
                        );

                        let top = position
                            + offset
                            + Vec2::new(0.0, config.grid_height as f32 * chunk_size as f32);
                        let right = position
                            + offset
                            + Vec2::new(config.grid_width as f32 * chunk_size as f32, 0.0);
                        let bottom = position + offset
                            - Vec2::new(0.0, config.grid_height as f32 * chunk_size as f32);
                        let left = position + offset
                            - Vec2::new(config.grid_width as f32 * chunk_size as f32, 0.0);

                        gizmos.line_2d(top, right, css::WHITE);
                        gizmos.line_2d(right, bottom, css::WHITE);
                        gizmos.line_2d(bottom, left, css::WHITE);
                        gizmos.line_2d(left, top, css::WHITE);
                    }
                }
            }
        }
    }

    if config.draw_entrances {
        // Draw graph nodes
        let center = Vec2::new(
            (config.grid_width as f32) / 2.0,
            (config.grid_height as f32) / 2.0,
        );

        for node in grid.graph.get_nodes() {
            let position = match config.map_type {
                MapType::Square | MapType::SquareCentered => Vec2::new(
                    (node.pos.x * config.grid_width) as f32,
                    (node.pos.y * config.grid_height) as f32,
                ),
                MapType::Isometric => Vec2::new(
                    (node.pos.y as f32 + node.pos.x as f32) * (config.grid_width as f32 * 0.5),
                    (node.pos.y as f32 - node.pos.x as f32) * (config.grid_height as f32 * 0.5),
                ),
            };

            gizmos.circle_2d(position - offset + center, 2.0, css::MAGENTA);
        }
    }

    if config.draw_cached_paths || config.draw_entrances {
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
        for path in grid.graph.get_all_paths() {
            if config.draw_entrances && !config.draw_cached_paths {
                if path.len() > 2 {
                    continue;
                }
            }

            // Iterate over path.path() drawing a line from one point to the next point until completed
            let mut iter = path.path().iter();
            let mut prev = iter.next().unwrap();

            for next in iter {
                let prev_position = match config.map_type {
                    MapType::Square | MapType::SquareCentered => Vec2::new(
                        (prev.x * config.grid_width) as f32,
                        (prev.y * config.grid_height) as f32,
                    ),
                    MapType::Isometric => Vec2::new(
                        (prev.y as f32 + prev.x as f32) * (config.grid_width as f32 * 0.5),
                        (prev.y as f32 - prev.x as f32) * (config.grid_height as f32 * 0.5),
                    ),
                };

                let next_position = match config.map_type {
                    MapType::Square | MapType::SquareCentered => Vec2::new(
                        (next.x * config.grid_width) as f32,
                        (next.y * config.grid_height) as f32,
                    ),
                    MapType::Isometric => Vec2::new(
                        (next.y as f32 + next.x as f32) * (config.grid_width as f32 * 0.5),
                        (next.y as f32 - next.x as f32) * (config.grid_height as f32 * 0.5),
                    ),
                };

                let mut color = css::BLUE;

                if config.draw_entrances && config.draw_cached_paths {
                    color = path_colors[color_index % path_colors.len()];
                }

                gizmos.line_2d(prev_position - offset, next_position - offset, color);

                prev = next;
            }

            color_index += 1;
        }
    }
}
