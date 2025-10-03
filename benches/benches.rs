use bevy::math::UVec3;
use criterion::{criterion_group, criterion_main, Criterion};

use bevy_northstar::{
    filter,
    grid::{Grid, GridSettings, GridSettingsBuilder},
    nav::Nav,
    nav_mask::{NavCellMask, NavMask, NavMaskLayer},
    pathfind::PathfindArgs,
    prelude::OrdinalNeighborhood,
    NavRegion,
};

use pprof::criterion::{Output, PProfProfiler};

fn setup_grid(map: tiled::Map, grid_settings: GridSettings) -> Grid<OrdinalNeighborhood> {
    let mut grid: Grid<OrdinalNeighborhood> = Grid::new(&grid_settings);

    for layer in map.layers() {
        if let Some(tiled_layer) = layer.as_tile_layer() {
            for y in 0..128 {
                for x in 0..128 {
                    let tile = tiled_layer.get_tile(x, y);
                    if let Some(tile) = tile {
                        // Let's make tiles with an id of 1 impassable
                        if tile.id() == 14 {
                            grid.set_nav(UVec3::new(x as u32, y as u32, 0), Nav::Passable(1));
                        }
                    }
                }
            }
        }
    }

    grid
}

fn benchmarks(c: &mut Criterion) {
    // Let's load the demo map so we can use it to get some actual real world benchmarks
    let mut loader = tiled::Loader::new();
    let map = loader.load_tmx_map("assets/demo_128.tmx").unwrap();

    let grid_settings = GridSettingsBuilder::new_2d(128, 128)
        .chunk_size(8)
        .default_impassable()
        .build();
    let mut grid = setup_grid(map.clone(), grid_settings);

    let mut group = c.benchmark_group("build");

    group.sample_size(10);

    group.bench_function("build_grid_128x128_tilemap", |b| b.iter(|| grid.build()));

    group.finish();

    /* BENCH EACH ALGORITHM ON THE 128x128 Demo Map */
    let mut group = c.benchmark_group("pathfinding");

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0));
    group.bench_function("hpa_refined_128x128_tilemap", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0)).coarse();
    group.bench_function("hpa_coarse_128x128_tilemap", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0)).astar();
    group.bench_function("astar_128x128_tilemap", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0)).waypoints();
    group.bench_function("hpa_waypoints_128x128_tilemap", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0)).thetastar();
    group.bench_function("thetastar_128x128_tilemap", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    group.finish();

    /* BENCH NEIGHBOR FILTERS */
    let mut group = c.benchmark_group("filters");

    let grid_settings = GridSettingsBuilder::new_2d(128, 128)
        .chunk_size(8)
        .add_neighbor_filter(filter::NoCornerCutting)
        .default_impassable()
        .build();
    let mut grid = setup_grid(map.clone(), grid_settings);

    grid.build();

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0));
    group.bench_function("hpa_refined_128x128_tilemap_w_filter", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0)).coarse();
    group.bench_function("hpa_coarse_128x128_tilemap_w_filter", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0)).astar();
    group.bench_function("astar_128x128_tilemap_w_filter", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    group.finish();

    /* BENCH NAVMASK */
    let mut group = c.benchmark_group("navmask");

    let grid_settings = GridSettingsBuilder::new_2d(128, 128)
        .chunk_size(8)
        .default_impassable()
        .build();
    let mut grid = setup_grid(map.clone(), grid_settings);

    grid.build();

    let mut mask = NavMask::new();
    let layer = NavMaskLayer::new();
    layer
        .insert_region_fill(
            &grid,
            NavRegion::new(UVec3::new(60, 0, 0), UVec3::new(80, 127, 0)),
            NavCellMask::ModifyCost(5),
        )
        .unwrap();
    mask.add_layer(layer).unwrap();

    let mut request =
        PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0)).mask(&mut mask);
    group.bench_function("hpa_refined_128x128_tilemap_w_mask", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0))
        .coarse()
        .mask(&mut mask);
    group.bench_function("hpa_coarse_128x128_tilemap_w_mask", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0))
        .astar()
        .mask(&mut mask);
    group.bench_function("astar_128x128_tilemap_w_mask", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0))
        .waypoints()
        .mask(&mut mask);
    group.bench_function("hpa_waypoints_128x128_tilemap_w_mask", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    let mut request = PathfindArgs::new(UVec3::new(2, 3, 0), UVec3::new(115, 11, 0))
        .thetastar()
        .mask(&mut mask);
    group.bench_function("thetastar_128x128_tilemap_w_mask", |b| {
        b.iter(|| assert!(grid.pathfind(&mut request).is_some()))
    });

    group.finish();

    // This is causing issues with github actions.
    /*let mut group = c.benchmark_group("large_grids");

    let grid_settings_3d = GridSettingsBuilder::new_3d(512, 512, 32)
        .chunk_size(16)
        .chunk_depth(16)
        .build();
    let mut large_grid3d = Grid::<OrdinalNeighborhood3d>::new(&grid_settings_3d);

    group.bench_function("build_grid_512x512x32_empty", |b| {
        b.iter(|| large_grid3d.build())
    });

    group.finish();*/
}

criterion_group! {
    name = benches;
    config = Criterion::default().with_profiler(PProfProfiler::new(100, Output::Flamegraph(None))).sample_size(10);
    targets = benchmarks
}

criterion_main!(benches);
