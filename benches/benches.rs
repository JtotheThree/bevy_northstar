use std::time::Duration;

use bevy::{math::UVec3, platform::collections::HashMap};
use criterion::{criterion_group, criterion_main, Criterion};

use bevy_northstar::{
    grid::{Grid, GridSettingsBuilder},
    prelude::{OrdinalNeighborhood, OrdinalNeighborhood3d},
};

mod profiler;

// TODO: These benches aren't great for for comparing HPA* performance to A* performance since A* is able to just drive straight to it's goals.
// They're sufficient for testing that changes don't add significant overhead, but it'd be nice to get more realistic benchmarks.
// We could probably use the demo map for this.

fn benchmarks(c: &mut Criterion) {
    let mut group = c.benchmark_group("pathfinding");

    let grid_settings = GridSettingsBuilder::new_2d(64, 64).chunk_size(32).build();

    let mut grid: Grid<OrdinalNeighborhood> = Grid::new(&grid_settings);

    group.sample_size(10);

    group.bench_function("build_grid_64x64", |b| b.iter(|| grid.build()));

    group.bench_function("pathfind_64x64", |b| {
        b.iter(|| {
            grid.pathfind(
                UVec3::new(0, 0, 0),
                UVec3::new(63, 63, 0),
                &HashMap::new(),
                false,
            )
        })
    });

    group.bench_function("raw_pathfind_64x64", |b| {
        b.iter(|| {
            grid.pathfind_astar(
                UVec3::new(0, 0, 0),
                UVec3::new(63, 63, 0),
                &HashMap::new(),
                false,
            )
        })
    });

    let grid_settings = GridSettingsBuilder::new_2d(512, 512).chunk_size(32).build();

    let mut grid: Grid<OrdinalNeighborhood> = Grid::new(&grid_settings);

    group.measurement_time(Duration::from_secs(10));
    group.bench_function("build_grid_512x512", |b| b.iter(|| grid.build()));

    group.bench_function("pathfind_512x512", |b| {
        b.iter(|| {
            grid.pathfind(
                UVec3::new(0, 0, 0),
                UVec3::new(511, 511, 0),
                &HashMap::new(),
                false,
            )
        })
    });

    group.measurement_time(Duration::from_secs(5));
    group.bench_function("raw_pathfind_512x512", |b| {
        b.iter(|| {
            grid.pathfind_astar(
                UVec3::new(0, 0, 0),
                UVec3::new(511, 511, 0),
                &HashMap::new(),
                false,
            )
        })
    });

    let grid_settings = GridSettingsBuilder::new_3d(128, 128, 4)
        .chunk_size(16)
        .build();

    let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

    group.measurement_time(Duration::from_secs(10));
    group.bench_function("build_grid_128x128x4", |b| b.iter(|| grid.build()));
    group.measurement_time(Duration::from_secs(5));

    group.bench_function("pathfind_128x128x4", |b| {
        b.iter(|| {
            grid.pathfind(
                UVec3::new(0, 0, 0),
                UVec3::new(127, 127, 3),
                &HashMap::new(),
                false,
            )
        })
    });

    group.bench_function("raw_pathfind_128x128x4", |b| {
        b.iter(|| {
            grid.pathfind_astar(
                UVec3::new(0, 0, 0),
                UVec3::new(127, 127, 3),
                &HashMap::new(),
                false,
            )
        })
    });

    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default().with_profiler(profiler::FlamegraphProfiler::new(100)).sample_size(10);
    targets = benchmarks
}

criterion_main!(benches);
