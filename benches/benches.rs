use criterion::{criterion_group, criterion_main, Criterion};
use bevy_northstar::Pathfinding;
use bevy::math::Vec3;

const N: usize = 640;

/*fn corner_to_corner_64() {
    let mut pathfinding = Pathfinding::default();

    for x in 0..N {
        for y in 0..N {
            pathfinding.add_vec(Vec3::new(x as f32, y as f32, 0.0), 0.0);
        }
    }

    for x in 0..N {
        for y in 0..N {
            if x > 0 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32 - 1.0, y as f32, 0.), 
                    false
                ).unwrap();
            }
            if x < 63 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32 + 1.0, y as f32, 0.), 
                    false
                ).unwrap();
            }
            if y > 0 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32, y as f32 - 1.0, 0.), 
                    false 
                ).unwrap();
            }
            if y < 63 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32, y as f32 + 1.0, 0.), 
                    false
                ).unwrap();
            }
        }
    }

    let _ = pathfinding.get_point_path(
        Vec3::new(0., 0., 0.),
        Vec3::new(63., 63., 0.),
        false
    );
}*/

fn corner_to_corner_64_without_setup(pathfinding: &mut Pathfinding) {
    let _ = pathfinding.get_path_by_vec(
        Vec3::new(0., 0., 0.),
        Vec3::new(N as f32 - 1.0, N as f32 -1.0, 0.),
        false
    );    
}


fn benchmarks(c: &mut Criterion) {
    let mut pathfinding = Pathfinding::default();

    for x in 0..N {
        for y in 0..N {
            pathfinding.add_vec(Vec3::new(x as f32, y as f32, 0.0), 0.0);
        }
    }

    for x in 0..N {
        for y in 0..N {
            if x > 0 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32 - 1.0, y as f32, 0.), 
                    false
                ).unwrap();
            }
            if x < N - 1 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32 + 1.0, y as f32, 0.), 
                    false
                ).unwrap();
            }
            if y > 0 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32, y as f32 - 1.0, 0.), 
                    false 
                ).unwrap();
            }
            if y < N - 1 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32, y as f32 + 1.0, 0.), 
                    false
                ).unwrap();
            }
        }
    } 

    let mut group = c.benchmark_group("pathfinding");

    println!("Done with setup.");

    group.sample_size(10);
    //group.bench_function("corner_to_corner_64", |b| b.iter(|| corner_to_corner_64() ));
    group.bench_function("without_setup", |b| b.iter(|| corner_to_corner_64_without_setup(&mut pathfinding.clone())));
    group.finish();
}

criterion_group!(benches, benchmarks);
criterion_main!(benches);