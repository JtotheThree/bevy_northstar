use bevy::{log::{self, LogPlugin}, prelude::*};
use bevy_northstar::{nav::NavCell, prelude::*};

#[derive(Resource)]
struct MyNavMask(NavMask);

fn main() {
    App::new()
        .add_plugins((MinimalPlugins, LogPlugin::default()))
        .add_systems(Startup, setup)
        .add_systems(Startup, demo_modify_cost.after(setup))
        .add_systems(Startup, demo_no_go.after(setup))
        .insert_resource(MyNavMask(NavMask::new()))
        .run();
}

fn setup(nav_mask: ResMut<MyNavMask>) {
    let cost_layer = NavMaskLayer::new();

    cost_layer.insert_region(
        Region3d::new(UVec3::new(0, 0, 0), UVec3::new(10, 10, 10)),
        NavCellMask::ModifyCost(5),
    ).unwrap();

    let no_go_layer = NavMaskLayer::new();
    no_go_layer.insert_region(
        Region3d::new(UVec3::new(11, 11, 11), UVec3::new(15, 15, 15)),
        NavCellMask::ImpassableOverride,
    ).unwrap();

    nav_mask.0.add_layer(cost_layer).unwrap();
    nav_mask.0.add_layer(no_go_layer).unwrap();
}

fn demo_modify_cost(nav_mask: Res<MyNavMask>) {
    let pos = UVec3::new(5, 5, 5);

    // NavCell defaults to Passable with a cost of 1.
    let original = NavCell::default();

    let masked = nav_mask.0.get(original.clone(), pos).unwrap();

    assert_eq!(masked.nav(), Nav::Passable(6)); // Original cost 1 + mask cost 5

    log::info!("Modify Cost - Original: {:?}, Masked: {:?}", original, masked);
}

fn demo_no_go(nav_mask: Res<MyNavMask>) {
    let pos = UVec3::new(12, 12, 12);

    // NavCell defaults to Passable with a cost of 1.
    let original = NavCell::default();

    let masked = nav_mask.0.get(original.clone(), pos).unwrap();

    assert_eq!(masked.nav(), Nav::Impassable); // No-go region overrides to Impassable

    log::info!("No Go - Original: {:?}, Masked: {:?}", original, masked);
}