use std::sync::{Arc, Mutex};

use bevy::{
    log::{self, LogPlugin},
    platform::collections::HashMap,
    prelude::*,
};
use bevy_northstar::{nav::NavCell, prelude::*};

#[derive(Resource, Debug, Default)]
struct MyNavMaskLayers(HashMap<String, Arc<Mutex<NavMaskLayer>>>);

#[derive(Resource, Debug, Default)]
struct MyNavMasks(HashMap<Entity, NavMask>);

#[derive(Resource, Debug, Default)]
struct MyAgents {
    red_faction: Option<Entity>,
    blue_faction: Option<Entity>,
}

fn main() {
    App::new()
        .add_plugins((MinimalPlugins, LogPlugin::default()))
        .add_systems(Startup, setup_agents)
        .add_systems(Startup, setup_layers)
        .add_systems(Startup, setup_masks.after(setup_agents))
        .add_systems(Startup, test_cells.after(setup_masks))
        .add_systems(Update, app_exit)
        .insert_resource(MyAgents::default())
        .insert_resource(MyNavMaskLayers::default())
        .insert_resource(MyNavMasks::default())
        .run();
}

fn setup_layers(mut layers: ResMut<MyNavMaskLayers>) {
    // Maybe different units will have penalties for crossing water tiles while other might not?
    let water_layer = NavMaskLayer::new();
    water_layer
        .insert_region(
            Region3d::new(UVec3::new(0, 0, 0), UVec3::new(10, 10, 10)),
            NavCellMask::ModifyCost(5),
        )
        .unwrap();

    // We don't want our red faction units to path through blue faction teritory.
    let red_faction_not_allowed_layer = NavMaskLayer::new();
    red_faction_not_allowed_layer
        .insert_region(
            Region3d::new(UVec3::new(11, 11, 11), UVec3::new(15, 15, 15)),
            NavCellMask::ImpassableOverride,
        )
        .unwrap();

    // We don't want our blue faction units to path through red faction teritory.
    let blue_faction_not_allowed_layer = NavMaskLayer::new();
    blue_faction_not_allowed_layer
        .insert_region(
            Region3d::new(UVec3::new(16, 16, 16), UVec3::new(20, 20, 20)),
            NavCellMask::ImpassableOverride,
        )
        .unwrap();

    layers
        .0
        .insert("water_layer".to_string(), Arc::new(Mutex::new(water_layer)));
    layers.0.insert(
        "red_faction_not_allowed_layer".to_string(),
        Arc::new(Mutex::new(red_faction_not_allowed_layer)),
    );
    layers.0.insert(
        "blue_faction_not_allowed_layer".to_string(),
        Arc::new(Mutex::new(blue_faction_not_allowed_layer)),
    );
}

// Setup some test entities.
fn setup_agents(mut commands: Commands, mut agents: ResMut<MyAgents>) {
    let red_faction_entity = commands.spawn_empty().id();
    let blue_faction_entity = commands.spawn_empty().id();

    agents.red_faction = Some(red_faction_entity);
    agents.blue_faction = Some(blue_faction_entity);
}

fn setup_masks(layers: Res<MyNavMaskLayers>, mut masks: ResMut<MyNavMasks>, agents: Res<MyAgents>) {
    let red_faction_entity = agents.red_faction.unwrap();
    let blue_faction_entity = agents.blue_faction.unwrap();

    let red_mask = NavMask::new();
    let blue_mask = NavMask::new();

    // Add the water nav mask layer to both factions
    // Maybe a potential green faction could pass through water without penalty?
    if let Some(layer) = layers.0.get("water_layer") {
        let layer_guard = layer.lock().unwrap();
        red_mask.add_layer(layer_guard.clone()).unwrap();
        blue_mask.add_layer(layer_guard.clone()).unwrap();
    }

    // Add the faction-specific layers
    if let Some(layer) = layers.0.get("red_faction_not_allowed_layer") {
        let layer_guard = layer.lock().unwrap();
        red_mask.add_layer(layer_guard.clone()).unwrap();
    }

    if let Some(layer) = layers.0.get("blue_faction_not_allowed_layer") {
        let layer_guard = layer.lock().unwrap();
        blue_mask.add_layer(layer_guard.clone()).unwrap();
    }

    masks.0.insert(red_faction_entity, red_mask);
    masks.0.insert(blue_faction_entity, blue_mask);
    log::info!("Masks set up for factions");
}

fn test_cells(masks: Res<MyNavMasks>, agents: Res<MyAgents>) {
    let red_faction_entity = agents.red_faction.unwrap();
    let red_faction_mask = masks.0.get(&red_faction_entity).unwrap();

    let blue_faction_entity = agents.blue_faction.unwrap();
    let blue_faction_mask = masks.0.get(&blue_faction_entity).unwrap();

    // Our default test cell we'll set to passable with a cost of 1.
    let grid_cell = NavCell::new(Nav::Passable(1));
    let water_pos = UVec3::new(5, 5, 5);
    let red_faction_pos = UVec3::new(17, 17, 17);
    let blue_faction_pos = UVec3::new(12, 12, 12);

    // Test a water cell
    let masked_water_cell = red_faction_mask.get(grid_cell.clone(), water_pos).unwrap();
    assert_eq!(masked_water_cell.nav(), Nav::Passable(6));
    log::info!(
        "Water Movement Cost - Original: {:?}, Masked: {:?}",
        grid_cell.nav(),
        masked_water_cell.nav()
    );

    // Test that the red faction agent cannot pass through the blue faction's no-go region
    let masked_red_faction_cell = red_faction_mask
        .get(grid_cell.clone(), blue_faction_pos)
        .unwrap();
    assert_eq!(masked_red_faction_cell.nav(), Nav::Impassable);
    log::info!(
        "Red Faction Agent to Blue Faction Area - Original: {:?}, Masked: {:?}",
        grid_cell.nav(),
        masked_red_faction_cell.nav()
    );

    // Test that the red faction agent can pass through its own territory
    let masked_red_faction_own_cell = red_faction_mask
        .get(grid_cell.clone(), red_faction_pos)
        .unwrap();
    assert_eq!(masked_red_faction_own_cell.nav(), Nav::Passable(1));
    log::info!(
        "Red Faction Agent to Red Faction Area - Original: {:?}, Masked: {:?}",
        grid_cell.nav(),
        masked_red_faction_own_cell.nav()
    );

    // Test that the blue faction agent cannot pass through the red faction's no-go region
    let masked_blue_faction_cell = blue_faction_mask
        .get(grid_cell.clone(), red_faction_pos)
        .unwrap();
    assert_eq!(masked_blue_faction_cell.nav(), Nav::Impassable);
    log::info!(
        "Blue Faction Agent to Red Faction Area - Original: {:?}, Masked: {:?}",
        grid_cell.nav(),
        masked_blue_faction_cell.nav()
    );
}

fn app_exit(mut exit: EventWriter<AppExit>) {
    exit.write(AppExit::Success);
}
