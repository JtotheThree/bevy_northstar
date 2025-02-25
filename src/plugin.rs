use crate::{dir::Dir, prelude::*};
use bevy::{log, prelude::*, utils::hashbrown::HashMap};

#[derive(Default)]
pub struct NorthstarPlugin<N: Neighborhood> {
    _neighborhood: std::marker::PhantomData<N>,
}

#[derive(Resource, Default)]
pub struct NorthstarSettings {
    pub collision: bool,
    pub avoidance_distance: usize,
}

impl<N: 'static + Neighborhood> Plugin for NorthstarPlugin<N> {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                update_blocking_map,
                clear_goals,
                pathfind::<N>,
                next_position::<N>,
            )
            .chain()
            .in_set(PathingSet),
        )
        .insert_resource(BlockingMap::default());
    }
}

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PathingSet;

#[derive(Resource, Default)]
pub struct BlockingMap(pub HashMap<UVec3, Entity>);


fn pathfind<N: Neighborhood>(
    grid: Res<Grid<N>>,
    mut commands: Commands,
    mut query: Query<(Entity, &Name, &Position, &Goal), Changed<Goal>>,
    blocking: Res<BlockingMap>,
) where
    N: 'static + Neighborhood,
{
    query.iter_mut().for_each(|(entity, name, start, goal)| {
        if start.0 == goal.0 {
            return;
        }

        let path = grid.get_path(start.0, goal.0, &blocking.0, false);

        //log::info!("Pathfinding for entity: {:?}", name);

        if let Some(path) = path {
            commands.entity(entity).insert(path);
        } else {
            //log::error!("Pathfinding failed for {:?}: no path found, goal: {:?}, blocking: {:?}", name, goal.0, blocking.0);
            commands.entity(entity).remove::<Next>(); // Just to be safe
            commands.entity(entity).insert(Goal(goal.0)); // Don't let anyone get stuck try again next frame
        }
    });
}

/*fn planning(
    mut commands: Commands,
    mut query: Query<(Entity, &Path, &Position), Without<Next>>,
) {
    for (entity, path, position) in query.iter_mut() {
        if path.is_empty() {
            commands.entity(entity).remove::<Path>();
            continue;
        }

        let next = path.path.front().unwrap().clone();

        // Determine the direction based on the current position to next
        let difference = next.as_vec3() - position.0.as_vec3();
        let normalized = difference.signum();

        if normalized == Vec3::ZERO {
            //log::error!("Normalized vector is zero");
            continue;
        }

        let dir = Dir::from_vec3(&normalized);

        commands.entity(entity).insert(Planned {
            next,
            dir: Some(dir),
        });
    }
}*/

fn clear_goals(
    mut commands: Commands,
    query: Query<(Entity, &Position, &Goal)>,
) {
    for (entity, position, goal) in query.iter() {
        if position.0 == goal.0 {
            commands.entity(entity).remove::<Path>();
            commands.entity(entity).remove::<Goal>();
        }
    }
}

fn next_position<N: Neighborhood>(
    mut commands: Commands,
    grid: Res<Grid<N>>,
    mut query: Query<(Entity, &Name, &mut Path, &Position, &Goal), Without<Next>>,
    mut blocking: ResMut<BlockingMap>,
    settings: Res<NorthstarSettings>,
) where
    N: 'static + Neighborhood,
{
    for (entity, name, mut path, position, goal) in query.iter_mut() {
        // If we're at the goal, we're done
        if position.0 == goal.0 {
            commands.entity(entity).remove::<Path>();
            commands.entity(entity).remove::<Goal>();
            continue;
        }

        // Handle avoidance if we have collision enabled
        if settings.collision {
            // Check if the next few positions are blocked
            let count = if path.path().len() > settings.avoidance_distance {
                settings.avoidance_distance
            } else {
                path.path().len()
            };

            let unblocked_pos: Vec<UVec3> = path
                .path
                .iter()
                .take(count)
                .filter(|pos| blocking.0.contains_key(&**pos) == false)
                .cloned()
                .collect();

            // If we have a blocked position in the path, repath
            if unblocked_pos.len() < count {
                // Get the first unlocked position AFTER skipping the count
                let avoidance_goal = path
                    .path
                    .iter()
                    .skip(count)
                    .find(|pos| blocking.0.contains_key(&**pos) == false)
                    .cloned();
            
                // If we have an avoidance goal, astar path to that
                if let Some(avoidance_goal) = avoidance_goal {
                    let new_path = grid.get_astar_path(position.0, avoidance_goal, &blocking.0, false);

                    // Replace the first few positions of path until the avoidance goal
                    if let Some(new_path) = new_path {
                        // Debug, check if any positions in the new path are blocked
                        /*let blocked_pos: Vec<UVec3> = new_path
                            .path
                            .iter()
                            .filter(|pos| blocking.0.contains_key(&**pos))
                            .cloned()
                            .collect();

                        if blocked_pos.len() > 0 {
                            log::error!("Blocked path: {:?}", blocked_pos);
                        }*/

                        //log::info!("We found a SHORT avoidance path!");

                        // Get every position AFTER the avoidance goal in the old path
                        let old_path = path
                            .path
                            .iter()
                            .skip_while(|pos| *pos != &avoidance_goal)
                            .cloned()
                            .collect::<Vec<UVec3>>();

                        // Combine the new path with the old path
                        let mut combined_path = new_path.path().to_vec();
                        combined_path.extend(old_path);

                        if combined_path.len() == 0 {
                            log::error!("Combined path is empty for entity: {:?}", name);
                            continue;
                        }

                        // Replace the path with the combined path
                        *path = Path::from_slice(&combined_path, new_path.cost());
                    } else {
                        // If we can't avoid locally we need to repath the whole thing
                        //log::info!("Can't find a path to the avoidance goal, trying to repath the whole thing...");

                        let new_path = grid.get_astar_path(position.0, goal.0, &blocking.0, true);
                        if let Some(new_path) = new_path {
                            //log::info!("We found a new full path!");
                            *path = new_path;
                        }  else {
                            //log::error!("SECOND repathing failed for {:?}: no path found, avoidance_goal: {:?}", name, avoidance_goal);
                            continue;
                        }
                    }
                } else {
                    // ummm.. try to astar path to goal?
                    //log::info!("No avoidance goal found, trying to repath the whole thing...");
                    let new_path = grid.get_astar_path(position.0, goal.0, &blocking.0, true);
                    if let Some(new_path) = new_path {
                        //log::info!("We found a new full path!");
                        *path = new_path;
                    } else {
                        //log::error!("Full Repathing failed for {:?}: no path found, goal: {:?}", name, goal.0);
                        continue;
                    }
                }
            } 
            /*else {
                log::info!("Nothing blocked, just moving on!");
            }*/
        }

        // We must have gotten to this point because of partial paths
        if path.path.is_empty() {
            let new_path = grid.get_path(position.0, goal.0, &blocking.0, false);
            if let Some(new_path) = new_path {
                *path = new_path;
            } else {
                //log::error!("FINAL FINAL pathing failed for {:?}: no path found, goal: {:?}", name, goal.0);
                continue;
            }
        }

        /*let potential_next = path.path.front().unwrap();

        if blocking.0.contains_key(potential_next) {
            log::error!("THE NEXT FUCKING POSITION IS BLOCKED, WHY???? {:?}", potential_next);
            continue;
        }*/

        let next = path.pop();

        if let Some(next) = next {
            blocking.0.remove(&position.0);
            blocking.0.insert(next, entity);
            commands.entity(entity).insert(Next(next));
        } else {
            log::error!("No next position found for entity: {:?}", name);
        }
    }
}

fn next_position_2<N: Neighborhood>(
    mut commands: Commands,
    grid: Res<Grid<N>>,
    mut query: Query<(Entity, &Name, &mut Path, &Position, &Goal), Without<Next>>,
    mut blocking: ResMut<BlockingMap>,
) where
    N: 'static + Neighborhood,
{
    for (entity, name, mut path, position, goal) in query.iter_mut() {
        if position.0 == goal.0 {
            commands.entity(entity).remove::<Position>();
            commands.entity(entity).remove::<Goal>();
            continue;
        }

        let count = if path.path().len() > 4 {
            4
        } else {
            path.path().len()
        };

        let unblocked_pos: Vec<UVec3> = path
            .path
            .iter()
            .take(count)
            .filter(|pos| blocking.0.contains_key(&**pos) == false)
            .cloned()
            .collect();

        if unblocked_pos.len() < count {
            // We have a blocking entity in the path
            log::info!("Repathing...");
            let new_path = grid.get_path(position.0, goal.0, &blocking.0, false);
            if let Some(new_path) = new_path {
                // Print which path is blocked
                /*let blocked_pos: Vec<UVec3> = path
                    .path
                    .iter()
                    .take(count)
                    .filter(|pos| blocking.0.contains_key(&**pos))
                    .cloned()
                    .collect();

                log::info!("Blocked path: {:?}", blocked_pos);*/
                *path = new_path
            } else {
                //log::error!("Repathing failed for {:?}: no path found, goal: {:?}, blocking: {:?}", name, goal.0, blocking.0);
                continue;
            };
        }

        let potential_next = path.path.front().unwrap();

        if blocking.0.contains_key(potential_next) {
            log::error!("THE NEXT FUCKING POSITION IS BLOCKED, WHY???? {:?}", potential_next);
            continue;
        }

        let next = path.pop();

        if let Some(next) = next {
            blocking.0.remove(&position.0);
            blocking.0.insert(next, entity);
            commands.entity(entity).insert(Next(next));
        } else {
            log::error!("No next position found for entity: {:?}", name);
        }
    }
}

fn update_blocking_map(
    mut blocking_set: ResMut<BlockingMap>,
    query: Query<(Entity, &Position)> // WHY DID I HAVE THIS Changed<Position>>,
) {
    blocking_set.0.clear();

    query.iter().for_each(|(entity, position)| {
        blocking_set.0.insert(position.0, entity);
    });
}
