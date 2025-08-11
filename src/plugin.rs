//! Northstar Plugin. This plugin handles the pathfinding and collision avoidance systems.
use std::collections::VecDeque;
#[cfg(feature = "stats")]
use std::time::Instant;

use bevy::{log, platform::collections::HashMap, prelude::*};

use crate::{prelude::*, WithoutPathingFailures};

/// General settings for the Northstar plugin.
#[derive(Resource, Debug, Copy, Clone)]
pub struct NorthstarPluginSettings {
    /// The maximum number of agents that can be processed per frame.
    /// This is useful to stagger the pathfinding and collision avoidance systems
    /// to prevent stutters when too many agents are pathfinding at once.
    pub max_pathfinding_agents_per_frame: usize,
    /// The maximum number of agents that can be processed for collision avoidance per frame.
    pub max_collision_avoidance_agents_per_frame: usize,
}

impl Default for NorthstarPluginSettings {
    fn default() -> Self {
        Self {
            max_pathfinding_agents_per_frame: 128,
            max_collision_avoidance_agents_per_frame: 128,
        }
    }
}

/// NorthstarPlugin is the main plugin for the Northstar pathfinding and collision avoidance systems.
///
#[derive(Default)]
pub struct NorthstarPlugin<N: Neighborhood> {
    _neighborhood: std::marker::PhantomData<N>,
}

/// Tracks the average time the pathfinding algorithm takes.
#[derive(Default, Debug)]
pub struct PathfindingStats {
    /// The average time taken for pathfinding in seconds.
    pub average_time: f64,
    /// The average length of the path found.
    pub average_length: f64,
    /// A `Vec` list of all the pathfinding times.
    pub pathfind_time: Vec<f64>,
    /// A `Vec` list of all the pathfinding lengths.
    pub pathfind_length: Vec<f64>,
}

/// Tracks the average time the collision avoidance algorithms take.
#[derive(Default, Debug)]
pub struct CollisionStats {
    /// The average time taken for collision avoidance in seconds.
    pub average_time: f64,
    /// The average length of the path found after collision avoidance.
    pub average_length: f64,
    /// A `Vec` list of all the collision avoidance times.
    pub avoidance_time: Vec<f64>,
    /// A `Vec` list of all the collision avoidance lengths.
    pub avoidance_length: Vec<f64>,
}

/// The `Stats` `Resource` holds the pathfinding and collision avoidance statistics.
#[derive(Resource, Default, Debug)]
pub struct Stats {
    /// Pathfinding frame time statistics.
    pub pathfinding: PathfindingStats,
    /// Collision avoidance frame time statistics.
    pub collision: CollisionStats,
}

impl Stats {
    /// If you're manually pathfinding and want to keep track of the pathfinding statistics,
    /// you can use this method to add the time and length of the path found.
    pub fn add_pathfinding(&mut self, time: f64, length: f64) {
        self.pathfinding.pathfind_time.push(time);
        self.pathfinding.pathfind_length.push(length);

        self.pathfinding.average_time = self.pathfinding.pathfind_time.iter().sum::<f64>()
            / self.pathfinding.pathfind_time.len() as f64;
        self.pathfinding.average_length = self.pathfinding.pathfind_length.iter().sum::<f64>()
            / self.pathfinding.pathfind_length.len() as f64;
    }

    /// Resets the pathfinding statistics.
    pub fn reset_pathfinding(&mut self) {
        self.pathfinding.average_time = 0.0;
        self.pathfinding.average_length = 0.0;
        self.pathfinding.pathfind_time.clear();
        self.pathfinding.pathfind_length.clear();
    }

    /// If you're manually pathfinding and want to keep track of the collision avoidance statistics,
    /// you can use this method to add the time and length of the path found after collision avoidance.
    pub fn add_collision(&mut self, time: f64, length: f64) {
        self.collision.avoidance_time.push(time);
        self.collision.avoidance_length.push(length);

        self.collision.average_time = self.collision.avoidance_time.iter().sum::<f64>()
            / self.collision.avoidance_time.len() as f64;
        self.collision.average_length = self.collision.avoidance_length.iter().sum::<f64>()
            / self.collision.avoidance_length.len() as f64;
    }

    /// Resets the collision statistics.
    pub fn reset_collision(&mut self) {
        self.collision.average_time = 0.0;
        self.collision.average_length = 0.0;
        self.collision.avoidance_time.clear();
        self.collision.avoidance_length.clear();
    }
}

impl<N: 'static + Neighborhood> Plugin for NorthstarPlugin<N> {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                tag_pathfinding_requests,
                update_blocking_map,
                pathfind::<N>,
                next_position::<N>,
                reroute_path::<N>,
            )
                .chain()
                .in_set(PathingSet),
        )
        .insert_resource(NorthstarPluginSettings::default())
        .insert_resource(BlockingMap::default())
        .insert_resource(Stats::default())
        .insert_resource(DirectionMap::default())
        .register_type::<Path>()
        .register_type::<Pathfind>()
        .register_type::<PathfindMode>()
        .register_type::<NextPos>()
        .register_type::<AgentOfGrid>()
        .register_type::<GridAgents>();
    }
}

/// The `PathingSet` is a system set that is used to group the pathfinding systems together.
/// You can use this set to schedule systems before or after the pathfinding systems.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PathingSet;

/// The `BlockingMap` `Resource` contains a map of positions of entities holding the `Blocking` component.
/// The map is rebuilt every frame at the beginning of the `PathingSet`.
#[derive(Resource, Default)]
pub struct BlockingMap(pub HashMap<UVec3, Entity>);

/// The `DirectionMap` `Resource` contains a map of every pathfinding entity's last moved direction.
/// This is mainly used for collision avoidance but could be used for other purposes.
#[derive(Resource, Default)]
pub struct DirectionMap(pub HashMap<Entity, Vec3>);

#[derive(Component)]
#[component(storage = "SparseSet")]
pub(crate) struct NeedsPathfinding;

// Flags all the entities with a changed `Pathfind` component to request pathfinding.
fn tag_pathfinding_requests(mut commands: Commands, query: Query<Entity, Changed<Pathfind>>) {
    for entity in query.iter() {
        commands.entity(entity).try_insert(NeedsPathfinding);
    }
}

// The main pathfinding system. Queries for entities with the a changed `Pathfind` component.
// It will pathfind to the goal position and insert a `Path` component with the path found.
fn pathfind<N: Neighborhood + 'static>(
    grid: Single<&Grid<N>>,
    mut commands: Commands,
    query: Query<(Entity, &AgentPos, &Pathfind), With<NeedsPathfinding>>,
    blocking: Res<BlockingMap>,
    settings: Res<NorthstarPluginSettings>,
    //mut queue: Local<VecDeque<Entity>>,
    #[cfg(feature = "stats")] mut stats: ResMut<Stats>,
) {
    let grid = grid.into_inner();

    // Limit the number of agents processed per frame to prevent stutters
    let mut count = 0;

    for (entity, start, pathfind) in &query {
        if count >= settings.max_pathfinding_agents_per_frame {
            return;
        }

        if start.0 == pathfind.goal {
            commands.entity(entity).remove::<Pathfind>();
            commands.entity(entity).remove::<NeedsPathfinding>();
            continue;
        }

        #[cfg(feature = "stats")]
        let start_time = Instant::now();

        let blocking = if grid.collision() {
            &blocking.0
        } else {
            &HashMap::new()
        };

        let path = match pathfind.mode {
            PathfindMode::Refined => {
                grid.pathfind(start.0, pathfind.goal, blocking, pathfind.partial)
            }
            PathfindMode::Coarse => {
                grid.pathfind_coarse(start.0, pathfind.goal, blocking, pathfind.partial)
            }
            PathfindMode::AStar => {
                grid.pathfind_astar(start.0, pathfind.goal, blocking, pathfind.partial)
            }
        };

        #[cfg(feature = "stats")]
        let elapsed_time = start_time.elapsed().as_secs_f64();

        if let Some(path) = path {
            #[cfg(feature = "stats")]
            stats.add_pathfinding(elapsed_time, path.cost() as f64);

            commands
                .entity(entity)
                .try_insert(path)
                .try_remove::<PathfindingFailed>()
                .try_remove::<NeedsPathfinding>();
            // We remove PathfindingFailed even if it's not there.
        } else {
            #[cfg(feature = "stats")]
            stats.add_pathfinding(elapsed_time, 0.0);

            commands
                .entity(entity)
                .try_insert(PathfindingFailed)
                .try_remove::<NeedsPathfinding>()
                .try_remove::<NextPos>(); // Just to be safe
        }

        count += 1;
    }
}

// The `next_position` system is responsible for popping the front of the path into a `NextPos` component.
// If collision is enabled it will check for nearyby blocked paths and reroute the path if necessary.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
fn next_position<N: Neighborhood + 'static>(
    mut query: Query<
        (Entity, &mut Path, &AgentPos, &Pathfind),
        (WithoutPathingFailures, Without<NextPos>),
    >,
    grid: Single<&Grid<N>>,
    mut blocking: ResMut<BlockingMap>,
    mut direction: ResMut<DirectionMap>,
    mut commands: Commands,
    settings: Res<NorthstarPluginSettings>,
    mut queue: Local<VecDeque<Entity>>,
    #[cfg(feature = "stats")] mut stats: ResMut<Stats>,
) {
    let grid = grid.into_inner();

    // Initialize the queue with all candidates once
    if queue.is_empty() {
        for (entity, ..) in query.iter() {
            queue.push_back(entity);
        }
    }

    let mut processed = 0;

    for _ in 0..queue.len() {
        if processed >= settings.max_collision_avoidance_agents_per_frame {
            break;
        }

        let entity = queue.pop_front().unwrap();

        // If the entity still exists and is valid
        if let Ok((entity, mut path, position, pathfind)) = query.get_mut(entity) {
            if position.0 == pathfind.goal {
                commands.entity(entity).try_remove::<Path>();
                commands.entity(entity).try_remove::<Pathfind>();
                continue;
            }

            let next = if grid.collision() {
                #[cfg(feature = "stats")]
                let start = Instant::now();

                let success = avoidance(
                    grid,
                    entity,
                    &mut path,
                    pathfind,
                    position.0,
                    &blocking.0,
                    &direction.0,
                    grid.avoidance_distance() as usize,
                );

                if !success {
                    commands.entity(entity).try_insert(AvoidanceFailed);
                    continue;
                }

                #[cfg(feature = "stats")]
                let elapsed = start.elapsed().as_secs_f64();
                #[cfg(feature = "stats")]
                stats.add_collision(elapsed, path.cost() as f64);

                processed += 1;
                path.pop()
            } else {
                path.pop()
            };

            if let Some(next) = next {
                direction
                    .0
                    .insert(entity, next.as_vec3() - position.0.as_vec3());

                if blocking.0.contains_key(&next) && grid.collision() {
                    // Someone beat us to it - requeue without inserting NextPos
                    queue.push_back(entity);
                    continue;
                }

                blocking.0.remove(&position.0);
                blocking.0.insert(next, entity);
                commands.entity(entity).try_insert(NextPos(next));

                // Re-queue for next frame
                queue.push_back(entity);
            }
        }
    }
}

// The `avoidance` function does a lookahead on the path, if any blocking entities are found
// that are moving in the opposite relateive direciton it will attempt a short astar reroute.
#[allow(clippy::too_many_arguments)]
fn avoidance<N: Neighborhood + 'static>(
    grid: &Grid<N>,
    entity: Entity,
    path: &mut Path,
    pathfind: &Pathfind,
    position: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    direction: &HashMap<Entity, Vec3>,
    avoidance_distance: usize,
) -> bool {
    if path.is_empty() {
        log::error!("Path is empty for entity: {:?}", entity);
        return false;
    }

    // Check if the next few positions are blocked
    let count = if path.path().len() > avoidance_distance {
        avoidance_distance
    } else {
        path.path().len()
    };

    // Precompute the dot product for the current position
    let next_position = path.path.front().unwrap();

    if path.path.len() == 1 && blocking.contains_key(next_position) {
        return false;
    }

    let difference = next_position.as_vec3() - position.as_vec3();

    let unblocked_pos: Vec<UVec3> = path
        .path
        .iter()
        .take(count)
        .filter(|pos| {
            if let Some(blocking_entity) = blocking.get(*pos) {
                // If the blocking entity is the same as the current entity, skip it
                if *blocking_entity == entity {
                    return true;
                }

                // Too risky to move to the next position regardless of the direction
                if *pos == next_position {
                    return false;
                }

                if let Some(blocking_dir) = direction.get(blocking_entity) {
                    let dot = difference.dot(*blocking_dir);
                    if dot <= 0.0 {
                        return false;
                    }
                } else {
                    // They have no direction
                    return false;
                }
            }
            true
        })
        .cloned()
        .collect();

    // If we have a blocked position in the path, repath
    if unblocked_pos.len() < count {
        let avoidance_goal = {
            let _ = info_span!("avoidance_goal", name = "avoidance_goal").entered();

            // Get the first unlocked position AFTER skipping the count, we don't care about the direction
            path.path
                .iter()
                .skip(count)
                .find(|pos| !blocking.contains_key(&**pos))
        };

        // If we have an avoidance goal, astar path to that
        if let Some(avoidance_goal) = avoidance_goal {
            // Calculate a good radius from the current position to the avoidance goal
            let radius = position.as_vec3().distance(avoidance_goal.as_vec3()) as u32
                + grid.avoidance_distance();

            //let new_path = grid.pathfind_astar(position, *avoidance_goal, blocking, false);
            let new_path =
                grid.pathfind_astar_radius(position, *avoidance_goal, radius, blocking, false);

            // Replace the first few positions of path until the avoidance goal
            if let Some(new_path) = new_path {
                // Get every position AFTER the avoidance goal in the old path
                let old_path = path
                    .path
                    .iter()
                    .skip_while(|pos| *pos != avoidance_goal)
                    .cloned()
                    .collect::<Vec<UVec3>>();

                // Combine the new path with the old path
                let mut combined_path = new_path.path().to_vec();
                combined_path.extend(old_path);

                if combined_path.is_empty() {
                    log::error!("Combined path is empty for entity: {:?}", entity);
                    return false;
                }

                let graph_path = path.graph_path.clone();

                // Replace the path with the combined path
                *path = Path::from_slice(&combined_path, new_path.cost());
                path.graph_path = graph_path;
            } else {
                return false;
            }
        } else {
            // No easy avoidance astar path to the next entrance
            return false;
        }
    }

    // DELETE THIS
    // We must have gotten to this point because of partial paths
    if path.path.is_empty() {
        // if goal is in blocking
        if blocking.contains_key(&pathfind.goal) {
            return false;
        }

        let new_path = grid.pathfind(position, pathfind.goal, blocking, false);

        if let Some(new_path) = new_path {
            *path = new_path;
            return true;
        } else {
            return false;
        }
    }

    true
}

// The `reroute_path` system is responsible for rerouting the path if the `AvoidanceFailed` component is present.
// It will attempt to find a new full path to the goal position and insert it into the entity.
// If the reroute fails, it will insert a `RerouteFailed` component to the entity.
// Once an entity has a reroute failure, no pathfinding will be attempted until the user handles reinserts the `Pathfind` component.
fn reroute_path<N: Neighborhood + 'static>(
    mut query: Query<(Entity, &AgentPos, &Pathfind, &Path), With<AvoidanceFailed>>,
    grid: Single<&Grid<N>>,
    blocking: Res<BlockingMap>,
    mut commands: Commands,
    settings: Res<NorthstarPluginSettings>,
    #[cfg(feature = "stats")] mut stats: ResMut<Stats>,
) {
    let grid = grid.into_inner();

    for (count, (entity, position, pathfind, path)) in query.iter_mut().enumerate() {
        // TODO: This doesn't really tie in with the main pathfinding agent counts. This will stil help limit how many are rereouting for now.
        // There's no point bridging it for the moment since this really needs to be reworked into an async system to really prevent stutters.
        if count >= settings.max_pathfinding_agents_per_frame {
            return;
        }

        #[cfg(feature = "stats")]
        let start = Instant::now();

        // Let's reroute the path
        let refined = match pathfind.mode {
            PathfindMode::Refined => true,
            PathfindMode::Coarse => false,
            PathfindMode::AStar => false, // A* is not supported for rerouting
        };

        let new_path = grid.reroute_path(path, position.0, pathfind.goal, &blocking.0, refined);

        if let Some(new_path) = new_path {
            // if the last position in the path is not the goal...
            if new_path.path().last().unwrap() != &pathfind.goal {
                log::error!("WE HAVE A PARTIAL ROUTE ISSUE: {:?}", entity);
            }

            #[cfg(feature = "stats")]
            let elapsed = start.elapsed().as_secs_f64();
            #[cfg(feature = "stats")]
            stats.add_collision(elapsed, new_path.cost() as f64);

            commands.entity(entity).try_insert(new_path);
            commands.entity(entity).try_remove::<AvoidanceFailed>();
        } else {
            commands.entity(entity).try_insert(RerouteFailed);
            commands.entity(entity).try_remove::<AvoidanceFailed>(); // Try again next frame

            #[cfg(feature = "stats")]
            let elapsed = start.elapsed().as_secs_f64();
            #[cfg(feature = "stats")]
            stats.add_collision(elapsed, 0.0);
        }
    }
}

fn update_blocking_map(
    mut blocking_set: ResMut<BlockingMap>,
    query: Query<(Entity, &AgentPos), With<Blocking>>,
) {
    blocking_set.0.clear();

    query.iter().for_each(|(entity, position)| {
        blocking_set.0.insert(position.0, entity);
    });
}
