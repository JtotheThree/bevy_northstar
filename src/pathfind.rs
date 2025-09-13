//! This module defines pathfinding functions which can be called directly.

use bevy::{
    ecs::entity::Entity,
    log,
    math::UVec3,
    platform::collections::{HashMap, HashSet},
};
use ndarray::ArrayView3;

use crate::{
    astar::astar_grid,
    chunk::Chunk,
    components::PathfindMode,
    dijkstra::dijkstra_grid,
    grid::Grid,
    hpa::hpa,
    nav::NavCell,
    nav_mask::NavMaskData,
    neighbor::Neighborhood,
    node::Node,
    path::Path,
    prelude::{NavMask, Pathfind},
    raycast::bresenham_path_internal,
    thetastar::thetastar_grid,
    NavRegion, SearchLimits,
};

/// Builder struct for pathfinding arguments
#[derive(Debug)]
pub struct PathfindArgs<'a> {
    pub(crate) start: UVec3,
    pub(crate) goal: UVec3,
    pub(crate) blocking: Option<&'a HashMap<UVec3, Entity>>,
    pub(crate) mask: Option<&'a mut NavMask>,
    pub(crate) mode: PathfindMode,
    pub(crate) limits: SearchLimits,
}

impl<'a> PathfindArgs<'a> {
    /// Create a new pathfinding request
    pub fn new(start: UVec3, goal: UVec3) -> Self {
        Self {
            start,
            goal,
            blocking: None,
            mask: None,
            mode: PathfindMode::Refined,
            limits: SearchLimits::default(),
        }
    }

    /// Sets the pathfinding request to return the closest path if a full path to the goal isn't possible.
    pub fn partial(mut self) -> Self {
        self.limits.partial = true;
        self
    }

    /// Limits the search to a region for the pathfinding request.
    pub fn search_region(mut self, region: NavRegion) -> Self {
        self.limits.boundary = Some(region);
        self
    }

    /// Limits the path search to a maximum distance from the start.
    /// No path will be returned if the maximum distance is exceeded.
    pub fn max_distance(mut self, max_distance: u32) -> Self {
        self.limits.distance = Some(max_distance);
        self
    }

    /// Sets the [`PathfindMode`] algorithm for the pathfinding request.
    /// You can use this or the helper methods like astar(), coarse(), etc.
    pub fn mode(mut self, mode: PathfindMode) -> Self {
        self.mode = mode;
        self
    }

    /// Coarse HPA* pathfinding mode. Uses cached data, path won't be the shortest path, but the performance is extremely fast
    pub fn coarse(mut self) -> Self {
        self.mode = PathfindMode::Coarse;
        self
    }

    /// HPA* Any-Angle waypoint mode. Calculates the HPA* path, but only returns the specific positions the agent needs to avoid walls.
    /// This is meant for games with fluid realtime movement.
    pub fn waypoints(mut self) -> Self {
        self.mode = PathfindMode::Waypoints;
        self
    }

    /// A* pathfinding mode. Uses the standard A* algorithm.
    pub fn astar(mut self) -> Self {
        self.mode = PathfindMode::AStar;
        self
    }

    /// ThetaStar any-angle pathfinding. Only returns the specific positions the agent needs to avoid walls.
    /// This is meant for games with fluid realtime movement.
    pub fn thetastar(mut self) -> Self {
        self.mode = PathfindMode::ThetaStar;
        self
    }

    /// Refined is the default so this isn't really needed.
    /// Gets an HPA* path and then refines with it with a line tracing algorithm.
    pub fn refined(mut self) -> Self {
        self.mode = PathfindMode::Refined;
        self
    }

    /// Provides a blocking hashmap, this is used for dynamic obstacles (i.e. collision)
    pub fn blocking(mut self, blocking: &'a HashMap<UVec3, Entity>) -> Self {
        self.blocking = Some(blocking);
        self
    }

    /// Provide a navigation mask for this pathfinding call
    pub fn mask(mut self, mask: &'a mut NavMask) -> Self {
        self.mask = Some(mask);
        self
    }
}

/// AStar pathfinding
///
/// This function is provided if you want to supply your own grid.
/// If you're using the built in [`Grid`] you can use the pathfinding helper functions
/// provided in the [`Grid`] struct.
///
/// # Arguments
/// * `neighborhood` - The [`Neighborhood`] to use for the pathfinding.
/// * `grid` - The [`ArrayView3`] of [`NavCell`]s to use for the pathfinding.
/// * `start` - The starting position.
/// * `goal` - The goal position.
/// * `blocking` - A hashmap of blocked positions for dynamic obstacles.
/// * `partial` - If true, the pathfinding will return a partial path if the goal is blocked.
#[inline(always)]
// This has to be moved internally since the base A* and Djikstra algorithms use precomputed neighbors now.
pub(crate) fn pathfind_astar<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &NavMaskData,
    limits: SearchLimits,
) -> Option<Path> {
    let goal_cell = grid[[goal.x as usize, goal.y as usize, goal.z as usize]].clone();

    if let Some(mask_cell) = mask.get(goal_cell, goal) {
        if mask_cell.is_impassable() && !limits.partial {
            return None;
        }
    }

    //let start_time = std::time::Instant::now();
    let path = astar_grid(neighborhood, grid, start, goal, blocking, mask, limits);
    //log::info!("ASTAR took {:?}", start_time.elapsed());

    if let Some(mut path) = path {
        path.path.pop_front();
        Some(path)
    } else {
        None
    }
}

/// ThetaStar pathfinding
///
/// This function is provided if you want to supply your own grid.
/// If you're using the built in [`Grid`] you can use the pathfinding helper functions
/// provided in the [`Grid`] struct.
///
/// # Arguments
/// * `neighborhood` - The [`Neighborhood`] to use for the pathfinding.
/// * `grid` - The [`ArrayView3`] of [`NavCell`]s to use for the pathfinding.
/// * `start` - The starting position.
/// * `goal` - The goal position.
/// * `blocking` - A hashmap of blocked positions for dynamic obstacles.
/// * `partial` - If true, the pathfinding will return a partial path if the goal is blocked.
#[inline(always)]
// This has to be moved internally since the base A* and Djikstra algorithms use precomputed neighbors now.
pub(crate) fn pathfind_thetastar<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &NavMaskData,
    limits: SearchLimits,
) -> Option<Path> {
    // If the goal is impassibe and partial isn't set, return none
    if grid[[start.x as usize, start.y as usize, start.z as usize]].is_impassable()
        || grid[[goal.x as usize, goal.y as usize, goal.z as usize]].is_impassable()
            && !limits.partial
    {
        return None;
    }

    let goal_cell = grid[[goal.x as usize, goal.y as usize, goal.z as usize]].clone();

    // if goal is in the blocking mask, return None
    if let Some(mask_cell) = mask.get(goal_cell, goal) {
        if mask_cell.is_impassable() && !limits.partial {
            //log::error!("Goal is in the blocking mask");
            return None;
        }
    }

    thetastar_grid(neighborhood, grid, start, goal, blocking, mask, limits)
}

/// HPA* pathfinding.
// Keeping this internal for now since Grid has it's own helper function to call this
// and [`Grid`] is required for it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn pathfind<N: Neighborhood>(
    grid: &Grid<N>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &mut NavMaskData,
    refined: bool,
    waypoints: bool,
    limits: SearchLimits,
) -> Option<Path> {
    if limits.partial {
        log::warn!("Partial pathfinding is not supported with HPA*, use A* or Theta* instead");
    }
    let goal_cell = grid.view()[[goal.x as usize, goal.y as usize, goal.z as usize]].clone();

    // If the goal is impassable and partial isn't set, return none
    if let Some(mask_cell) = mask.get(goal_cell, goal) {
        if mask_cell.is_impassable() && !limits.partial {
            return None;
        }
    }

    let start_chunk = grid.chunk_at_position(start)?;
    let goal_chunk = grid.chunk_at_position(goal)?;

    // If the start and goal are in the same chunk, use AStar directly
    if start_chunk == goal_chunk {
        let path = astar_grid(
            &grid.neighborhood,
            &grid.view(),
            start,
            goal,
            blocking,
            mask,
            limits,
        );

        if let Some(mut path) = path {
            path.path.pop_front();
            return Some(path);
        } else {
            return None;
        }
    }

    // Find viable nodes in the start and goal chunks
    let (start_nodes, start_paths) =
        filter_and_rank_chunk_nodes(grid, start_chunk, start, goal, mask)?;
    let (goal_nodes, goal_paths) =
        filter_and_rank_chunk_nodes(grid, goal_chunk, goal, start, mask)?;

    let mut path: Vec<UVec3> = Vec::new();
    let mut cost = 0;

    for start_node in &start_nodes {
        for goal_node in goal_nodes.clone() {
            let node_path = hpa(grid, start_node.pos, goal_node.pos, blocking, mask, limits);

            if let Some(mut node_path) = node_path {
                let start_keys: HashSet<_> = start_paths.keys().copied().collect();
                let goal_keys: HashSet<_> = goal_paths.keys().copied().collect();

                trim_path(
                    &mut node_path,
                    &start_keys,
                    &goal_keys,
                    start_chunk,
                    goal_chunk,
                );

                let start_pos = node_path.path.front().unwrap();
                let goal_pos = node_path.path.back().unwrap();

                // Add start_path to the node_path
                let start_path = start_paths.get(&(start_pos - start_chunk.min())).unwrap();
                path.extend(start_path.path().iter().map(|pos| *pos + start_chunk.min()));
                cost += start_path.cost();

                // Add the node_path to the path (check for connection point overlap)
                let node_positions = node_path.path();
                if !path.is_empty()
                    && !node_positions.is_empty()
                    && path.last() == Some(&node_positions[0])
                {
                    // Skip the first position of node_path since it duplicates the last position of start_path
                    path.extend(node_positions.iter().skip(1));
                } else {
                    path.extend(node_positions.iter());
                }
                cost += node_path.cost();

                // Add goal path to path (check for connection point overlap)
                let end_path = goal_paths.get(&(goal_pos - goal_chunk.min())).unwrap();
                let goal_positions: Vec<UVec3> = end_path
                    .path()
                    .iter()
                    .rev()
                    .map(|pos| *pos + goal_chunk.min())
                    .collect();

                if !path.is_empty()
                    && !goal_positions.is_empty()
                    && path.last() == Some(&goal_positions[0])
                {
                    // Skip the first position of goal_path since it duplicates the last position of node_path
                    path.extend(goal_positions.iter().skip(1));
                } else {
                    path.extend(goal_positions.iter());
                }
                cost += end_path.cost();

                if path.is_empty() {
                    return None;
                }

                // On some occassions extending the goal path can add in a duplicate goal position at the end.
                // It's cheaper/cleaner to just clean up after it.
                if path.len() >= 2 && path[path.len() - 1] == path[path.len() - 2] {
                    path.pop();
                }

                if !refined && !waypoints {
                    // If we're not refining, return the path as is
                    let mut path = Path::new(path, cost);
                    path.graph_path = node_path.path;
                    return Some(path);
                }

                if waypoints {
                    let waypoints_path = extract_waypoints(
                        &grid.neighborhood,
                        &grid.view(),
                        &Path::new(path, cost),
                        mask,
                    );
                    if waypoints_path.is_empty() {
                        log::warn!("Waypoints path is empty, returning None");
                        return None;
                    }

                    return Some(waypoints_path);
                }

                let mut refined_path = optimize_path(
                    &grid.neighborhood,
                    &grid.view(),
                    mask,
                    &Path::from_slice(&path, cost),
                );

                // remove the starting position from the refined path
                refined_path.path.pop_front();

                // add the graph path to the refined path
                refined_path.graph_path = node_path.path;

                return Some(refined_path);
            }
        }
    }

    None
}

// Some times the Graph A* will return a path that has valid but redundant nodes at the start and end
// of the path. Leading to awkward paths where the agent appears to veers off before heading to the goal.
// This trims the path to ensure that only one entrance and exit node is used for the start and goal chunks.
pub(crate) fn trim_path(
    path: &mut Path,
    start_keys: &HashSet<UVec3>, // local positions in start_paths
    goal_keys: &HashSet<UVec3>,  // local positions in goal_paths
    start_chunk: &Chunk,
    goal_chunk: &Chunk,
) {
    // === Trim start ===
    if let Some(i) = path
        .path
        .iter()
        .position(|pos| start_keys.contains(&(*pos - start_chunk.min())))
    {
        // Remove everything before this index
        for _ in 0..i {
            path.path.pop_front();
        }
    }

    // === Trim end ===
    if let Some(i) = path
        .path
        .iter()
        .rposition(|pos| goal_keys.contains(&(*pos - goal_chunk.min())))
    {
        // Remove everything after this index
        let len = path.path.len();
        for _ in (i + 1)..len {
            path.path.pop_back();
        }
    }

    assert!(
        !path.path.is_empty(),
        "BUG: trim_path() removed all nodes — this should never happen"
    );
}

/// Extract waypoints from a path by checking for line of sight between nodes.
pub(crate) fn extract_waypoints<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    path: &Path,
    mask: &NavMaskData,
) -> Path {
    if path.is_empty() {
        return path.clone();
    }

    let filtered = !neighborhood.filters().is_empty();
    let mut waypoints_path = Vec::with_capacity(path.len());
    let mut total_cost = 0;
    let mut i = 0;

    waypoints_path.push(path.path[i]); // Always keep the first node

    while i < path.len() - 1 {
        let mut found = false;
        for farthest in (i + 1..path.len()).rev() {
            let candidate = path.path[farthest];
            if let Some(shortcut) = bresenham_path_internal(
                grid,
                path.path[i],
                candidate,
                neighborhood.is_ordinal(),
                filtered,
                true,
                mask,
            ) {
                for &pos in shortcut.iter().skip(1) {
                    let cell_val = grid[[pos.x as usize, pos.y as usize, pos.z as usize]].clone();
                    let masked_cell = mask.get(cell_val.clone(), pos).unwrap_or(cell_val);
                    total_cost += masked_cell.cost;
                }

                waypoints_path.push(candidate);
                i = farthest;
                found = true;
                break;
            }
        }
        if !found {
            // No shortcut found, advance by one
            i += 1;
            if i < path.len() && waypoints_path.last() != Some(&path.path[i]) {
                if let Some(step) = bresenham_path_internal(
                    grid,
                    *waypoints_path.last().unwrap(),
                    path.path[i],
                    neighborhood.is_ordinal(),
                    filtered,
                    true,
                    mask,
                ) {
                    for &pos in step.iter().skip(1) {
                        let cell_val =
                            grid[[pos.x as usize, pos.y as usize, pos.z as usize]].clone();
                        let masked_cell = mask.get(cell_val.clone(), pos).unwrap_or(cell_val);
                        total_cost += masked_cell.cost;
                    }
                }
                waypoints_path.push(path.path[i]);
            }
        }
    }

    Path::new(waypoints_path, total_cost)
}

/// Optimize a path by using line of sight checks to skip waypoints.
///
/// This is used to optimize paths generated by the HPA* algorithms.
/// [`Grid::pathfind`] uses this internally so this is only needed if you want to
/// optimize a path that was generated by a different method.
///
/// # Arguments
///
/// * `neighborhood` - The [`Neighborhood`] to use for the pathfinding.
/// * `grid` - The [`ArrayView3`] of the grid.
/// * `path` - The [`Path`] to optimize.
/// * `ordinal` - If true, use ordinal movement. If false, use cardinal movement.
///
#[inline(always)]
pub(crate) fn optimize_path<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    mask: &NavMaskData,
    path: &Path,
) -> Path {
    if path.is_empty() {
        return path.clone();
    }

    let filtered = !neighborhood.filters().is_empty();
    let mut refined_path = Vec::with_capacity(path.len());
    let mut i = 0;

    // Pre-compute all cells to avoid repeated grid access
    let path_cells: Vec<NavCell> = path
        .path
        .iter()
        .map(|pos| {
            let cell = grid[[pos.x as usize, pos.y as usize, pos.z as usize]].clone();
            mask.get(cell.clone(), *pos).unwrap_or(cell)
        })
        .collect();

    refined_path.push(path.path[i]); // Always keep the first node

    while i < path.len() {
        let mut shortcut_taken = false;

        let search_limit = if path.len() < 100 {
            (i + 25).min(path.len()) // Smaller limit for short paths
        } else if path.len() < 500 {
            (i + 50).min(path.len()) // Medium limit
        } else {
            (i + 75).min(path.len()) // Larger limit for very long paths
        };

        for farthest in (i + 1..search_limit).rev() {
            let candidate = path.path[farthest];

            let maybe_shortcut = bresenham_path_internal(
                grid,
                path.path[i],
                candidate,
                neighborhood.is_ordinal(),
                filtered,
                false,
                mask,
            );

            if let Some(shortcut) = maybe_shortcut {
                // Calculate shortcut cost - still need to compute this for masked cells
                let shortcut_cost: u32 = shortcut
                    .iter()
                    .skip(1)
                    .map(|pos| {
                        let cell = grid[[pos.x as usize, pos.y as usize, pos.z as usize]].clone();
                        mask.get(cell.clone(), *pos).unwrap_or(cell).cost
                    })
                    .sum();

                // Use pre-computed cells for non-shortcut cost
                let non_shortcut_cost: u32 = path_cells
                    .iter()
                    .skip(i + 1)
                    .take(farthest - i)
                    .map(|cell| cell.cost)
                    .sum();

                if shortcut_cost <= non_shortcut_cost {
                    refined_path.extend(shortcut.into_iter().skip(1));
                    i = farthest;
                    shortcut_taken = true;
                    break;
                }
            }
        }

        if !shortcut_taken {
            i += 1;
            if i < path.len() {
                refined_path.push(path.path[i]);
            }
        }
    }

    // Use pre-computed approach for final cost calculation
    let cost = refined_path
        .iter()
        .map(|pos| {
            let cell = grid[[pos.x as usize, pos.y as usize, pos.z as usize]].clone();
            mask.get(cell.clone(), *pos).unwrap_or(cell).cost
        })
        .sum();

    let mut path = Path::new(refined_path.clone(), cost);
    path.graph_path = refined_path.into();
    path
}
/*#[inline(always)]
pub(crate) fn optimize_path<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    mask: &NavMaskData,
    path: &Path,
) -> Path {
    if path.is_empty() {
        return path.clone();
    }

    let filtered = !neighborhood.filters().is_empty();

    let mut refined_path = Vec::with_capacity(path.len());
    let mut i = 0;

    refined_path.push(path.path[i]); // Always keep the first node

    while i < path.len() {
        let mut shortcut_taken = false;

        for farthest in (i + 1..path.len()).rev() {
            let candidate = path.path[farthest];

            // Reject if direction changes drastically
            /*if let Some(prev_dir) = last_dir {
                if dir != prev_dir && dir.dot(prev_dir) < 0 {
                    continue; // Skip this candidate
                }
            }*/

            let maybe_shortcut = bresenham_path(
                grid,
                path.path[i],
                candidate,
                neighborhood.is_ordinal(),
                filtered,
                false,
            );

            if let Some(shortcut) = maybe_shortcut {
                refined_path.extend(shortcut.into_iter().skip(1));
                i = farthest;
                shortcut_taken = true;
                break;
            }
        }

        if !shortcut_taken {
            i += 1;
            if i < path.len() {
                refined_path.push(path.path[i]);
            }
        }
    }

    // This is trash.
    //let refined_path = push_turns_to_corners(&refined_path, grid);

    // Recompute cost of new path
    let cost = refined_path
        .iter()
        .map(|pos| grid[[pos.x as usize, pos.y as usize, pos.z as usize]].cost)
        .sum();

    let mut path = Path::new(refined_path.clone(), cost);
    path.graph_path = refined_path.into();
    path
}*/

// Filters and ranks nodes within a chunk based on reachability and proximity.
#[inline(always)]
fn filter_and_rank_chunk_nodes<'a, N: Neighborhood>(
    grid: &'a Grid<N>,
    chunk: &Chunk,
    source: UVec3,
    target: UVec3,
    mask: &NavMaskData,
) -> Option<(Vec<&'a Node>, HashMap<UVec3, Path>)> {
    let nodes = grid.graph().nodes_in_chunk(chunk);

    let min = chunk.min().as_ivec3();

    let mask_local = mask.translate_by(-min);

    // Get paths from source to all nodes in this chunk
    let paths = dijkstra_grid(
        &grid.chunk_view(chunk),
        source - chunk.min(),
        &nodes
            .iter()
            .map(|node| node.pos - chunk.min())
            .collect::<Vec<_>>(),
        false,
        &mask_local,
    );

    let filtered_nodes = nodes
        .iter()
        .filter(|node| paths.contains_key(&(node.pos - chunk.min())))
        .collect::<Vec<_>>();

    if filtered_nodes.is_empty() {
        return None;
    }

    let mut ranked_nodes = filtered_nodes
        .iter()
        .map(|node| {
            let d_start = manhattan_distance(node.pos, source);
            let d_goal = manhattan_distance(node.pos, target);
            (*node, d_start + d_goal)
        })
        .collect::<Vec<_>>();

    ranked_nodes.sort_by_key(|(_, dist)| *dist);
    Some((ranked_nodes.into_iter().map(|(n, _)| *n).collect(), paths))
}

#[inline(always)]
fn manhattan_distance(a: UVec3, b: UVec3) -> i32 {
    (a.x as i32 - b.x as i32).abs()
        + (a.y as i32 - b.y as i32).abs()
        + (a.z as i32 - b.z as i32).abs()
}

/// Recursively reroutes a path by astar pathing to further chunks until a path can be found.
///
/// Useful if local collision avoidance is failing.
///
/// If you're using the plugin pathing systems, you shouldn't need to call this directly.
///
/// # Arguments
#[inline(always)]
pub(crate) fn reroute_path<N: Neighborhood>(
    grid: &Grid<N>,
    path: &Path,
    start: UVec3,
    pathfind: &Pathfind,
    blocking: &HashMap<UVec3, Entity>,
    mask: &mut NavMaskData,
) -> Option<Path> {
    // When the starting chunks entrances are all blocked, this will try astar path to the NEXT chunk in the graph path
    // recursively until it can find a path out.
    // If it can't find a path out, it will return None.

    if !grid.in_bounds(start) || !grid.in_bounds(pathfind.goal) {
        return None;
    }

    if path.graph_path.is_empty() {
        // Our only option here is to try a new path to the goal
        // We can unwrap here because this is internal and the caller has at least inserted the default
        match pathfind.mode.unwrap() {
            PathfindMode::Refined | PathfindMode::Coarse | PathfindMode::AStar => {
                return pathfind_astar(
                    &grid.neighborhood,
                    &grid.view(),
                    start,
                    pathfind.goal,
                    blocking,
                    mask,
                    pathfind.limits,
                );
            }
            PathfindMode::Waypoints | PathfindMode::ThetaStar => {
                return pathfind_thetastar(
                    &grid.neighborhood,
                    &grid.view(),
                    start,
                    pathfind.goal,
                    blocking,
                    mask,
                    pathfind.limits,
                );
            }
        }
    }

    let max_attempts = 3;

    let new_path = path.graph_path.iter().take(max_attempts).find_map(|pos| {
        let new_path = match pathfind.mode.unwrap() {
            PathfindMode::Refined | PathfindMode::Coarse | PathfindMode::AStar => pathfind_astar(
                &grid.neighborhood,
                &grid.view(),
                start,
                *pos,
                blocking,
                mask,
                pathfind.limits,
            ),
            PathfindMode::Waypoints | PathfindMode::ThetaStar => pathfind_thetastar(
                &grid.neighborhood,
                &grid.view(),
                start,
                *pos,
                blocking,
                mask,
                pathfind.limits,
            ),
        };

        if new_path.is_some() && !new_path.as_ref().unwrap().is_empty() {
            new_path
        } else {
            None
        }
    });

    // If we find a new route, we need to solve the rest of it
    if let Some(new_path) = &new_path {
        let mut full_path = Vec::new();

        for pos in new_path.path() {
            full_path.push(*pos);
        }

        let last_pos = *full_path.last().unwrap();

        let remaining_path = match pathfind.mode.unwrap() {
            PathfindMode::Refined | PathfindMode::Coarse | PathfindMode::AStar => pathfind_astar(
                &grid.neighborhood,
                &grid.view(),
                last_pos,
                pathfind.goal,
                blocking,
                mask,
                pathfind.limits,
            ),
            PathfindMode::Waypoints | PathfindMode::ThetaStar => pathfind_thetastar(
                &grid.neighborhood,
                &grid.view(),
                last_pos,
                pathfind.goal,
                blocking,
                mask,
                pathfind.limits,
            ),
        };

        if let Some(remaining_path) = remaining_path {
            for pos in remaining_path.path() {
                full_path.push(*pos);
            }

            let mut path = Path::new(full_path, new_path.cost() + remaining_path.cost());
            path.graph_path = remaining_path.graph_path.clone();
            return Some(path);
        }
    }

    None
}
