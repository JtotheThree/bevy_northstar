//! This module defines pathfinding functions which can be called directly.

use bevy::{
    ecs::entity::Entity, log, math::UVec3, platform::collections::{HashMap, HashSet}
};
use ndarray::ArrayView3;

use crate::{
    astar::{astar_graph, astar_grid}, chunk::Chunk, dijkstra::dijkstra_grid, grid::Grid, hpa::hpa, nav::NavCell, nav_mask::NavMaskData, neighbor::Neighborhood, node::Node, path::Path, raycast::bresenham_path, thetastar::thetastar_grid
};

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
    partial: bool,
) -> Option<Path> {
    // Ensure the goal is within bounds of the grid
    let shape = grid.shape();
    if start.x as usize >= shape[0] || start.y as usize >= shape[1] || start.z as usize >= shape[2]
    {
        log::warn!("Start is out of bounds: {:?}", start);
        return None;
    }

    if goal.x as usize >= shape[0] || goal.y as usize >= shape[1] || goal.z as usize >= shape[2] {
        log::warn!("Goal is out of bounds: {:?}", goal);
        return None;
    }

    let goal_cell = grid[[goal.x as usize, goal.y as usize, goal.z as usize]].clone();

    if let Some(mask_cell) = mask.get(goal_cell, goal) {
        if mask_cell.is_impassable() && !partial {
            return None;
        }
    }

    //let start_time = std::time::Instant::now();
    let path = astar_grid(neighborhood, grid, start, goal, 1024, partial, blocking, mask);
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
    partial: bool,
) -> Option<Path> {
    // Ensure the goal is within bounds of the grid
    let shape = grid.shape();
    if start.x as usize >= shape[0] || start.y as usize >= shape[1] || start.z as usize >= shape[2]
    {
        log::warn!("Start is out of bounds: {:?}", start);
        return None;
    }

    if goal.x as usize >= shape[0] || goal.y as usize >= shape[1] || goal.z as usize >= shape[2] {
        log::warn!("Goal is out of bounds: {:?}", goal);
        return None;
    }

    // If the goal is impassibe and partial isn't set, return none
    if grid[[start.x as usize, start.y as usize, start.z as usize]].is_impassable()
        || grid[[goal.x as usize, goal.y as usize, goal.z as usize]].is_impassable() && !partial
    {
        return None;
    }

    let goal_cell = grid[[goal.x as usize, goal.y as usize, goal.z as usize]].clone();

    // if goal is in the blocking mask, return None
    if let Some(mask_cell) = mask.get(goal_cell, goal) {
        if mask_cell.is_impassable() && !partial {
            //log::error!("Goal is in the blocking mask");
            return None;
        }
    }

    thetastar_grid(neighborhood, grid, start, goal, 1024, partial, blocking, mask)
}

pub(crate) fn pathfind_new<N: Neighborhood>(
    grid: &Grid<N>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &mut NavMaskData,
    partial: bool,
    refined: bool,
    waypoints: bool,
) -> Option<Path> {
    if !grid.in_bounds(start) {
        log::warn!("Start is out of bounds: {:?}", start);
        return None;
    }

    // Make sure the goal is in grid bounds
    if !grid.in_bounds(goal) {
        log::warn!("Goal is out of bounds: {:?}", goal);
        return None;
    }

    let goal_cell = grid.view()[[goal.x as usize, goal.y as usize, goal.z as usize]].clone();

    // If the goal is impassable and partial isn't set, return none
    if let Some(mask_cell) = mask.get(goal_cell, goal) {
        if mask_cell.is_impassable() && !partial {
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
            100,
            partial,
            blocking,
            mask,
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
            let node_path = hpa(
                grid,
                start_node.pos,
                goal_node.pos,
                100,
                partial,
                blocking,
                mask,
            );

            if let Some(mut node_path) = node_path {
                /*for pos in &node_path.path {
                    if let Some(cell) = mask.get(grid.navcell(*pos).clone(), *pos) {
                        if cell.is_impassable() {
                            log::error!("Path goes through impassable cell at {:?}", pos);
                        }
                    }
                }*/

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
                // skip(1) skips the start position, we don't want to return that ever
                path.extend(start_path.path().iter().skip(1).map(|pos| *pos + start_chunk.min()));
                cost += start_path.cost();

                // Add the node_path to the path
                path.extend(node_path.path());
                cost += node_path.cost();

                // Add goal path to path
                let end_path = goal_paths.get(&(goal_pos - goal_chunk.min())).unwrap();
                path.extend(
                    end_path
                        .path()
                        .iter()
                        .rev()
                        .map(|pos| *pos + goal_chunk.min()),
                );
                cost += end_path.cost();

                if path.is_empty() {
                    return None;
                }

                // On some occassions extending the goal path can add in a duplicate goal position at the end.
                // It's cheaper/cleaner to just clean up after it.
                if path.len() >= 2 && path[path.len() - 1] == path[path.len() - 2] {
                    path.pop();
                }

                // Same with the start
                if path.len() >= 2 && path[0] == path[1] {
                    log::warn!("Start contains duplicate nodes: {:?} <-> {:?}", path[0], path[1]);
                }

                if path[0] == start {
                    log::warn!("Path shouldn't have the start position in it!!!");
                }

                //log::info!("Found unrefined path: {:?}, with cost {}", path, cost);

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

                //log::info!("HPA refinement took {:?}", start_time.elapsed());

                // remove the starting position from the refined path
                refined_path.path.pop_front();

                // Debug test, ensure that NONE of the positions appear in the mask
                /*for pos in &refined_path.path {
                    if let Some(cell) = mask.get(grid.navcell(*pos).clone(), *pos) {
                        if cell.is_impassable() {
                            log::error!("Refined path goes through impassable cell at {:?}", pos);
                        }
                    }
                }*/

                // add the graph path to the refined path
                refined_path.graph_path = node_path.path;

                //log::info!("Refined path: {:?}, with cost: {:?}", refined_path.path(), refined_path.cost());

                return Some(refined_path);
            }
        }
    }

    None
}


/// HPA* pathfinding.
// Keeping this internal for now since Grid has it's own helper function to call this
// and [`Grid`] is required for it.
#[inline(always)]
pub(crate) fn pathfind<N: Neighborhood>(
    grid: &Grid<N>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &NavMaskData,
    partial: bool,
    refined: bool,
    waypoints: bool,
) -> Option<Path> {
    if !grid.in_bounds(start) {
        log::warn!("Start is out of bounds: {:?}", start);
        return None;
    }

    // Make sure the goal is in grid bounds
    if !grid.in_bounds(goal) {
        log::warn!("Goal is out of bounds: {:?}", goal);
        return None;
    }

    let goal_cell = grid.view()[[goal.x as usize, goal.y as usize, goal.z as usize]].clone();

    // If the goal is impassable and partial isn't set, return none
    if let Some(mask_cell) = mask.get(goal_cell, goal) {
        if mask_cell.is_impassable() && !partial {
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
            100,
            partial,
            blocking,
            mask,
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
            let node_path = astar_graph(
                &grid.neighborhood,
                grid.graph(),
                start_node.pos,
                goal_node.pos,
                100,
            );

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

                // Add node_path paths to path
                for (node, next_node) in
                    node_path.path().iter().zip(node_path.path().iter().skip(1))
                {
                    // Get the cached edge path between node and next node
                    let cached_path = grid.graph().node_at(*node).unwrap().edges[next_node].clone();
                    path.extend(cached_path.path().iter().skip(1));
                    cost += cached_path.cost();
                }

                // Add end path to path
                let end_path = goal_paths.get(&(goal_pos - goal_chunk.min())).unwrap();
                path.extend(
                    end_path
                        .path()
                        .iter()
                        .rev()
                        .map(|pos| *pos + goal_chunk.min()),
                );
                cost += end_path.cost();

                if path.is_empty() {
                    return None;
                }

                // On some occassions extending the goal path can add in a duplicate goal position at the end.
                // It's cheaper/cleaner to just clean up after it.
                if path.len() >= 2 && path[path.len() - 1] == path[path.len() - 2] {
                    path.pop();
                }

                // Same with the start
                if path.len() >= 2 && path[0] == path[1] {
                    log::warn!("Start contains duplicate nodes: {:?}", path);
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
            if let Some(shortcut) = bresenham_path(
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
                if let Some(step) = bresenham_path(
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

    log::info!("Found total_cost: {}", total_cost);

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
    let path_cells: Vec<NavCell> = path.path
        .iter()
        .map(|pos| {
            let cell = grid[[pos.x as usize, pos.y as usize, pos.z as usize]].clone();
            mask.get(cell.clone(), *pos).unwrap_or(cell)
        })
        .collect();

    refined_path.push(path.path[i]); // Always keep the first node

    while i < path.len() {
        let mut shortcut_taken = false;

        for farthest in (i + 1..path.len()).rev() {
            let candidate = path.path[farthest];

            let maybe_shortcut = bresenham_path(
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

    // Print all key values in the mask
    let min = chunk.min().as_ivec3();
    //let max = chunk.max().as_ivec3();

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
        100,
        &mask_local,
    );

   // Debug check: convert local positions to global before checking mask
    /*for path in paths.values() {
        for local_pos in &path.path {
            let global_pos = *local_pos + chunk.min(); // Convert to global coordinates
            let cell_val = grid.view()[[global_pos.x as usize, global_pos.y as usize, global_pos.z as usize]].clone();
            let masked_cell = mask.get(cell_val.clone(), global_pos).unwrap_or(cell_val); // Now using global mask with global pos
            if masked_cell.is_impassable() {
                log::error!("START PATH goes through impassable cell at {:?} (local: {:?})", global_pos, local_pos);
            }
        }
    }*/

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
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    mask: &NavMaskData,
    refined: bool,
) -> Option<Path> {
    // When the starting chunks entrances are all blocked, this will try astar path to the NEXT chunk in the graph path
    // recursively until it can find a path out.
    // If it can't find a path out, it will return None.

    if !grid.in_bounds(start) {
        log::warn!("Start is out of bounds: {:?}", start);
        return None;
    }

    if !grid.in_bounds(goal) {
        log::warn!("Goal is out of bounds: {:?}", goal);
        return None;
    }

    if path.graph_path.is_empty() {
        // Our only option here is to astar path to the goal
        return pathfind_astar(&grid.neighborhood, &grid.view(), start, goal, blocking, mask, false);
    }

    let new_path = path.graph_path.iter().find_map(|pos| {
        let new_path = pathfind_astar(&grid.neighborhood, &grid.view(), start, *pos, blocking, mask, false);
        if new_path.is_some() && !new_path.as_ref().unwrap().is_empty() {
            new_path
        } else {
            None
        }
    });

    // HPA the rest of the way to the goal using get_path from the last position in the new path to the goal
    if let Some(new_path) = new_path {
        let mut hpa_path = Vec::new();

        for pos in new_path.path() {
            hpa_path.push(*pos);
        }

        let last_pos = *new_path.path().last().unwrap();

        let hpa = pathfind(grid, last_pos, goal, blocking, mask, false, refined, false);

        if let Some(hpa) = hpa {
            for pos in hpa.path() {
                hpa_path.push(*pos);
            }

            let mut path = Path::new(hpa_path, new_path.cost() + hpa.cost());
            path.graph_path = hpa.graph_path.clone();
            return Some(path);
        }
    }

    None
}
