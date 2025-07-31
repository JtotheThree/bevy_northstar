//! This module defines pathfinding functions which can be called directly.

use bevy::{
    ecs::entity::Entity,
    log,
    math::{IVec3, UVec3},
    platform::collections::{HashMap, HashSet},
};
use ndarray::ArrayView3;

use crate::{
    astar::{astar_graph, astar_grid},
    chunk::Chunk,
    dijkstra::dijkstra_grid,
    grid::Grid,
    nav::NavCell,
    nav_mask::NavMasks,
    node::Node,
    path::Path,
    prelude::Neighborhood,
    raycast::{bresenham_path, bresenham_path_filtered},
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
    masks: &NavMasks,
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

    // if goal is blocked in the nav mask return None
    if let Some(nav_cell) = masks.nav(grid, goal) {
        if nav_cell.is_impassable() && !partial {
            return None;
        }
    }

    let path = astar_grid(neighborhood, grid, start, goal, 1024, partial, masks);

    if let Some(mut path) = path {
        path.path.pop_front();
        Some(path)
    } else {
        None
    }
}

/// HPA* pathfinding.
// Keeping this internal for now since Grid has it's own helper function to call this
// and [`Grid`] is required for it.
#[inline(always)]
pub(crate) fn pathfind<N: Neighborhood>(
    grid: &Grid<N>,
    start: UVec3,
    goal: UVec3,
    masks: &NavMasks,
    partial: bool,
    refined: bool,
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

    // If the goal is impassable and partial isn't set, return none
    if grid.view()[[start.x as usize, start.y as usize, start.z as usize]].is_impassable()
        || grid.view()[[goal.x as usize, goal.y as usize, goal.z as usize]].is_impassable()
            && !partial
    {
        return None;
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
        filter_and_rank_chunk_nodes(grid, start_chunk, start, goal, masks)?;
    let (goal_nodes, goal_paths) =
        filter_and_rank_chunk_nodes(grid, goal_chunk, goal, start, masks)?;

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

                if !refined {
                    // If we're not refining, return the path as is
                    let mut path = Path::new(path, cost);
                    path.graph_path = node_path.path;
                    return Some(path);
                }

                let mut refined_path = optimize_path(
                    &grid.neighborhood,
                    &grid.view(),
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

/*#[inline(always)]
pub(crate) fn trim_path(
    path: &mut Path,
    starts: Vec<UVec3>,
    goals: Vec<UVec3>,
) {
    // Trim out reduntant nodes in the viable starts
    if let Some(last_start_node) = path
        .path
        .iter()
        .rev()
        .find(|&&pos| starts.contains(&pos))
        .cloned()
    {
        while let Some(first) = path.path.front() {
            // Trim only if it's a different node in the start chunk
            if starts.contains(first) && *first != last_start_node {
                path.path.pop_front();
            } else {
                break;
            }
        }
    }

    // Trim out redundant nodes in the viable goals
    if let Some(first_goal_node) = path
        .path
        .iter()
        .find(|&&pos| goals.contains(&pos))
        .cloned()
    {
        while let Some(last) = path.path.back() {
            if goals.contains(last) && *last != first_goal_node {
                path.path.pop_back();
            } else {
                break;
            }
        }
    }

    assert!(
        !path.path.is_empty(),
        "BUG: trim_path() removed all nodes — this should never happen"
    );
}*/

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
    path: &Path,
) -> Path {
    if path.is_empty() {
        return path.clone();
    }

    let filtered = !neighborhood.filters().is_empty();

    let mut refined_path = Vec::with_capacity(path.len());
    let mut i = 0;

    let mut last_dir: Option<IVec3> = None;

    refined_path.push(path.path[i]); // Always keep the first node

    while i < path.len() {
        let mut shortcut_taken = false;

        for farthest in (i + 1..path.len()).rev() {
            let candidate = path.path[farthest];
            let dir = (candidate.as_ivec3() - path.path[i].as_ivec3()).signum();

            // Reject if direction changes drastically
            if let Some(prev_dir) = last_dir {
                if dir != prev_dir && dir.dot(prev_dir) < 0 {
                    continue; // Skip this candidate
                }
            }

            let maybe_shortcut = if filtered {
                bresenham_path_filtered(grid, path.path[i], candidate, neighborhood.is_ordinal())
            } else {
                bresenham_path(grid, path.path[i], candidate, neighborhood.is_ordinal())
            };

            if let Some(shortcut) = maybe_shortcut {
                refined_path.extend(shortcut.into_iter().skip(1));
                i = farthest;
                shortcut_taken = true;
                last_dir = Some(dir);
                break;
            }
        }

        if !shortcut_taken {
            i += 1;
            if i < path.len() {
                let dir = if let Some(prev) = refined_path.last() {
                    (path.path[i].as_ivec3() - prev.as_ivec3()).signum()
                } else {
                    IVec3::ZERO
                };
                refined_path.push(path.path[i]);
                last_dir = Some(dir);
            }
        }
    }

    // Recompute cost of new path
    let cost = refined_path
        .iter()
        .map(|pos| grid[[pos.x as usize, pos.y as usize, pos.z as usize]].cost)
        .sum();

    let mut path = Path::new(refined_path.clone(), cost);
    path.graph_path = refined_path.into();
    path
}

/*pub(crate) fn optimize_path_old<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<NavCell>,
    path: &Path,
) -> Path {
    if path.is_empty() {
        return path.clone();
    }

    let filtered = !neighborhood.filters().is_empty();

    let goal = *path.path.back().unwrap();
    let mut refined_path = Vec::with_capacity(path.len());
    let mut i = 0;

    refined_path.push(path.path[i]); // Always keep the first node

    while i < path.len() {
        if !filtered {
            if let Some(direct_path) = bresenham_path(grid, path.path[i], goal, neighborhood.is_ordinal()) {
                // If we can reach the goal directly, add it and break
                refined_path.extend(direct_path.into_iter().skip(1));
                break;
            }
        } else {
            if let Some(direct_path) = bresenham_path_filtered(grid, path.path[i], goal, neighborhood.is_ordinal()) {
                // If we can reach the goal directly, add it and break
                refined_path.extend(direct_path.into_iter().skip(1));
                break;
            }
        }

        // Try to find the farthest reachable waypoint from i (greedy)
        let mut shortcut_taken = false;
        for farthest in (i + 1..path.len()).rev() {
            if !filtered {
                if let Some(shortcut) = bresenham_path(
                    grid,
                    path.path[i],
                    path.path[farthest],
                    neighborhood.is_ordinal(),
                ) {
                    refined_path.extend(shortcut.into_iter().skip(1));
                    i = farthest;
                    shortcut_taken = true;
                    break;
                }
            } else if let Some(shortcut) = bresenham_path_filtered(
                grid,
                path.path[i],
                path.path[farthest],
                neighborhood.is_ordinal(),
            ) {
                refined_path.extend(shortcut.into_iter().skip(1));
                i = farthest;
                shortcut_taken = true;
                break;
            }
        }

        if !shortcut_taken {
            // No shortcut found — advance by one
            i += 1;
            if i < path.len() {
                refined_path.push(path.path[i]);
            }
        }
    }

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
    masks: &NavMasks,
) -> Option<(Vec<&'a Node>, HashMap<UVec3, Path>)> {
    let nodes = grid.graph().nodes_in_chunk(chunk);

    let min = chunk.min().as_ivec3();
    let max = chunk.max().as_ivec3();

    let mask_local = CompositeNavMaskView::new(mask, min, (min, max));

    // Adjust the blocking map to the local chunk coordinates
    /*let adjusted_blocking = blocking
        .iter()
        .map(|(pos, entity)| (chunk.to_local(pos), *entity))
        .collect::<HashMap<_, _>>();*/


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
    mask: &CompositeNavMask,
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
        return pathfind_astar(
            &grid.neighborhood,
            &grid.view(),
            start,
            goal,
            mask,
            false,
        );
    }

    let new_path = path.graph_path.iter().find_map(|pos| {
        let new_path = pathfind_astar(
            &grid.neighborhood,
            &grid.view(),
            start,
            *pos,
            mask,
            false,
        );
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

        let hpa = pathfind(grid, last_pos, goal, mask, false, refined);

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
