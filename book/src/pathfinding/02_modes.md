# Available Modes / Algorithms

## PathfindMode
The pathfinding algorithm enum. Current options are:

### Grid Movement Modes
These modes return paths that keep the agent’s movement constrained to the grid. Every position in the path is adjacent to the next.

#### `PathfindMode::Refined`
##### This is the default algorithm
Gets a high level path to the goal at the chunk level. If a path is found, the path is iterated over with a line of sight / tracing algorithm to attempt to create the shortest path to the goal. The refinement is more expensive than the HPA* algorithm but not nearly as expensive as using A*.

*Manual call*: `Grid::pathfind()`.

#### `PathfindMode::Coarse`
Returns the unrefined HPA* path pulled from the cached entrance paths. This will not return the path with the least steps to the goal but is extremely fast to generate. It's great for natural paths NPCs might use to move around a building for example.

*Manual call*: `Grid::pathfind_coarse()`.

#### `PathfindMode::AStar`
This is standard A* pathfinding. It's very expensive for long distance goals on large maps but is still useful for very short distances or when you're concerned with the absolute shortest path. A good use would be movement where action points are subtracted based on number of moves.

*Manual call*: `Grid::pathfind_astar()`.

### Any Angle Modes
Any-angle modes return the path as a series of waypoint segments. Each waypoint marks the start or end of a straight-line segment that can be traversed without intersecting any obstacles. This is intended for freeform movement where the agent is not constrained to follow the grid.

> **⚠️ Warning**
> The cost of an any-angle path is a good estimate but does not account for the exact cost of every grid cell the agent may or may not pass through between waypoints.

#### `PathfindMode::Waypoints`
Very fast HPA* any-angle pathfinding that returns only the essential waypoints instead of the full step-by-step path. It uses the standard HPA* algorithm, but during the refinement stage it outputs only the waypoints rather than expanding every intermediate step.

*Manual call*: `Grid::pathfind_waypoints()`.

#### `PathfindMode::ThetaStar`
An expensive but near-optimal any-angle algorithm. It produces paths similar to PathfindMode::Waypoints, but can sometimes yield slightly shorter results. This is a lazy Theta* implementation, which calculates line-of-sight for all neighbors during the search, making it significantly more costly to compute.

*Manual call*: `Grid::pathfind_thetastar()`.

### Limited Search
There is also an extra AStar algorthm that is currently only available by manually calling it on the grid `Grid::pathfind_astar_radius()`. This will run AStar pathfinding limited to a u32 radius around the starting point.
