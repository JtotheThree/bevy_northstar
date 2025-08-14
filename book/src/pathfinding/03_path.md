# The `Path` Component / Struct

The `Path` component will be inserted to an entity when after a path is found. It is also returned when calling the manual pathfinding methods on `Grid`.

You can use it draw paths in your game, for example when highlighting tiles for a path in a turn based game.

There are a few notable methods on `Path`.

### `path() -> &[UVec3]`

Returns an array of all positions in the path.

### `cost() -> u32`

The full cost of the path. It is the sum of all movement costs for each cell in the grid that the path crosses over.

### `len() -> usize`

The total number of positions **currently** in the path.

### `next() -> Option<UVec3>`

Returns the next position in the path without removing it. Very useful in animation systems to play a different animation based on tile transition for example, playing a jumping animation when the next position is a higher z depth.

### `pop() -> Option<UVec3>`

Removes and returns the next position of the path.

#### See the crate docs for more. A couple of notable ones are `reverse()` and `translate_by(offset)`