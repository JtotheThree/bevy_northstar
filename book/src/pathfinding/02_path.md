# Path
The `Path` component will be inserted to an entity when after a path is found. It is also returned when calling the manual pathfinding methods on `Grid`.

You can use it draw paths in your game, for example when highlighting tiles for a path in a turn based game.

There are a few noteable methods on `Path`.

### `path() -> &[UVec3]`

Returns an array of all positions in the path.

### `cost() -> u32`

The full cost of the path. It is the sum of all movement costs for each cell in the grid that the path crosses over.

### `len() -> usize`

The total number of positions **currently** in the path.

### `pop() -> Option<UVec3>`

Removes and returns the first position of the path.

#### See the crate docs for more. A couple of notable ones are `reverse()` and `translate_by(offset)`