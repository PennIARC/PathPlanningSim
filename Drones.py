NUM_DRONES = 4
SIGHT_RANGE = 1  # cells seen in every direction (including diagonals), plus the cell under the drone
DRONE_COLORS = [[255, 96, 141], [237, 171, 80], [99, 199, 77], [44, 232, 245]]


class Drone:
    """One drone on the grid. All positions are (x, y) cell indices."""

    def __init__(self, index, color, position, liveMap):
        self.index = index
        self.color = color
        self.position = tuple(position)  # (x, y) cell the drone is in

        # The drone's own copy of the obstacles known on the live map. Starts as a
        # snapshot and then grows with whatever this drone sees itself.
        self.obstacles = {
            (x, y)
            for y, row in enumerate(liveMap.cells)
            for x, cell in enumerate(row)
            if cell.isObstacle
        }

        self.seenCells = set()      # (x, y) of cells this drone has seen
        self.assignedCells = set()  # (x, y) of cells this drone is assigned to search

        self.path = [self.position]   # cells to fly through, in order; path[0] is the start cell
        self.pathIndex = 0            # where along the path the drone currently is
        self.trail = [self.position]  # every cell the drone has been in, oldest first

    def set_path(self, path):
        self.path = list(path)
        self.pathIndex = 0

    def finished(self):
        return self.pathIndex >= len(self.path) - 1

    def step(self):
        """Moves one cell along the path. Returns False if the drone was already at the end."""
        if self.finished():
            return False
        self.pathIndex += 1
        self.position = self.path[self.pathIndex]
        self.trail.append(self.position)
        return True

    def visible_cells(self, grid):
        """Cells within SIGHT_RANGE of the drone that are inside the grid."""
        px, py = self.position
        return [
            (x, y)
            for y in range(py - SIGHT_RANGE, py + SIGHT_RANGE + 1)
            for x in range(px - SIGHT_RANGE, px + SIGHT_RANGE + 1)
            if grid.in_bounds(x, y)
        ]

    def sense(self, world, liveMap):
        """Looks at the real map around the drone and records what it sees on the live map."""
        for x, y in self.visible_cells(world):
            self.seenCells.add((x, y))
            isObstacle = world.cells[y][x].isObstacle
            if isObstacle:
                self.obstacles.add((x, y))
            known = liveMap.cells[y][x]
            if not known.searched or known.isObstacle != isObstacle:
                liveMap.set_cell(x, y, searched=True, isObstacle=isObstacle)

    # ---------------- Drawing. pygame is imported inside these so the rest of
    # this module works without it when the sim runs headless.

    def _cell_center(self, cell, origin, tile_size):
        return (origin[0] + (cell[0] + 0.5) * tile_size, origin[1] + (cell[1] + 0.5) * tile_size)

    def draw_trail(self, surface, origin, tile_size, width=1):
        """Draws straight line segments through every cell the drone has been in."""
        import pygame

        if len(self.trail) < 2:
            return
        points = [self._cell_center(cell, origin, tile_size) for cell in self.trail]
        pygame.draw.lines(surface, self.color, False, points, width)

    def draw(self, surface, origin, tile_size):
        """Draws the drone as a circle centred on its cell.

        origin is the top-left pixel of the grid on `surface`; tile_size is the
        pixel size of one cell on that surface.
        """
        import pygame

        pygame.draw.circle(surface, self.color, self._cell_center(self.position, origin, tile_size), max(2, tile_size * 0.3))


def spawn_drones(world, liveMap, numDrones=NUM_DRONES):
    """Drones side by side, one cell each, centred on the bottom row."""
    first_x = (world.width - numDrones) // 2
    y = world.height - 1
    return [Drone(i, DRONE_COLORS[i % len(DRONE_COLORS)], (first_x + i, y), liveMap) for i in range(numDrones)]
