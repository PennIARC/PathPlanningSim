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

        self.seenMines = set()      # (x, y) of mines this drone has seen
        self.assignedMines = set()  # (x, y) of mines this drone is currently assigned to

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
        """Looks at the real map around the drone and records what it sees on the live map.

        Only touches live-map cells whose state actually changes, so calling this
        every step costs no redraws while the drone sits still.
        """
        for x, y in self.visible_cells(world):
            isObstacle = world.cells[y][x].isObstacle
            if isObstacle:
                self.obstacles.add((x, y))
            known = liveMap.cells[y][x]
            if not known.searched or known.isObstacle != isObstacle:
                liveMap.set_cell(x, y, searched=True, isObstacle=isObstacle)

    def draw(self, surface, origin, tile_size):
        """Draws the drone as a circle centred on its cell.

        origin is the top-left pixel of the grid on `surface`; tile_size is the
        pixel size of one cell on that surface. pygame is imported here so the
        rest of this module works without it when the sim runs headless.
        """
        import pygame

        cx = origin[0] + (self.position[0] + 0.5) * tile_size
        cy = origin[1] + (self.position[1] + 0.5) * tile_size
        pygame.draw.circle(surface, self.color, (cx, cy), max(2, tile_size / 2))


def spawn_drones(world, liveMap, numDrones=NUM_DRONES):
    """Drones stacked one cell each on the left edge, centred vertically."""
    first_y = (world.height - numDrones) // 2
    return [Drone(i, DRONE_COLORS[i % len(DRONE_COLORS)], (0, first_y + i), liveMap) for i in range(numDrones)]
