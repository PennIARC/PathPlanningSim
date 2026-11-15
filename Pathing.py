GRID_WIDTH = 16    # cells across the field
GRID_HEIGHT = 60   # cells along the field; drones start at the bottom and work upwards
CELL_SIZE_FT = 5   # 80 ft x 300 ft field


class Cell:
    """One grid cell. Colouring in the visualizer is derived from these params."""

    __slots__ = ("droneAllocationIndex", "searched", "isObstacle")

    def __init__(self):
        self.droneAllocationIndex = None  # index of the drone assigned to this cell, None if unassigned
        self.searched = False
        self.isObstacle = False


class Grid:
    """2D field grid. cells[y][x] is a Cell object.

    `version` goes up on every set_cell() so the visualizer knows when to redraw.
    After editing a Cell directly, bump it yourself with touch().
    """

    def __init__(self, width=GRID_WIDTH, height=GRID_HEIGHT):
        self.width = width
        self.height = height
        self.cells = [[Cell() for _ in range(width)] for _ in range(height)]
        self.version = 0

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def cell(self, x, y):
        return self.cells[y][x]

    def set_cell(self, x, y, **params):
        """Sets cell params by name, e.g. set_cell(3, 2, searched=True)."""
        cell = self.cells[y][x]
        for name, value in params.items():
            setattr(cell, name, value)
        self.version += 1

    def touch(self):
        self.version += 1


def split_into_columns(width, numDrones):
    """Splits the field's width into one vertical strip per drone, as evenly as possible.

    Returns a list of x-index lists, e.g. width 8 and 4 drones -> [[0, 1], [2, 3], [4, 5], [6, 7]].
    """
    bounds = [round(i * width / numDrones) for i in range(numDrones + 1)]
    return [list(range(bounds[i], bounds[i + 1])) for i in range(numDrones)]


def boustrophedon_path(columnXs, height, startX):
    """Back-and-forth sweep of one vertical strip, from the bottom row to the top row.

    Each row is crossed fully, then the path moves up one row and crosses back
    the other way. The first row starts from whichever side of the strip is
    closer to startX. Returns a list of (x, y) cells, each next to the one before.
    """
    if not columnXs:
        return []
    leftToRight = abs(startX - columnXs[0]) <= abs(startX - columnXs[-1])
    path = []
    for y in range(height - 1, -1, -1):
        xs = columnXs if leftToRight else columnXs[::-1]
        path.extend((x, y) for x in xs)
        leftToRight = not leftToRight
    return path


def path_from(start, sweep):
    """Full path for a drone: its start cell, a straight walk to the sweep's first cell, then the sweep."""
    path = [tuple(start)]
    if not sweep:
        return path
    x, y = path[0]
    tx, ty = sweep[0]
    while x != tx:
        x += 1 if tx > x else -1
        path.append((x, y))
    while y != ty:
        y += 1 if ty > y else -1
        path.append((x, y))
    return path + sweep[1:] if path[-1] == sweep[0] else path + sweep
