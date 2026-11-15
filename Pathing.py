GRID_WIDTH = 150   # cells (300 ft along the field at 2 ft per cell)
GRID_HEIGHT = 40   # cells (80 ft across the field at 2 ft per cell)
CELL_SIZE_FT = 2


class Cell:
    """One grid cell. Colouring in the visualizer is derived from these params."""

    __slots__ = ("droneAllocationIndex", "searched", "isObstacle")

    def __init__(self):
        self.droneAllocationIndex = None  # index of the drone assigned to this cell, None if unassigned
        self.searched = False
        self.isObstacle = False


class Grid:
    """2D field grid. cells[y][x] is a Cell object.

    The visualizer only redraws cells listed in `dirty`, so change cells through
    set_cell(), or call mark_dirty() after editing a Cell directly.
    """

    def __init__(self, width=GRID_WIDTH, height=GRID_HEIGHT):
        self.width = width
        self.height = height
        self.cells = [[Cell() for _ in range(width)] for _ in range(height)]
        self.dirty = set()  # (x, y) of cells changed since the visualizer last drew them

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def cell(self, x, y):
        return self.cells[y][x]

    def set_cell(self, x, y, **params):
        """Sets cell params by name, e.g. set_cell(3, 2, searched=True), and flags the cell for redraw."""
        cell = self.cells[y][x]
        for name, value in params.items():
            setattr(cell, name, value)
        self.dirty.add((x, y))

    def mark_dirty(self, x, y):
        self.dirty.add((x, y))

    def mark_all_dirty(self):
        self.dirty.update((x, y) for y in range(self.height) for x in range(self.width))
