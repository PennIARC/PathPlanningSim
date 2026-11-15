import heapq

GRID_WIDTH = 20    # cells across the field
GRID_HEIGHT = 40   # cells along the field; drones start at the bottom and work upwards

NEIGHBOURS = ((1, 0), (-1, 0), (0, -1), (0, 1))  # drones move one cell left, right, up or down per tick


class Cell:
    """One grid cell. Colouring in the visualizer is derived from these params."""

    __slots__ = ("droneAllocationIndex", "searched", "isObstacle", "visitedBy")

    def __init__(self):
        self.droneAllocationIndex = None  # index of the drone assigned to this cell, None if unassigned
        self.searched = False
        self.isObstacle = False
        self.visitedBy = []  # indices of the drones that have flown over this cell, in order of first visit


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
    """Back-and-forth ("bous") sweep of one vertical strip, from the bottom row to the top row.

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


def adjacent(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def astar(grid, start, goal, blocked):
    """Shortest path from start to goal, never entering a cell in `blocked`.

    A* is a shortest-path search that explores cells in order of steps taken so
    far plus the straight-line steps still to go. Returns the cells to step
    through, ending with goal and not including start; [] if start is goal;
    None if there is no way through.
    """
    if start == goal:
        return []
    if goal in blocked or not grid.in_bounds(*goal):
        return None

    def remaining(cell):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    steps = {start: 0}
    cameFrom = {}
    frontier = [(remaining(start), 0, start)]
    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = cameFrom[current]
            return path[::-1]
        if cost > steps[current]:
            continue
        for dx, dy in NEIGHBOURS:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.in_bounds(*nxt) or nxt in blocked:
                continue
            if cost + 1 < steps.get(nxt, float("inf")):
                steps[nxt] = cost + 1
                cameFrom[nxt] = current
                heapq.heappush(frontier, (cost + 1 + remaining(nxt), cost + 1, nxt))
    return None


class Planner:
    """Decides where every drone goes each tick.

    Each drone has `targets`: the cells it still has to fly over, in order. At
    first that is its bous sweep. Its `route` is the cell-by-cell way to
    targets[0]. While nothing is in the way the route is a single step to the
    next bous cell. When a target turns out to be an obstacle it is dropped and
    A* finds a detour to the next one, around every obstacle any drone has seen.

    A detour may cross cells that belong to another drone. For each such cell:
      - the owner gets there first: nothing changes, the cell is just flown through
      - both would get there on the same tick: the owner goes first and the
        detouring drone waits a turn (handled by resolve_moves)
      - the owner would get there later: the detouring drone takes the cell over,
        and it becomes one of its own targets
    The walk from a drone's start cell to its strip never takes cells.
    """

    REROUTE_AFTER = 2  # ticks a drone that has to give way waits behind another drone before finding a way round it
    GIVE_UP_AFTER = 8  # ticks blocked with no way round before the target is skipped
    STALL_FACTOR = 3   # a target not reached within this many times (width + height) ticks is skipped

    def __init__(self, grid, drones, obstacles):
        self.grid = grid            # the live map; holds which drone each cell is assigned to
        self.drones = drones
        self.obstacles = obstacles  # shared set of (x, y) obstacle cells the drones have seen so far

    def assign(self):
        """Gives each drone a vertical strip of the field and the bous sweep of it as its targets."""
        columns = split_into_columns(self.grid.width, len(self.drones))
        for drone, columnXs in zip(self.drones, columns):
            for x in columnXs:
                for y in range(self.grid.height):
                    self.grid.set_cell(x, y, droneAllocationIndex=drone.index)
                    drone.assignedCells.add((x, y))
            drone.targets = boustrophedon_path(columnXs, self.grid.height, drone.position[0])
            drone.mark_visited()

    # ---------------- Planning

    def plan(self):
        """Makes sure every drone has a usable route to its next target."""
        for _ in range(len(self.drones) + 1):  # taking a cell changes its owner's plan, so go round again
            changedOthers = False
            for drone in self.drones:
                changedOthers |= self._plan_drone(drone)
            if not changedOthers:
                break

    def _plan_drone(self, drone):
        """Returns True if this took cells from another drone, whose plan then needs another look."""
        drone.targets = [t for t in drone.targets if t not in self.obstacles]
        if not drone.targets:
            drone.route = []
            return False

        if drone.ticksSinceProgress > self.STALL_FACTOR * (self.grid.width + self.grid.height):
            drone.targets.pop(0)  # safety net: never let one target hold a drone up forever
            drone.ticksSinceProgress = 0
            return self._route_to_next_target(drone, self.obstacles)

        # In a head-on meeting only the drone that gives way looks for a way round; the other holds its line
        if drone.route and drone.blockedTicks >= (self.REROUTE_AFTER if drone.yielding else self.GIVE_UP_AFTER):
            return self._route_around_drone(drone)

        routeOk = drone.route and drone.route[-1] == drone.targets[0] and not any(c in self.obstacles for c in drone.route)
        if routeOk:
            return False
        return self._route_to_next_target(drone, self.obstacles)

    def _route_to_next_target(self, drone, blocked):
        """Runs A* to the next target, skipping targets that cannot be reached."""
        while drone.targets:
            route = astar(self.grid, drone.position, drone.targets[0], blocked)
            if route:
                drone.route = route
                drone.detouring = len(route) > 1
                drone.blockedTicks = 0
                return self._take_cells(drone) if drone.detouring and not drone.inTransit else False
            drone.targets.pop(0)  # walled in by known obstacles: skip it
        drone.route = []
        return False

    def _route_around_drone(self, drone):
        """The drone has been stuck behind another drone: go round it, or skip the cell if it is the target."""
        blockedCell = drone.route[0]
        if blockedCell == drone.targets[0]:
            drone.targets.pop(0)
            drone.blockedTicks = 0
            return self._route_to_next_target(drone, self.obstacles)

        route = astar(self.grid, drone.position, drone.targets[0], self.obstacles | {blockedCell})
        if route:
            drone.route = route
            drone.detouring = True
            drone.blockedTicks = 0
            return self._take_cells(drone) if not drone.inTransit else False

        blocker = next((d for d in self.drones if d.position == blockedCell), None)
        if (blocker is not None and blocker.finished()) or drone.blockedTicks >= self.GIVE_UP_AFTER:
            drone.targets.pop(0)
            drone.blockedTicks = 0
            return self._route_to_next_target(drone, self.obstacles)
        return False  # no way round yet: keep waiting

    def _take_cells(self, drone):
        """Applies the three cases to every other drone's cell on this drone's detour."""
        changedOthers = False
        taken = []
        etas = {}
        for i, cell in enumerate(drone.route[:-1]):
            ownerIndex = self.grid.cells[cell[1]][cell[0]].droneAllocationIndex
            if ownerIndex is None or ownerIndex == drone.index:
                continue
            owner = self.drones[ownerIndex]
            if cell not in owner.targets:
                continue  # the owner has already been there, or is not going: just fly through
            if ownerIndex not in etas:
                etas[ownerIndex] = self._arrival_ticks(owner)
            ownerEta = etas[ownerIndex].get(cell)
            if ownerEta is not None and ownerEta <= i + 1:
                continue  # the owner gets there first, or on the same tick (it then goes first and this drone waits)

            owner.targets.remove(cell)
            owner.assignedCells.discard(cell)
            if owner.route and owner.route[-1] == cell:
                owner.route = []
            drone.assignedCells.add(cell)
            self.grid.set_cell(cell[0], cell[1], droneAllocationIndex=drone.index)
            taken.append(cell)
            changedOthers = True
        drone.targets[0:0] = taken  # required stops, in the order the detour reaches them
        return changedOthers

    def _arrival_ticks(self, drone):
        """How many ticks from now the drone expects to reach each cell ahead of it, if nothing holds it up."""
        ticks = {}
        count = 0
        position = drone.position
        for cell in drone.route:
            count += 1
            ticks.setdefault(cell, count)
            position = cell
        for target in drone.targets:
            if target == position:
                continue
            if adjacent(position, target):
                leg = [target]
            else:
                leg = astar(self.grid, position, target, self.obstacles)
                if leg is None:
                    continue
            for cell in leg:
                count += 1
                ticks.setdefault(cell, count)
            position = target
        return ticks

    # ---------------- Moving

    def resolve_moves(self):
        """Works out which drones may take their next step this tick. Returns the set that may.

        A drone has to wait if its next cell is being entered by a drone with
        right of way, is occupied by a drone that is staying put, or would mean
        swapping places. Drones on their bous path have right of way over
        detouring ones; ties go to the lower index.
        """
        wants = {d: d.route[0] for d in self.drones if d.route}
        at = {d.position: d for d in self.drones}
        order = sorted(wants, key=lambda d: (d.detouring, d.index))
        rank = {d: i for i, d in enumerate(order)}
        moving = set(wants)
        for drone in self.drones:
            drone.yielding = True
        changed = True
        while changed:
            changed = False
            entering = set()
            for drone in order:
                if drone not in moving:
                    continue
                cell = wants[drone]
                occupant = at.get(cell)
                blocked = cell in entering
                if occupant is not None and occupant is not drone:
                    if occupant in moving and wants[occupant] == drone.position:  # head-on: would swap places
                        blocked = True
                        drone.yielding = rank[drone] > rank[occupant]
                    elif occupant not in moving:
                        blocked = True
                if blocked:
                    moving.discard(drone)
                    changed = True
                else:
                    entering.add(cell)
        return moving
