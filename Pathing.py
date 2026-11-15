import collections
import heapq

GRID_WIDTH = 30    # cells across the field
GRID_HEIGHT = 60   # cells along the field; drones start at the bottom and work upwards

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

    Each drone sweeps its strip row by row from the bottom (the bous sweep).
    `targets` is the set of its own cells still to fly over and `stops` is an
    ordered list of cells it took over from other drones, which come first.
    Whenever it needs somewhere to go it heads for whichever end of the
    unvisited cells in its current row is closer, so it sweeps a row from
    either side. `route` is the cell-by-cell way there, found with A* around
    every obstacle any drone has seen. Obstacle cells are dropped as they are
    discovered.

    A route longer than one step is a detour, and it may cross cells that
    belong to another drone. For each such cell:
      - the owner gets there first: nothing changes, the cell is just flown through
      - both would get there on the same tick: the owner goes first and the
        detouring drone waits a turn (handled by resolve_moves)
      - the owner would get there later: the detouring drone takes the cell over
        and it becomes one of its stops
    The walk from a drone's start cell to its strip never takes cells.
    """

    REROUTE_AFTER = 2  # ticks a drone that has to give way waits behind another drone before finding a way round it
    GIVE_UP_AFTER = 8  # ticks blocked with no way round before the cell it was heading for is skipped
    STALL_FACTOR = 3   # a cell not reached within this many times (width + height) ticks is skipped

    def __init__(self, grid, drones, obstacles):
        self.grid = grid            # the live map; holds which drone each cell is assigned to
        self.drones = drones
        self.obstacles = obstacles  # shared set of (x, y) obstacle cells the drones have seen so far

    def assign(self):
        """Gives each drone a vertical strip of the field; every cell in it becomes a target."""
        columns = split_into_columns(self.grid.width, len(self.drones))
        for drone, columnXs in zip(self.drones, columns):
            for x in columnXs:
                for y in range(self.grid.height):
                    self.grid.set_cell(x, y, droneAllocationIndex=drone.index)
                    drone.assignedCells.add((x, y))
                    drone.targets.add((x, y))
            drone.mark_visited()

    # ---------------- Planning

    def plan(self):
        """Makes sure every drone has a usable route to the next cell it should fly over."""
        for x, y in self.obstacles:  # a cell found to be an obstacle no longer belongs to any drone
            index = self.grid.cells[y][x].droneAllocationIndex
            if index is not None:
                self.drones[index].assignedCells.discard((x, y))
                self.grid.set_cell(x, y, droneAllocationIndex=None)
        for _ in range(len(self.drones) + 1):  # taking a cell changes its owner's plan, so go round again
            changedOthers = False
            for drone in self.drones:
                changedOthers |= self._plan_drone(drone)
            if not changedOthers:
                break

    def _plan_drone(self, drone):
        """Returns True if this took cells from another drone, whose plan then needs another look."""
        drone.targets -= self.obstacles
        drone.stops = [c for c in drone.stops if c not in self.obstacles]
        if drone.makingWay:  # stepping aside for another drone: finish that before anything else
            if drone.route and drone.blockedTicks < self.REROUTE_AFTER and not any(c in self.obstacles for c in drone.route):
                return False
            drone.makingWay = False
            drone.route = []
        if drone.finished():
            drone.route = []
            return False

        if drone.route and drone.ticksSinceProgress > self.STALL_FACTOR * (self.grid.width + self.grid.height):
            self._drop(drone, drone.route[-1])  # safety net: never let one cell hold a drone up forever
            drone.ticksSinceProgress = 0
            return self._route_to_next_goal(drone)

        # In a head-on meeting only the drone that gives way looks for a way round; the other holds its line
        if drone.route and drone.blockedTicks >= (self.REROUTE_AFTER if drone.yielding else self.GIVE_UP_AFTER):
            return self._route_around_drone(drone)

        if drone.route:
            goal = drone.route[-1]
            goalWanted = goal == drone.stops[0] if drone.stops else goal in drone.targets
            if goalWanted and not any(c in self.obstacles for c in drone.route):
                return False
        return self._route_to_next_goal(drone)

    def _next_leg(self, position, stops, targets):
        """Picks the next cell to head for and returns the A* route to it; [] if there is nowhere left to go.

        Stops come first, in order. After that the goal is one end of the
        unvisited cells in the lowest unfinished row: whichever end is fewer
        steps away (the left one on a tie). Cells that cannot be reached at all
        are removed from `stops` / `targets` as they are found.
        """
        while stops:
            route = astar(self.grid, position, stops[0], self.obstacles)
            if route:
                return route
            stops.pop(0)
        while targets:
            row = max(y for _, y in targets)
            xs = [x for x, y in targets if y == row]
            ends = [(min(xs), row), (max(xs), row)]
            if ends[0] == ends[1]:
                ends.pop()
            routes = []
            for end in ends:
                route = astar(self.grid, position, end, self.obstacles)
                if route:
                    routes.append(route)
                else:
                    targets.discard(end)  # walled in by known obstacles: skip it
            if routes:
                return min(routes, key=len)
        return []

    def _set_route(self, drone, route):
        """Returns True if the new route took cells from another drone."""
        drone.route = route
        drone.detouring = len(route) > 1
        drone.blockedTicks = 0
        return self._take_cells(drone) if drone.detouring and not drone.inTransit else False

    def _route_to_next_goal(self, drone):
        return self._set_route(drone, self._next_leg(drone.position, drone.stops, drone.targets))

    def _drop(self, drone, cell):
        drone.targets.discard(cell)
        if cell in drone.stops:
            drone.stops.remove(cell)

    def _route_around_drone(self, drone):
        """The drone has been stuck behind another drone: go round it, or skip the cell if it was heading there."""
        blockedCell, goal = drone.route[0], drone.route[-1]
        blocker = next((d for d in self.drones if d.position == blockedCell), None)
        parked = blocker is not None and blocker.finished() and not blocker.route
        if parked and self._make_way(blocker, drone):
            drone.blockedTicks = 0  # a finished drone was sitting in the way: it moves aside and this one carries on
            return False
        if blockedCell == goal:
            self._drop(drone, goal)
            return self._route_to_next_goal(drone)

        route = astar(self.grid, drone.position, goal, self.obstacles | {blockedCell})
        if route:
            changedOthers = self._set_route(drone, route)
            drone.detouring = True
            return changedOthers

        # No way round, e.g. a one-cell-wide gap: the drone giving way backs out to let the other through
        if blocker is not None and not parked and drone.yielding and self._make_way(drone, blocker):
            return False
        if parked or drone.blockedTicks >= self.GIVE_UP_AFTER:
            self._drop(drone, goal)
            return self._route_to_next_goal(drone)
        return False  # nowhere to go yet: keep waiting

    def _make_way(self, mover, other):
        """Sends `mover` to the nearest free cell that is off `other`'s route. Returns False if there is none."""
        avoid = set(other.route) | {other.position}
        occupied = {d.position for d in self.drones if d is not mover}
        cameFrom = {mover.position: None}
        queue = collections.deque([mover.position])
        while queue:
            cell = queue.popleft()
            if cell != mover.position and cell not in avoid:
                path = []
                while cell != mover.position:
                    path.append(cell)
                    cell = cameFrom[cell]
                mover.route = path[::-1]
                mover.detouring = True
                mover.makingWay = True
                mover.blockedTicks = 0
                return True
            for dx, dy in NEIGHBOURS:
                nxt = (cell[0] + dx, cell[1] + dy)
                if self.grid.in_bounds(*nxt) and nxt not in self.obstacles and nxt not in occupied and nxt not in cameFrom:
                    cameFrom[nxt] = cell
                    queue.append(nxt)
        return False

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
            if cell not in owner.targets and cell not in owner.stops:
                continue  # the owner has already been there, or is not going: just fly through
            if ownerIndex not in etas:
                etas[ownerIndex] = self._arrival_ticks(owner, len(drone.route))
            ownerEta = etas[ownerIndex].get(cell)
            if ownerEta is not None and ownerEta <= i + 1:
                continue  # the owner gets there first, or on the same tick (it then goes first and this drone waits)

            self._drop(owner, cell)
            owner.assignedCells.discard(cell)
            if owner.route and owner.route[-1] == cell:
                owner.route = []
            drone.assignedCells.add(cell)
            self.grid.set_cell(cell[0], cell[1], droneAllocationIndex=drone.index)
            taken.append(cell)
            changedOthers = True
        drone.stops[0:0] = taken  # required stops, in the order the detour reaches them
        return changedOthers

    def _arrival_ticks(self, drone, horizon):
        """How many ticks from now the drone expects to reach each cell ahead of it, if nothing holds it up.

        Plays the drone's own goal-picking forward for `horizon` ticks. Cells it
        would only reach after that are left out.
        """
        ticks = {}
        count = 0
        position = drone.position
        stops, targets = list(drone.stops), set(drone.targets)
        leg = drone.route
        while leg and count < horizon:
            for cell in leg:
                count += 1
                ticks.setdefault(cell, count)
                targets.discard(cell)
                if cell in stops:
                    stops.remove(cell)
            position = leg[-1]
            leg = self._next_leg(position, stops, targets)
        return ticks

    # ---------------- Moving

    def resolve_moves(self):
        """Works out which drones may take their next step this tick. Returns the set that may.

        A drone has to wait if its next cell is being entered by a drone with
        right of way, is occupied by a drone that is staying put, or would mean
        swapping places. Drones making a single step along their sweep have
        right of way over detouring ones; ties go to the lower index.
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
