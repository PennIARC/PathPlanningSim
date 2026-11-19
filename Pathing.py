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


def astar(grid, start, goal, blocked, flyer=None):
    """Shortest path from start to goal, never entering a cell in `blocked`.

    A* is a shortest-path search that explores cells in order of steps taken so
    far plus the straight-line steps still to go. Returns the cells to step
    through, ending with goal and not including start; [] if start is goal;
    None if there is no way through.

    Ties between equally short paths are settled in two steps, and neither
    ever makes a path longer. First the path with the fewest turns wins.
    Then, if `flyer` (the index of the drone the path is for) is given, the
    one that runs over the most cells another drone has flown and this one
    has not, so a second pair of eyes sees them.
    """
    if start == goal:
        return []
    if goal in blocked or not grid.in_bounds(*goal):
        return None

    def remaining(cell):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    def same_eyes(cell):
        """1 unless another drone has flown this cell and the flyer has not."""
        if flyer is None:
            return 0
        visitedBy = grid.cells[cell[1]][cell[0]].visitedBy
        return 0 if visitedBy and flyer not in visitedBy else 1

    never = (float("inf"), 0, 0)
    # A state is a cell plus the direction it was entered from (an index into NEIGHBOURS, -1 at the start),
    # since the number of turns so far depends on which way the path is heading.
    best = {(start, -1): (0, 0, 0)}  # state -> (steps, turns, cells that were not new eyes), compared in that order
    cameFrom = {}
    frontier = [(remaining(start), 0, 0, 0, start, -1)]
    while frontier:
        _, turns, sameEyes, cost, current, heading = heapq.heappop(frontier)
        if current == goal:
            path, state = [], (current, heading)
            while state[0] != start:
                path.append(state[0])
                state = cameFrom[state]
            return path[::-1]
        if (cost, turns, sameEyes) > best[(current, heading)]:
            continue
        for direction, (dx, dy) in enumerate(NEIGHBOURS):
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.in_bounds(*nxt) or nxt in blocked:
                continue
            reached = (cost + 1, turns + (heading not in (-1, direction)), sameEyes + same_eyes(nxt))
            if reached < best.get((nxt, direction), never):
                best[(nxt, direction)] = reached
                cameFrom[(nxt, direction)] = (current, heading)
                heapq.heappush(frontier, (cost + 1 + remaining(nxt), reached[1], reached[2], cost + 1, nxt, direction))
    return None


class Planner:
    """Decides where every drone goes each tick.

    Each drone sweeps its strip in lanes: up one column of cells, across one
    cell, down the next (the bous sweep). Lanes run the length of the field,
    so a drone turns twice per lane and not twice per row. `targets` is the
    set of its own cells still to fly over and `stops` is an ordered list of
    cells it took over from other drones, which come first. Whenever it needs
    somewhere to go it scores the ends of every unbroken up-and-down run of
    unvisited cells: A* steps to get there, plus LANE_COST for each lane the
    end lies beyond the earliest lane the drone still has cells in. The
    cheapest end wins. So it finishes a lane before starting the next, enters
    a lane from whichever end is nearer, and goes back for cells it had to
    leave when it next passes close to them.
    `route` is the cell-by-cell way there, found with A* around every obstacle
    any drone has seen. Obstacle cells are dropped as they are discovered.

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
    LANE_COST = 3      # extra steps a drone will fly to finish cells in earlier lanes before starting one lane further on.
                       # Tuned by measurement on the 30 x 60 field: see the Parameters table in the README.
    LANE_ORDER = "pairs"    # which way each drone works across its strip: "pairs" has neighbours 0+1 and 2+3 sweep
                            # towards each other so their unflown cells meet; "outward" has all four sweep away from the middle
    HANDOVER_WEIGHT = 1.0   # cells a drone is leaving behind go to another drone if that drone's trip to them, times
                            # this, is shorter than the owner's own trip (0 = never hand over)
    HANDOVER_ALONG = 0      # in that trip a step left or right costs 1 and a step up or down costs this, so cells go to
                            # drones close by sideways, not ones far off sideways that happen to be level (1 = plain steps).
                            # 0 measured fastest on the default map (0.1 and 0.25 were slower and left more stray cells)
    REBALANCE_EVERY = 1     # ticks between checks that share work out between drones (0 = never)
    REBALANCE_GAP = 1       # two neighbours' remaining work must differ by more than this many cells before cells move
    HELP_MIN = 6            # a finished drone only helps a drone that has at least this many cells left
    SIDE_LANE_MIN = 3       # a detour's up-and-down stretch over old ground moves one lane over, at a cost of two steps,
                            # if that flies it over at least this many of the drone's unvisited cells (0 = never)

    def __init__(self, grid, drones, obstacles):
        self.grid = grid            # the live map; holds which drone each cell is assigned to
        self.drones = drones
        self.obstacles = obstacles  # shared set of (x, y) obstacle cells the drones have seen so far
        # First x of every drone's strip but the leftmost, as the field is first split. Set here, not in assign(),
        # because stepping back in the visualizer builds a new Planner without assigning again.
        self.boundaries = [xs[0] for xs in split_into_columns(grid.width, len(drones))[1:]]
        # +1 if the drone works across its strip from left to right, -1 if from right to left
        if self.LANE_ORDER == "pairs":
            self.sweep = [1 if i % 2 == 0 else -1 for i in range(len(drones))]
        else:
            self.sweep = [-1 if i < len(drones) / 2 else 1 for i in range(len(drones))]

    def _lane(self, flyer, cell):
        """How far along the drone's sweep a cell's lane is. Lower numbers are flown first."""
        return self.sweep[flyer] * cell[0]

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

    def plan(self, tick=0):
        """Makes sure every drone has a usable route to the next cell it should fly over."""
        for x, y in self.obstacles:  # a cell found to be an obstacle no longer belongs to any drone
            index = self.grid.cells[y][x].droneAllocationIndex
            if index is not None:
                drone = self.drones[index]
                drone.assignedCells.discard((x, y))
                self._drop(drone, (x, y))
                self.grid.set_cell(x, y, droneAllocationIndex=None)
        if self.REBALANCE_EVERY and tick % self.REBALANCE_EVERY == 0:
            self._rebalance()
        for _ in range(len(self.drones) + 1):  # taking a cell changes its owner's plan, so go round again
            changedOthers = False
            for drone in self.drones:
                changedOthers |= self._plan_drone(drone)
            changedOthers |= self._settle_head_ons()
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

    def _next_leg(self, position, stops, targets, flyer):
        """Picks the next cell to head for and returns the A* route to it; [] if there is nowhere left to go.

        Stops come first, in order. After that the goal is the cheapest end of
        any unbroken up-and-down run of unvisited cells, where the cost is the
        A* steps to reach it plus LANE_COST per lane it lies beyond the
        earliest lane with cells left. Ends that cannot be reached at all are
        removed from `stops` / `targets` as they are found.
        """
        while stops:
            route = astar(self.grid, position, stops[0], self.obstacles, flyer)
            if route:
                return self._use_side_lanes(position, route, stops, targets)
            stops.pop(0)
        best = self._best_end(position, targets, flyer)
        if best is None:
            return []
        return self._use_side_lanes(position, best[1], stops, targets)

    def _use_side_lanes(self, position, route, stops, targets):
        """Moves up-and-down stretches of a detour onto the lane to the left or right when that covers new cells.

        A detour often has to cross cells already flown, e.g. along a finished
        lane to get round the end of a wall. If the same stretch one lane over
        is all unvisited cells of this drone, the route steps over to that
        lane, flies the stretch there and steps back: two extra steps, and
        that part of the lane never needs its own visit. Where the route was
        about to turn that way anyway the swap costs nothing and is always made.
        """
        if not self.SIDE_LANE_MIN or len(route) < 2:
            return route
        path = [position] + route
        end = len(path) - 1
        while end > 0:  # work backwards so earlier indexes stay valid as the path is edited
            start = end
            while start > 0 and path[start - 1][0] == path[end][0]:
                start -= 1
            best = None  # (cells gained minus extra steps, first index, last index, lane offset, extra steps)
            for dx in (-1, 1):
                i = start
                while i <= end:
                    if (path[i][0] + dx, path[i][1]) not in targets:
                        i += 1
                        continue
                    j = i
                    while j < end and (path[j + 1][0] + dx, path[j + 1][1]) in targets:
                        j += 1
                    turnsThere = j + 1 < len(path) and path[j + 1] == (path[j][0] + dx, path[j][1])
                    skipped = path[i + 1:j + 1] if turnsThere else path[i + 1:j]
                    extra = 0 if turnsThere else 2
                    gain = j - i + 1 - turnsThere  # the turn already flew over the last of them
                    wanted = any(cell in targets or cell in stops for cell in skipped)
                    if j > i and not wanted and gain >= (1 if turnsThere else self.SIDE_LANE_MIN):
                        if best is None or gain - extra > best[0]:
                            best = (gain - extra, i, j, dx, extra)
                    i = j + 1
            if best is not None:
                _, i, j, dx, extra = best
                side = [(x + dx, y) for x, y in path[i:j + 1]]
                path[i + 1:j + (0 if extra else 1)] = side if extra else side[:-1]
            end = start - 1
        return path[1:]

    def _best_end(self, position, targets, flyer):
        """(cost, route, cell) for the cheapest end of any run of unvisited cells; None if none can be reached."""
        while targets:
            first = min(self._lane(flyer, cell) for cell in targets)  # the earliest lane that still has cells in it

            def beyond(end):
                return self.LANE_COST * (self._lane(flyer, end) - first)

            # Try ends in order of the least they could possibly cost, and stop once no later one can win
            ordered = sorted((self._straight_steps(position, end) + beyond(end), end) for end in self._run_ends(targets))
            best = None
            for leastCost, end in ordered:
                if best is not None and leastCost >= best[0]:
                    break
                route = astar(self.grid, position, end, self.obstacles, flyer)
                if not route:
                    targets.discard(end)  # walled in by known obstacles: skip it
                    continue
                cost = len(route) + beyond(end)
                if best is None or cost < best[0]:
                    best = (cost, route, end)
            if best is not None:
                return best
        return None

    def _steps_through_seen(self, position, limit, seenOnly=True):
        """Steps from `position` to every cell within `limit` steps, moving only through cells already seen.

        The last step may land on an unseen cell, so cells bordering the seen area are included.
        With seenOnly off, unseen cells count as free, which is how A* treats them.
        """
        steps = {position: 0}
        queue = collections.deque([position])
        while queue:
            cell = queue.popleft()
            if steps[cell] >= limit:
                continue
            for dx, dy in NEIGHBOURS:
                nxt = (cell[0] + dx, cell[1] + dy)
                if not self.grid.in_bounds(*nxt) or nxt in self.obstacles or nxt in steps:
                    continue
                steps[nxt] = steps[cell] + 1
                if not seenOnly or self.grid.cells[nxt[1]][nxt[0]].searched:
                    queue.append(nxt)
        return steps

    def _sideways_trip(self, position):
        """Cost from `position` to every reachable cell, around known obstacles, with unseen cells counted as free.

        A step left or right costs 1 and a step up or down costs HANDOVER_ALONG,
        so the cost is mostly how far away a cell is sideways.
        """
        cost = {position: 0}
        heap = [(0, position)]
        while heap:
            c, cell = heapq.heappop(heap)
            if c > cost[cell]:
                continue
            for dx, dy in NEIGHBOURS:
                nxt = (cell[0] + dx, cell[1] + dy)
                if not self.grid.in_bounds(*nxt) or nxt in self.obstacles:
                    continue
                nc = c + (1 if dx else self.HANDOVER_ALONG)
                if nc < cost.get(nxt, float("inf")):
                    cost[nxt] = nc
                    heapq.heappush(heap, (nc, nxt))
        return cost

    @staticmethod
    def _straight_steps(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def _run_ends(targets):
        """Both ends of every unbroken up-and-down run of cells in `targets`."""
        lanes = collections.defaultdict(list)
        for x, y in targets:
            lanes[x].append(y)
        ends = set()
        for x, ys in lanes.items():
            ys.sort()
            for i, y in enumerate(ys):
                if i == 0 or ys[i - 1] != y - 1:
                    ends.add((x, y))
                if i == len(ys) - 1 or ys[i + 1] != y + 1:
                    ends.add((x, y))
        return ends

    def _set_route(self, drone, route):
        """Returns True if the new route took cells from another drone."""
        drone.route = route
        drone.detouring = len(route) > 1
        drone.blockedTicks = 0
        return self._take_cells(drone) if drone.detouring and not drone.inTransit else False

    def _route_to_next_goal(self, drone):
        handedOver = self._hand_over_skipped(drone)
        return self._set_route(drone, self._next_leg(drone.position, drone.stops, drone.targets, drone.index)) or handedOver

    def _hand_over_skipped(self, drone):
        """A drone about to start a later lane gives the cells it is leaving behind to any drone that is closer to them.

        The cells in earlier lanes are split into connected groups. For each group the
        trip is the cheapest way from a drone to the group's nearest cell,
        around known obstacles, with unseen cells counted as free, where a step
        up or down costs only HANDOVER_ALONG of a sideways step (_sideways_trip),
        so a drone far off sideways does not win just by being level. So while a
        way in may still exist the owner's trip stays short and it keeps the
        cells; once the walls are known and another drone is better placed,
        they change hands. Returns True if any did.
        """
        if not self.HANDOVER_WEIGHT or drone.stops or drone.inTransit or drone.makingWay:
            return False
        here = self._lane(drone.index, drone.position)
        below = {cell for cell in drone.targets if self._lane(drone.index, cell) < here}
        if not below:
            return False
        best = self._best_end(drone.position, drone.targets, drone.index)
        if best is None or self._lane(drone.index, best[2]) <= here:
            return False  # not moving on to a later lane yet
        never = float("inf")
        ownSteps = self._sideways_trip(drone.position)
        others = [(d, self._sideways_trip(d.position))
                  for d in self.drones if d is not drone and not d.inTransit and not d.makingWay]
        handedOver = False
        for group in self._connected_groups(below & drone.targets):
            ownTrip = min(ownSteps.get(cell, never) for cell in group)
            trip, _, receiver = min(((min(steps.get(cell, never) for cell in group), d.index, d) for d, steps in others),
                                    default=(never, 0, None))
            if trip * self.HANDOVER_WEIGHT < ownTrip:
                self._transfer(group, drone, receiver)
                handedOver = True
        return handedOver

    @staticmethod
    def _connected_groups(cells):
        """Splits a set of cells into groups whose cells touch side to side or top to bottom."""
        left = set(cells)
        groups = []
        while left:
            stack = [left.pop()]
            group = set(stack)
            while stack:
                x, y = stack.pop()
                for dx, dy in NEIGHBOURS:
                    nxt = (x + dx, y + dy)
                    if nxt in left:
                        left.discard(nxt)
                        group.add(nxt)
                        stack.append(nxt)
            groups.append(group)
        return groups

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

        route = astar(self.grid, drone.position, goal, self.obstacles | {blockedCell}, drone.index)
        if route:
            changedOthers = self._set_route(drone, route)
            drone.detouring = True
            return changedOthers

        # No way round, e.g. a one-cell-wide gap: the drone giving way backs out to let the other through.
        # If it is boxed in with nowhere to back out to, the other drone moves aside for it instead.
        if blocker is not None and not parked and drone.yielding:
            if self._make_way(drone, blocker) or self._make_way(blocker, drone):
                drone.blockedTicks = 0
                return False
        if parked or drone.blockedTicks >= self.GIVE_UP_AFTER:
            self._drop(drone, goal)
            return self._route_to_next_goal(drone)
        return False  # nowhere to go yet: keep waiting

    def _settle_head_ons(self):
        """Sorts out pairs of drones about to fly into each other, before either loses a tick waiting.

        The one without right of way goes round the other if it can, else backs
        out of its way, else (boxed in) the other backs out instead. Returns
        True if any plan changed.
        """
        changed = False
        for a in self.drones:
            for b in self.drones:
                headOn = a.index < b.index and a.route and b.route and a.route[0] == b.position and b.route[0] == a.position
                if not headOn:
                    continue
                first, second = sorted((a, b), key=lambda d: (d.detouring, d.index))  # `first` has right of way
                if second.makingWay or first.makingWay:
                    continue
                goal = second.route[-1]
                route = astar(self.grid, second.position, goal, self.obstacles | {first.position}, second.index) if goal != first.position else None
                if route:
                    self._set_route(second, route)
                    second.detouring = True
                    changed = True
                elif self._make_way(second, first) or self._make_way(first, second):
                    changed = True
        return changed

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

    # ---------------- Sharing work out between drones

    @staticmethod
    def _load(drone):
        """Rough count of the work a drone has left: cells to fly over, plus any long trip it is on."""
        return len(drone.targets) + len(drone.stops) + (len(drone.route) if drone.detouring else 0)

    def _transfer(self, cells, source, receiver):
        """Moves cells from one drone's sweep to another's."""
        for cell in cells:
            self._drop(source, cell)
            source.assignedCells.discard(cell)
            receiver.targets.add(cell)
            receiver.assignedCells.add(cell)
            self.grid.set_cell(cell[0], cell[1], droneAllocationIndex=receiver.index)
        if source.route and source.route[-1] in cells:
            source.route = []
        receiver.inTransit = False

    def _rebalance(self):
        """Evens out the work: finished drones help the busiest one, and neighbours shift their shared boundary."""
        helped = set()  # drones given cells in this pass; nobody takes work from them straight away
        for drone in self.drones:
            if drone.finished() and not drone.makingWay:
                self._help_busiest(drone, helped)
        loads = {drone: self._load(drone) for drone in self.drones}  # measured once, so every pair is judged on the same numbers
        for left, right in zip(self.drones, self.drones[1:]):
            if not left.finished() and not right.finished():
                self._shift_boundary(left, right, loads)

    def _help_busiest(self, helper, helped):
        """A finished drone takes over the cells the busiest drone would reach last.

        It takes as many as it can finish, travel included, by the time the
        other drone finishes the rest, so the two end at about the same time.
        """
        busy = [d for d in self.drones if d is not helper and not d.finished() and d not in helped]
        if not busy:
            return
        owner = max(busy, key=lambda d: (self._load(d), -d.index))
        if self._load(owner) < self.HELP_MIN or not owner.targets:
            return
        arrival = self._arrival_ticks(owner, 4 * self._load(owner) + self.grid.width + self.grid.height)
        never = float("inf")
        # Soonest first. Cells the owner is not predicted to reach at all are left with it to sort out.
        order = sorted((cell for cell in owner.targets if cell in arrival), key=lambda cell: (arrival[cell], cell))
        steps = self._steps_through_seen(helper.position, never, seenOnly=False)  # real distances, around known obstacles
        nearest = never  # helper's distance to the closest cell it would take
        travel = [0] * len(order)
        for i in range(len(order) - 1, -1, -1):
            nearest = min(nearest, steps.get(order[i], never))
            travel[i] = nearest
        for keep in range(1, len(order) - 1):
            ownerDone = arrival[order[keep - 1]]  # when the owner would finish the cells it keeps
            if travel[keep] + len(order) - keep <= ownerDone:
                self._transfer(order[keep:], owner, helper)
                helped.add(helper)
                return

    def _shift_boundary(self, left, right, loads):
        """If one of two neighbouring drones has more left to do, the other takes cells from the nearest lane of its strip.

        Cells only move where the two drones' unflown cells meet: the heavier
        drone's lane nearest the lighter one must be next to, or shared with,
        a lane the lighter drone still has cells in. Half the difference in
        work is moved, which leaves the two with the same amount. The cells
        come from one end of that lane as one unbroken piece: next to the
        lighter drone's own piece of the lane if it has one, otherwise from
        the end further from the heavier drone. So a lane is only ever split
        into one part for each drone, and neither has to turn more than once
        to fly its part.
        """
        heavy, light = (left, right) if loads[left] >= loads[right] else (right, left)
        gap = loads[heavy] - loads[light]
        if gap <= self.REBALANCE_GAP:
            return
        towards = 1 if light is right else -1
        lanes = collections.defaultdict(list)
        for x, y in heavy.targets:
            lanes[x].append(y)
        if not lanes:
            return
        edge = max(lanes) if towards == 1 else min(lanes)  # the heavier drone's lane nearest the lighter drone
        lightLanes = {x for x, _ in light.targets}
        if edge not in lightLanes and edge + towards not in lightLanes:
            return  # their unflown cells do not meet
        ys = sorted(lanes[edge])  # from the far end of the field towards the drones' start
        lightYs = [y for x, y in light.targets if x == edge]
        if lightYs:
            if min(lightYs) > ys[-1]:
                ys.reverse()  # the lighter drone holds the near part of this lane: grow that part
        elif heavy.position[0] == edge and heavy.position[1] < (ys[0] + ys[-1]) / 2:
            ys.reverse()  # the heavier drone is flying this lane from the far end: give away the near end
        ahead = {heavy.position} | set(heavy.route[:1])
        piece = [(edge, y) for y in ys[:int(gap // 2)] if (edge, y) not in ahead]  # each cell moved closes the gap by two
        if piece:
            self._transfer(piece, heavy, light)

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
        leg = drone.route or self._next_leg(position, stops, targets, drone.index)  # it may not have planned its next step yet
        while leg and count < horizon:
            for cell in leg:
                count += 1
                ticks.setdefault(cell, count)
                targets.discard(cell)
                if cell in stops:
                    stops.remove(cell)
            position = leg[-1]
            leg = self._next_leg(position, stops, targets, drone.index)
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
