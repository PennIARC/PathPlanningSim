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

    `flyer` is the index of the drone the path is for. It only settles ties:
    among equally short paths, the one that runs over the most cells another
    drone has flown and this one has not is chosen, so a second pair of eyes
    sees them. It never makes a path longer.
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

    never = (float("inf"), 0)
    best = {start: (0, 0)}  # cell -> (steps, cells on the way that were not new eyes), compared in that order
    cameFrom = {}
    frontier = [(remaining(start), 0, 0, start)]
    while frontier:
        _, sameEyes, cost, current = heapq.heappop(frontier)
        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = cameFrom[current]
            return path[::-1]
        if (cost, sameEyes) > best[current]:
            continue
        for dx, dy in NEIGHBOURS:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.in_bounds(*nxt) or nxt in blocked:
                continue
            reached = (cost + 1, sameEyes + same_eyes(nxt))
            if reached < best.get(nxt, never):
                best[nxt] = reached
                cameFrom[nxt] = current
                heapq.heappush(frontier, (cost + 1 + remaining(nxt), reached[1], cost + 1, nxt))
    return None


class Planner:
    """Decides where every drone goes each tick.

    Each drone sweeps its strip row by row from the bottom (the bous sweep).
    `targets` is the set of its own cells still to fly over and `stops` is an
    ordered list of cells it took over from other drones, which come first.
    Whenever it needs somewhere to go it scores the ends of every unbroken run
    of unvisited cells: A* steps to get there, plus CLIMB_COST for each row the
    end is above the drone. The cheapest end wins. So it sweeps a row from
    whichever side is nearer, finishes what is at or below it before moving
    up, and does not shuttle around a wall to keep rows in strict order.
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
    CLIMB_COST = 3     # extra steps a drone will fly to finish cells at or below its row before moving up one row.
                       # Tuned by measurement on the 30 x 60 field: see the Parameters table in the README.
    BACKTRACK_WEIGHT = 1.0  # a drone about to move up goes back for skipped cells below it if the trip is at most
                            # this many times the rows it still has to climb (0 = never go back early)
    HANDOVER_WEIGHT = 1.0   # cells a drone is leaving behind go to another drone if that drone's trip to them, times
                            # this, is shorter than the owner's own trip (0 = never hand over)
    REBALANCE_EVERY = 1     # ticks between checks that share work out between drones (0 = never)
    REBALANCE_GAP = 1       # two neighbours' remaining work must differ by more than this many cells before cells move
    HELP_MIN = 6            # a finished drone only helps a drone that has at least this many cells left

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

    def _next_leg(self, position, stops, targets, flyer=None):
        """Picks the next cell to head for and returns the A* route to it; [] if there is nowhere left to go.

        Stops come first, in order. After that the goal is the cheapest end of
        any unbroken run of unvisited cells in a row, where the cost is the A*
        steps to reach it plus CLIMB_COST per row it lies above the drone.
        Ends that cannot be reached at all are removed from `stops` /
        `targets` as they are found.
        """
        while stops:
            route = astar(self.grid, position, stops[0], self.obstacles, flyer)
            if route:
                return route
            stops.pop(0)
        best = self._best_end(position, targets, flyer)
        if best is None:
            return []
        if best[2][1] < position[1]:  # about to move up a row: first see if skipped cells below are worth going back for
            goBack = self._route_back_to_skipped(position, targets, flyer)
            if goBack:
                return goBack
        return best[1]

    def _best_end(self, position, targets, flyer=None):
        """(cost, route, cell) for the cheapest end of any run of unvisited cells; None if none can be reached."""
        while targets:
            # Try ends in order of the least they could possibly cost, and stop once no later one can win
            ordered = sorted((self._straight_steps(position, end) + self._climb(position, end), end) for end in self._run_ends(targets))
            best = None
            for leastCost, end in ordered:
                if best is not None and leastCost >= best[0]:
                    break
                route = astar(self.grid, position, end, self.obstacles, flyer)
                if not route:
                    targets.discard(end)  # walled in by known obstacles: skip it
                    continue
                cost = len(route) + self._climb(position, end)
                if best is None or cost < best[0]:
                    best = (cost, route, end)
            if best is not None:
                return best
        return None

    def _route_back_to_skipped(self, position, targets, flyer=None):
        """Route to the nearest skipped cell below the drone, if going back for it now beats leaving it to the end.

        Going back now costs about the trip there and back. Leaving it costs
        about the trip plus the rows still to climb, since the drone then has
        to come all the way down again. So it is worth going back when the trip
        is no longer than BACKTRACK_WEIGHT times the rows left to climb.

        Only cells whose shortest route runs entirely through cells the drones
        have already seen count. While an unseen cell could still be a way in,
        the drone keeps sweeping and finds out.
        """
        if not self.BACKTRACK_WEIGHT:
            return None
        below = [end for end in self._run_ends(targets) if end[1] > position[1]]
        budget = self.BACKTRACK_WEIGHT * (position[1] - min(y for _, y in targets))
        if not below or budget < 1:
            return None
        steps = self._steps_through_seen(position, budget)
        for known, end in sorted((steps[end], end) for end in below if end in steps)[:3]:
            route = astar(self.grid, position, end, self.obstacles, flyer)
            if route and len(route) == known:  # no shorter way through unseen cells, so this route is certain
                return route
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

    @staticmethod
    def _straight_steps(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _climb(self, position, cell):
        return self.CLIMB_COST * max(0, position[1] - cell[1])  # rows go up the field as y gets smaller

    @staticmethod
    def _run_ends(targets):
        """Both ends of every unbroken left-to-right run of cells in `targets`."""
        rows = collections.defaultdict(list)
        for x, y in targets:
            rows[y].append(x)
        ends = set()
        for y, xs in rows.items():
            xs.sort()
            for i, x in enumerate(xs):
                if i == 0 or xs[i - 1] != x - 1:
                    ends.add((x, y))
                if i == len(xs) - 1 or xs[i + 1] != x + 1:
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
        """A drone about to move up gives the cells it is leaving behind to any drone that is closer to them.

        The cells below it are split into connected groups. For each group the
        trip is the fewest steps from a drone to the group's nearest cell,
        around known obstacles, with unseen cells counted as free. So while a
        way in may still exist the owner's trip stays short and it keeps the
        cells; once the walls are known and another drone is better placed,
        they change hands. Returns True if any did.
        """
        if not self.HANDOVER_WEIGHT or drone.stops or drone.inTransit or drone.makingWay:
            return False
        below = {cell for cell in drone.targets if cell[1] > drone.position[1]}
        if not below:
            return False
        best = self._best_end(drone.position, drone.targets, drone.index)
        if best is None or best[2][1] >= drone.position[1]:
            return False  # not moving up yet
        never = float("inf")
        ownSteps = self._steps_through_seen(drone.position, never, seenOnly=False)
        others = [(d, self._steps_through_seen(d.position, never, seenOnly=False))
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
        """If one of two neighbouring drones has more left to do, the other takes cells from the edge of its column.

        The candidates are the heavier drone's cell nearest the lighter drone in
        every row above both of them, where that cell touches the lighter
        drone's cells. Half the difference in work is moved, furthest up the
        field first, which leaves the two with the same amount. Those rows
        simply become one cell wider for the lighter drone.
        """
        heavy, light = (left, right) if loads[left] >= loads[right] else (right, left)
        gap = loads[heavy] - loads[light]
        if gap <= self.REBALANCE_GAP:
            return
        towards = 1 if light is right else -1
        top = min(left.position[1], right.position[1]) - 1  # leave the rows the drones are working on alone
        rows = collections.defaultdict(list)
        for x, y in heavy.targets:
            if y < top:
                rows[y].append(x)
        strip = []
        for y, xs in rows.items():
            x = max(xs) if towards == 1 else min(xs)
            beside = (x + towards, y)
            if self.grid.in_bounds(*beside) and self.grid.cells[y][beside[0]].droneAllocationIndex == light.index:
                strip.append((x, y))
        strip.sort(key=lambda cell: cell[1])  # furthest up the field first
        strip = strip[:int(gap // 2)]         # each cell moved closes the gap by two
        if strip:
            self._transfer(strip, heavy, light)

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
