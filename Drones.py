import math

from palette import DRONE_COLORS

NUM_DRONES = 4
SIGHT_RANGE = 1  # cells seen in every direction (including diagonals), plus the cell under the drone
CORNER_RADIUS = 0.3  # how far along each cell a trail's corner starts curving, as a fraction of a cell
TRAIL_LANES = 3      # most trail lines drawn side by side between two cells; further passes overlap these
TRAIL_SPACING = 0.22 # gap between side-by-side trail lines, as a fraction of a cell


def round_corners(points, radius, samples=8):
    """Replaces each sharp corner of a polyline with a short curve.

    The curve starts `radius` before the corner and ends `radius` after it.
    Right-angle corners (every corner on a grid path) become true quarter
    circles, so they all come out the same shape. Returns the new list of points.
    """
    if len(points) < 3:
        return list(points)
    out = [points[0]]
    for a, p, b in zip(points, points[1:], points[2:]):
        ax, ay, bx, by = a[0] - p[0], a[1] - p[1], b[0] - p[0], b[1] - p[1]
        la, lb = (ax * ax + ay * ay) ** 0.5, (bx * bx + by * by) ** 0.5
        if la == 0 or lb == 0 or abs(ax * by - ay * bx) < 1e-9:  # straight on (or a repeat): nothing to round
            out.append(p)
            continue
        r = min(radius, la / 2, lb / 2)
        p1 = (p[0] + ax / la * r, p[1] + ay / la * r)
        p2 = (p[0] + bx / lb * r, p[1] + by / lb * r)
        rightAngle = abs(ax * bx + ay * by) < 1e-9
        cx, cy = p1[0] + p2[0] - p[0], p1[1] + p2[1] - p[1]  # centre of the quarter circle
        for i in range(samples + 1):
            t = i / samples
            if rightAngle:
                c, s = math.cos(t * math.pi / 2), math.sin(t * math.pi / 2)
                out.append((cx + (p1[0] - cx) * c + (p2[0] - cx) * s, cy + (p1[1] - cy) * c + (p2[1] - cy) * s))
            else:
                out.append(((1 - t) ** 2 * p1[0] + 2 * t * (1 - t) * p[0] + t * t * p2[0],
                            (1 - t) ** 2 * p1[1] + 2 * t * (1 - t) * p[1] + t * t * p2[1]))
    out.append(points[-1])
    return out


class Drone:
    """One drone on the grid. All positions are (x, y) cell indices."""

    def __init__(self, index, color, position, obstacles, passCounts):
        self.index = index
        self.color = color
        self.position = tuple(position)  # (x, y) cell the drone is in

        self.obstacles = obstacles    # set of (x, y) obstacle cells seen so far, shared by every drone
        self.passCounts = passCounts  # {(cell, cell): times any drone has flown between those two cells}, shared

        self.seenCells = set()      # (x, y) of cells this drone has seen
        self.assignedCells = set()  # (x, y) of cells this drone is assigned to search
        self.visited = {self.position}  # (x, y) of cells this drone has flown over
        self.trail = [self.position]    # every cell the drone has been in, oldest first
        self.trailLanes = [None]  # for each trail entry, how many earlier flights there were along the step into it

        # Planning state, managed by Pathing.Planner
        self.targets = set()    # this drone's own cells still to fly over
        self.stops = []         # cells taken over from other drones; flown over first, in this order
        self.route = []         # cell-by-cell steps to the cell it is heading for
        self.detouring = False  # the current route is longer than a single step along the sweep
        self.inTransit = True   # still on the way from the start cell to the first target
        self.blockedTicks = 0   # ticks in a row spent waiting behind another drone
        self.yielding = True    # when blocked: this drone is the one that has to give way
        self.makingWay = False  # the current route just moves this drone out of another drone's way
        self.ticksSinceProgress = 0  # ticks since a target or stop was last ticked off

    def finished(self):
        return not self.targets and not self.stops

    def mark_visited(self):
        """Ticks the current cell off this drone's targets and stops."""
        self.visited.add(self.position)
        if self.position in self.targets or self.position in self.stops:
            self.targets.discard(self.position)
            if self.position in self.stops:
                self.stops.remove(self.position)
            self.inTransit = False
            self.ticksSinceProgress = 0

    def advance(self):
        """Moves one cell along the route."""
        previous = self.position
        self.position = self.route.pop(0)
        self.trail.append(self.position)
        step = (min(previous, self.position), max(previous, self.position))
        self.trailLanes.append(self.passCounts.get(step, 0))
        self.passCounts[step] = self.passCounts.get(step, 0) + 1
        self.blockedTicks = 0
        self.mark_visited()

    def wait(self):
        """Stays put for a tick because another drone is in the way."""
        self.blockedTicks += 1

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

    def _step_shift(self, i, tile_size):
        """Sideways shift, in pixels, of the line for the step from trail[i - 1] to trail[i].

        A step between two cells flown once runs through their centres. Flown
        twice (by any drones, in either direction), the two lines straddle the
        centre; three times, they use the centre and both sides; a fourth
        flight onwards reuses those three positions.
        """
        a, b = self.trail[i - 1], self.trail[i]
        lanes = min(self.passCounts.get((min(a, b), max(a, b)), 1), TRAIL_LANES)
        return (self.trailLanes[i] % TRAIL_LANES % lanes - (lanes - 1) / 2) * TRAIL_SPACING * tile_size

    def _trail_points(self, origin, tile_size):
        """Pixel points of the trail, with each step shifted sideways into its own lane."""
        points = []
        last = len(self.trail) - 1
        for i, cell in enumerate(self.trail):
            cx, cy = self._cell_center(cell, origin, tile_size)
            # Each step in or out of this cell as (direction of travel, sideways shift)
            steps = []
            if i > 0:
                steps.append(((cell[0] - self.trail[i - 1][0], cell[1] - self.trail[i - 1][1]), self._step_shift(i, tile_size)))
            if i < last:
                steps.append(((self.trail[i + 1][0] - cell[0], self.trail[i + 1][1] - cell[1]), self._step_shift(i + 1, tile_size)))

            def shifted(direction, shift, along=0.0):
                # A sideways shift moves a left-right step up or down, and an up-down step left or right
                sx, sy = (0, shift) if direction[0] else (shift, 0)
                return (cx + sx + direction[0] * along, cy + sy + direction[1] * along)

            if not steps:
                points.append((cx, cy))
            elif len(steps) == 1:
                points.append(shifted(*steps[0]))
            else:
                (dirIn, shiftIn), (dirOut, shiftOut) = steps
                if bool(dirIn[0]) != bool(dirOut[0]):   # a corner: meet where the two shifted lines cross
                    sx = shiftIn if dirIn[1] else shiftOut
                    sy = shiftIn if dirIn[0] else shiftOut
                    points.append((cx + sx, cy + sy))
                elif shiftIn == shiftOut:
                    points.append(shifted(dirIn, shiftIn))
                elif dirIn == dirOut:                   # straight on, changing lane across the cell
                    points.append(shifted(dirIn, shiftIn, -tile_size / 4))
                    points.append(shifted(dirOut, shiftOut, tile_size / 4))
                else:                                   # turning back: a small U from one lane to the other
                    points.append(shifted(dirIn, shiftIn))
                    points.append(shifted(dirOut, shiftOut))
        return points

    def draw_trail(self, surface, origin, tile_size, width=1):
        """Draws line segments through every cell the drone has been in, with slightly curved corners."""
        import pygame

        if len(self.trail) < 2:
            return
        points = round_corners(self._trail_points(origin, tile_size), tile_size * CORNER_RADIUS)
        pygame.draw.lines(surface, self.color, False, points, width)
        # A dot on every point fills the notches pygame leaves where thick segments meet at an angle
        for point in points:
            pygame.draw.circle(surface, self.color, point, width / 2)

    def draw(self, surface, origin, tile_size):
        """Draws the drone as a circle centred on its cell.

        origin is the top-left pixel of the grid on `surface`; tile_size is the
        pixel size of one cell on that surface.
        """
        import pygame

        pygame.draw.circle(surface, self.color, self._cell_center(self.position, origin, tile_size), max(2, tile_size * 0.3))


def spawn_drones(world, obstacles, passCounts, numDrones=NUM_DRONES):
    """Drones side by side, one cell each, centred on the bottom row.

    `obstacles` (seen obstacle cells) and `passCounts` (flights between each pair of cells) are shared by all of them.
    """
    first_x = (world.width - numDrones) // 2
    y = world.height - 1
    return [Drone(i, DRONE_COLORS[i % len(DRONE_COLORS)], (first_x + i, y), obstacles, passCounts) for i in range(numDrones)]
